#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <lvgl.h>
#include "display.h"

// TFT init/reset SPI is via the Qualia's IO expander (XCA9554).
Arduino_XCA9554SWSPI *expander = new Arduino_XCA9554SWSPI(
  PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI,
  &Wire, 0x3F
);

// RGB (dot-clock) panel timing for 2.1" round display
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,
  TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_R5,
  TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
  TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,
  // 2.1 inch display timing
  1 /* hsync_polarity */, 50 /* hsync_front_porch */, 2 /* hsync_pulse_width */, 44 /* hsync_back_porch */,
  1 /* vsync_polarity */, 16 /* vsync_front_porch */, 2 /* vsync_pulse_width */, 18 /* vsync_back_porch */
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
  TFT_HOR_RES /* width */, TFT_VER_RES /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
  expander, GFX_NOT_DEFINED /* RST */, TL021WVC02_init_operations, sizeof(TL021WVC02_init_operations)
);

// Frame counter for FPS measurement
static volatile uint32_t flush_count = 0;

uint32_t getAndResetFlushCount() {
  uint32_t count = flush_count;
  flush_count = 0;
  return count;
}

/* Change to your screen resolution */
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

#if LV_USE_LOG != 0
  void my_print(lv_log_level_t level, const char *buf) {
    LV_UNUSED(level);
  }
#endif

uint32_t millis_cb(void) {
  return millis();
}

#ifdef DIRECT_MODE
// ============================================================
// Double-buffered, tear-free flip (DIRECT_MODE)
//
// LVGL renders directly into one of the two PSRAM framebuffers. On the last
// flush of a refresh we hand that buffer to the RGB peripheral, which switches
// to scanning it at the next VSYNC (zero-copy; IDF flushes the cache for us).
// We then block until that VSYNC fires before returning, so LVGL never starts
// rendering the next frame into a buffer that is still being scanned out — the
// other buffer only becomes free once the flip has taken effect. A two-handle
// handshake makes us wait for a VSYNC that occurs *after* the flip request,
// ignoring any stale VSYNC that fired just before it.
// ============================================================
static SemaphoreHandle_t sem_vsync_end = NULL;  // given by ISR after our frame's VSYNC
static SemaphoreHandle_t sem_gui_ready = NULL;   // armed by flush before it waits

static bool IRAM_ATTR on_vsync_event(esp_lcd_panel_handle_t panel,
                                     const esp_lcd_rgb_panel_event_data_t *edata,
                                     void *user_ctx) {
  BaseType_t high_task_awoken = pdFALSE;
  if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
    xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
  }
  return high_task_awoken == pdTRUE;
}
#endif

/* LVGL calls it when a rendered frame is ready to be shown */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  #ifdef DIRECT_MODE
    // Only act on the final flush of the refresh: px_map then holds the fully
    // rendered frame to display. Earlier flushes (disjoint dirty areas) already
    // landed in the same buffer, so there is nothing to do for them.
    if (lv_display_flush_is_last(disp)) {
      gfx->flipFramebuffer((uint16_t *)px_map);  // switch to this buffer at next VSYNC
      if (sem_gui_ready != NULL && sem_vsync_end != NULL) {
        // Drain any leftover give from a previously timed-out wait so we don't
        // sail through the wait below without an actual VSYNC.
        xSemaphoreTake(sem_vsync_end, 0);
        xSemaphoreGive(sem_gui_ready);  // arm the handshake
        // Wait for the flip to take effect. Generous timeout (~5 frames) so a
        // missing/misbehaving VSYNC degrades to tearing instead of a frozen UI.
        xSemaphoreTake(sem_vsync_end, pdMS_TO_TICKS(100));
      }
      flush_count++;
    }
  #else
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    if (lv_display_flush_is_last(disp)) {
      flush_count++;
    }
  #endif

  lv_disp_flush_ready(disp);
}

void initDisplay() {
  #ifdef GFX_EXTRA_PRE_INIT
    GFX_EXTRA_PRE_INIT();
  #endif

  Wire.begin();
  Wire.setClock(400000);

  // Init Display
  if (!gfx->begin()) {
    // Display init failed
  }
  gfx->fillScreen(RGB565_BLACK);

  #ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  #endif

  #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  #endif
}

void initLVGL() {
  lv_init();

  lv_tick_set_cb(millis_cb);

  #if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print);
  #endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  #ifdef DIRECT_MODE
    // Render straight into the panel's two PSRAM framebuffers — no intermediate
    // buffer, no copy on flush. LVGL alternates between them (DIRECT mode keeps
    // both coherent) and we flip on VSYNC for tear-free output.
    bufSize = screenWidth * screenHeight;
    // Hand LVGL the back buffer first: LVGL renders its first frame into the
    // first buffer it is given, and the panel boots up scanning the front
    // buffer, so this keeps the very first render off the live buffer.
    disp_draw_buf = (lv_color_t *)gfx->getBackFramebuffer();
    lv_color_t *disp_draw_buf2 = (lv_color_t *)gfx->getFramebuffer();

    // VSYNC handshake used by my_disp_flush() to wait out each flip. If either
    // semaphore can't be created, skip the callback and run unsynchronized
    // (tearing instead of a NULL deref in the ISR).
    sem_vsync_end = xSemaphoreCreateBinary();
    sem_gui_ready = xSemaphoreCreateBinary();
    if (sem_vsync_end != NULL && sem_gui_ready != NULL) {
      esp_lcd_rgb_panel_event_callbacks_t cbs = {};
      cbs.on_vsync = on_vsync_event;
      esp_lcd_rgb_panel_register_event_callbacks(gfx->getPanelHandle(), &cbs, NULL);
    }
  #else
    bufSize = screenWidth * 80;
    #ifdef ESP32
      disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (!disp_draw_buf) {
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
      }
      if (!disp_draw_buf) {
        bufSize = screenWidth * 40;
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!disp_draw_buf) {
          disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
        }
      }
    #else
      disp_draw_buf = (lv_color_t *)malloc(bufSize * 2);
    #endif
  #endif

  if (!disp_draw_buf) {
    // Buffer allocation failed
  } else {
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    #ifdef DIRECT_MODE
      lv_display_set_buffers(disp, disp_draw_buf, disp_draw_buf2, bufSize * 2, LV_DISPLAY_RENDER_MODE_DIRECT);
    #else
      lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    #endif
  }
}
