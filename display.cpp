#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "display.h"
#include "ui.h"

Arduino_XCA9554SWSPI *expander = new Arduino_XCA9554SWSPI(
  PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI,
  &Wire, 0x3F);

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,
  TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_R5,
  TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
  TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,
  1 /* hsync_polarity */, 50 /* hsync_front_porch */, 2 /* hsync_pulse_width */, 44 /* hsync_back_porch */,
  1 /* vsync_polarity */, 16 /* vsync_front_porch */, 2 /* vsync_pulse_width */, 18 /* vsync_back_porch */
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
  // 2.1" 480x480 round display
  TFT_HOR_RES /* width */, TFT_VER_RES /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
  expander, GFX_NOT_DEFINED /* RST */, TL021WVC02_init_operations, sizeof(TL021WVC02_init_operations));

/* Change to your screen resolution */
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

#if LV_USE_LOG != 0
  void my_print(lv_log_level_t level, const char *buf) {
    LV_UNUSED(level);
    // Serial.println(buf);
    // Serial.flush();
  }
#endif

uint32_t millis_cb(void) {
  return millis();
}

/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  #ifndef DIRECT_MODE
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  #endif  // #ifndef DIRECT_MODE

  /*Call it to tell LVGL you are ready*/
  lv_disp_flush_ready(disp);
}

void initDisplay() {
#ifdef GFX_EXTRA_PRE_INIT
    GFX_EXTRA_PRE_INIT();
  #endif

  // Init Display
  if (!gfx->begin()) {
    // Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

  #ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  #endif

  lv_init();

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(millis_cb);

  /* register print function for debugging */
  #if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print);
  #endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  #ifdef DIRECT_MODE
    bufSize = screenWidth * screenHeight;
  #else
    bufSize = screenWidth * 40;
  #endif

  #ifdef ESP32
    #if defined(DIRECT_MODE) && defined(RGB_PANEL)
      disp_draw_buf = (lv_color_t *)gfx->getFramebuffer();
    #else   // !DIRECT_MODE
      disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (!disp_draw_buf) {
        // remove MALLOC_CAP_INTERNAL flag try again
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
      }
    #endif  // !DIRECT_MODE
  #else   // !ESP32
    // Serial.println("LVGL draw_buf allocate MALLOC_CAP_INTERNAL failed! malloc again...");
    disp_draw_buf = (lv_color_t *)malloc(bufSize * 2);
  #endif  // !ESP32
  if (!disp_draw_buf) {
    // Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    #ifdef DIRECT_MODE
        lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_DIRECT);
    #else
        lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    #endif
  }
}

void flushDisplay() {
  lv_task_handler(); /* let the GUI do its work */

  #ifdef DIRECT_MODE
    #ifdef RGB_PANEL
      gfx->flush();
    #else
      gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)draw_buf, screenWidth, screenHeight);
    #endif
  #endif  // #ifdef DIRECT_MODE
}