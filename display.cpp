#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <lvgl.h>
#include "constants.h"
#include "display.h"
#include "ui.h"

#ifndef TOUCH_INT_PIN
  #define TOUCH_INT_PIN 0
#endif

#ifndef TOUCH_RST_PIN
  #define TOUCH_RST_PIN 0
#endif

#ifndef GT911_I2C_ADDR
  #define GT911_I2C_ADDR 0x14
#endif

#ifndef TOUCH_ROTATION
  #define TOUCH_ROTATION ROTATION_INVERTED
#endif

static constexpr uint8_t kTouchMaxPoints = 5;

// TFT init/reset SPI is via the Qualia's IO expander (XCA9554).
Arduino_XCA9554SWSPI *expander = new Arduino_XCA9554SWSPI(
  PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI,
  &Wire, 0x3F
);


// RGB (dot-clock) panel timing for the 2.8\" TL028WVC01 480x480 round panel:
// hsync: FP=10, PW=2, BP=10
// vsync: FP=10, PW=6, BP=10  [oai_citation:2‡learn.adafruit.com](https://learn.adafruit.com/adafruit-qualia-esp32-s3-for-rgb666-displays/qualia-rgb666-with-tl028wvc01-2-8-round-display)
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,
  TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_R5,
  TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
  TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,
  // 2.1 inch display
  1 /* hsync_polarity */, 50 /* hsync_front_porch */, 2 /* hsync_pulse_width */, 44 /* hsync_back_porch */,
  1 /* vsync_polarity */, 16 /* vsync_front_porch */, 2 /* vsync_pulse_width */, 18 /* vsync_back_porch */
  // 2.8 inch display
  // 1 /* hsync_polarity */, 10 /* hsync_front_porch */, 2 /* hsync_pulse_width */, 10 /* hsync_back_porch */,
  // 1 /* vsync_polarity */, 10 /* vsync_front_porch */, 6 /* vsync_pulse_width */, 10 /* vsync_back_porch */
  // If you need to force PCLK, some variants support extra args here (left out for maximum compatibility).
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
  // 2.1" 480x480 round display
  TFT_HOR_RES /* width */, TFT_VER_RES /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
  expander, GFX_NOT_DEFINED /* RST */, TL021WVC02_init_operations, sizeof(TL021WVC02_init_operations)
  //expander, GFX_NOT_DEFINED /* RST */, TL028WVC01_init_operations, sizeof(TL028WVC01_init_operations)
);

TAMC_GT911 tp(
  SDA, SCL,
  TOUCH_INT_PIN,
  TOUCH_RST_PIN,
  TFT_HOR_RES, TFT_VER_RES
);

static lv_indev_t *touch_indev = NULL;

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

static inline int16_t clamp_coord(int16_t value, int16_t max_value) {
  if (value < 0) return 0;
  if (value >= max_value) return max_value - 1;
  return value;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  (void)indev;
  tp.read();

  int16_t touch_x[kTouchMaxPoints];
  int16_t touch_y[kTouchMaxPoints];
  uint8_t touch_count = 0;

  if (tp.isTouched && tp.touches > 0) {
    touch_count = tp.touches;
    if (touch_count > kTouchMaxPoints) {
      touch_count = kTouchMaxPoints;
    }

    for (uint8_t i = 0; i < touch_count; i++) {
      int16_t x = clamp_coord(tp.points[i].x, TFT_HOR_RES);
      int16_t y = clamp_coord(tp.points[i].y, TFT_VER_RES);
      touch_x[i] = x;
      touch_y[i] = y;

      #if DEBUG
        Serial.printf("Touch %u: x=%d y=%d size=%d\n", i, x, y, tp.points[i].size);
      #endif
    }

    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = touch_x[0];
    data->point.y = touch_y[0];
    updateTouchDebugDots(touch_count, touch_x, touch_y);
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
    updateTouchDebugDots(0, NULL, NULL);
  }
}

void initDisplay() {
#ifdef GFX_EXTRA_PRE_INIT
    GFX_EXTRA_PRE_INIT();
  #endif

  Wire.begin();
  Wire.setClock(400000);

  // Init Display
  if (!gfx->begin()) {
    // Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(RGB565_BLACK);

  #ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  #endif

  // Backlight (if your board variant defines TFT_BL)
  #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  #endif

}

void initTouch() {
  tp.begin(GT911_I2C_ADDR);
  tp.setRotation(TOUCH_ROTATION);

  touch_indev = lv_indev_create();
  lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch_indev, touch_read_cb);

  #if DEBUG
    Serial.println("GT911 touch ready at 0x14. Touch the screen.");
  #endif
}

void initLVGL() {
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
    bufSize = screenWidth * 80;
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
      if (!disp_draw_buf) {
        bufSize = screenWidth * 40;
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!disp_draw_buf) {
          disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
        }
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

  // lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
  // draw_buf_1 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  // lv_display_set_flush_cb(disp, my_disp_flush);
  // lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
}
