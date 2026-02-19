#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include "display.h"
#include "touch.h"

// CST826 touch controller (I2C at 0x38 on Qualia 2.1" display)
#define CST826_I2C_ADDR      0x38
#define CST826_REG_TOUCHES   0x02
#define CST826_REG_TOUCH_DATA 0x03

static lv_indev_t *touch_indev = NULL;

static uint8_t cst826_read_reg(uint8_t reg) {
  Wire.beginTransmission(CST826_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)CST826_I2C_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  (void)indev;

  uint8_t touches = cst826_read_reg(CST826_REG_TOUCHES);
  if (touches > 0 && touches <= 5) {
    // Read 6 bytes of touch data for the first point
    Wire.beginTransmission(CST826_I2C_ADDR);
    Wire.write(CST826_REG_TOUCH_DATA);
    Wire.endTransmission(false);
    uint8_t buf[6];
    Wire.requestFrom((uint8_t)CST826_I2C_ADDR, (uint8_t)6);
    for (int i = 0; i < 6 && Wire.available(); i++) {
      buf[i] = Wire.read();
    }

    int16_t x = ((buf[0] & 0x0F) << 8) | buf[1];
    int16_t y = ((buf[2] & 0x0F) << 8) | buf[3];

    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void initTouch() {
  touch_indev = lv_indev_create();
  lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch_indev, touch_read_cb);

  #if DEBUG
    Serial.println("CST826 touch ready at 0x38");
  #endif
}
