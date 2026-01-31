#include <Arduino.h>
#include <math.h>
#include <limits.h>
#include <lvgl.h>
#include "constants.h"
#include "display.h"
#include "sensor.h"
#include "ui.h"

LV_IMG_DECLARE(gauge_bg);
LV_IMG_DECLARE(needle);

lv_obj_t *needleImg;
lv_obj_t *oilTempLabel;
lv_obj_t *oilPressureLabel;
lv_obj_t *egtLabel;
lv_obj_t *intercoolerLabel;
lv_obj_t *oilTempUnit;
lv_obj_t *oilPressureUnit;
lv_obj_t *egtUnit;
lv_obj_t *intercoolerUnit;
lv_obj_t *fps_label;

// Keep current values of all the sensors,
// so UI updates only happen when the displayed value changes.
int boostPressure = -1;
int lastOilPressureTenths = INT_MIN;
int lastOilTemperatureC = INT_MIN;
int lastEgtC = INT_MIN;
int lastIntercoolerTemperatureC = INT_MIN;

void createSensorWidget(lv_obj_t *parent, const char *labelText, lv_obj_t **valueLabel, lv_obj_t **unitLabel, const char *unitText, int col, int row)
{
  // Create styles
  static lv_style_t style_label;
  lv_style_init(&style_label);
  lv_style_set_text_color(&style_label, lv_color_darken(lv_color_white(), 64));
  lv_style_set_text_font(&style_label, &lv_font_montserrat_18);

  static lv_style_t style_value;
  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_white());
  lv_style_set_text_font(&style_value, &lv_font_montserrat_34);

  static lv_style_t style_units;
  lv_style_init(&style_units);
  lv_style_set_text_color(&style_units, lv_color_darken(lv_color_white(), 64));
  lv_style_set_text_font(&style_units, &lv_font_montserrat_18);

  lv_obj_t *container = lv_obj_create(parent);
  lv_obj_set_grid_cell(container, LV_GRID_ALIGN_STRETCH, col, 1, LV_GRID_ALIGN_STRETCH, row, 1);
  lv_obj_set_style_border_width(container, 0, LV_PART_MAIN); // Remove border
  lv_obj_set_style_radius(container, 0, LV_PART_MAIN);       // Remove rounded corners
  lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);

  *valueLabel = lv_label_create(container);
  lv_label_set_text(*valueLabel, "--");
  lv_obj_add_style(*valueLabel, &style_value, 0);
  lv_obj_align(*valueLabel, LV_ALIGN_LEFT_MID, 0, 10); // Align value to the left middle

  lv_obj_t *label = lv_label_create(container);
  lv_label_set_text(label, labelText);
  lv_obj_add_style(label, &style_label, 0);
  lv_obj_align_to(label, *valueLabel, LV_ALIGN_OUT_TOP_LEFT, 0, 0); // Align label to the top left

  *unitLabel = lv_label_create(container);
  lv_label_set_text(*unitLabel, unitText);
  lv_obj_add_style(*unitLabel, &style_units, 0);
  lv_obj_align_to(*unitLabel, *valueLabel, LV_ALIGN_OUT_RIGHT_BOTTOM, 2, -3); // Align units to the right
}

void create_fps_label() {

  static lv_style_t style_label;
  lv_style_init(&style_label);
  lv_style_set_text_color(&style_label, lv_color_darken(lv_color_white(), 64));
  lv_style_set_text_font(&style_label, &lv_font_montserrat_14);

  fps_label = lv_label_create(lv_scr_act());
  lv_label_set_text(fps_label, "FPS: 0");
  lv_obj_add_style(fps_label, &style_label, LV_PART_MAIN);
  lv_obj_align(fps_label, LV_ALIGN_CENTER, 0, 100); // Adjust position as needed
}

void update_fps_label(lv_timer_t * timer) {
    uint32_t fps = getAndResetFlushCount();
    char fps_text[16];
    snprintf(fps_text, sizeof(fps_text), "FPS: %u", (unsigned int)fps);
    lv_label_set_text(fps_label, fps_text);
}

void initUI() {

  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);

  lv_obj_t * background = lv_image_create(lv_screen_active());
  lv_obj_align(background, LV_ALIGN_CENTER, 0, 0);

  /*From variable*/
  lv_image_set_src(background, &gauge_bg);
  lv_obj_set_size(background, 480, 480);

  needleImg = lv_image_create(lv_screen_active());
  lv_image_set_src(needleImg, &needle);
  // As it's aligned to center, need to set the x y to half the width/height of the needle image.
  lv_obj_align(needleImg, LV_ALIGN_CENTER, 116, 0);
  // Set the pivot to the base of the needle
  lv_image_set_pivot(needleImg, 3, 10);
  lv_image_set_rotation(needleImg, 1350);

  // Create a style for the circle
  static lv_style_t style_circle;
  lv_style_init(&style_circle);
  lv_style_set_radius(&style_circle, LV_RADIUS_CIRCLE); // Make it perfectly round
  lv_style_set_bg_color(&style_circle, lv_color_black()); // Black 
  lv_style_set_border_color(&style_circle, lv_color_white()); // White border
  lv_style_set_border_width(&style_circle, 4); // 4px border width
  lv_style_set_shadow_width(&style_circle, 10); // Shadow width
  lv_style_set_shadow_color(&style_circle, lv_color_black()); // Black shadow
  lv_style_set_shadow_spread(&style_circle, 0); // No spread to the shadow
  lv_style_set_shadow_ofs_x(&style_circle, 0); // X offset of shadow
  lv_style_set_shadow_ofs_y(&style_circle, 0); // Y offset of shadow

  // Create a container for the circle
  lv_obj_t * circle = lv_obj_create(lv_screen_active());
  lv_obj_add_style(circle, &style_circle, 0);
  lv_obj_set_size(circle, 280, 280); // Set the size of the circle
  lv_obj_center(circle); // Center the circle on the screen

// Create a 2x2 grid
  static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  lv_obj_t *grid = lv_obj_create(circle);
  lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
  lv_obj_set_size(grid, LV_PCT(80), LV_PCT(80));
  lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN); // Remove border
  lv_obj_set_style_radius(grid, 0, LV_PART_MAIN);       // Remove rounded corners
  lv_obj_align(grid, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_pad_row(grid, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_column(grid, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_all(grid, 0, LV_PART_MAIN);

  // Create sensor widgets
  createSensorWidget(grid, "Oil Press.", &oilPressureLabel, &oilPressureUnit, "bar", 0, 0);
  lv_obj_align_to(oilPressureUnit, oilPressureLabel, LV_ALIGN_OUT_RIGHT_BOTTOM, 25, -3); // Extra offset for "bar"
  createSensorWidget(grid, "Oil Temp.", &oilTempLabel, &oilTempUnit, "°C", 1, 0);
  createSensorWidget(grid, "EGT", &egtLabel, &egtUnit, "°C", 0, 1);
  createSensorWidget(grid, "Intake", &intercoolerLabel, &intercoolerUnit, "°C", 1, 1);

  create_fps_label();

  // Create a timer to update the FPS label every second
  lv_timer_create(update_fps_label, 1000, NULL);

}

void setBoostPressure(double value)
{
    if (needleImg == NULL) return;
    
    int mbar = (int)(value / PSI_BAR_CONVERSION * 1000);
    if (mbar < 0) mbar = 0;
    if (mbar > 2000) mbar = 2000;
    if (boostPressure >= 0 && abs(mbar - boostPressure) < 5) {
      return;
    }
    if (boostPressure != mbar) {

      int needle_angle = (1350 + (int)((mbar * 2700.0) / 2000.0)); // Convert value to angle (0-270) and offset by 135 to match the scale background
      lv_image_set_rotation(needleImg, needle_angle);// Set the angle (LVGL angle is in 1/10 degree units)
      boostPressure = mbar;
    }
}

static void setLabelValue(lv_obj_t *valueLabel, lv_obj_t *unitLabel, const char *value) {
  if (valueLabel == NULL || unitLabel == NULL) return;
  lv_label_set_text(valueLabel, value);
  lv_obj_align_to(unitLabel, valueLabel, LV_ALIGN_OUT_RIGHT_BOTTOM, 2, -3); // Align units to the right
}

void setOilTemperature(double value) {
    #if ENABLE_OIL_SENSOR
    if (!isnan(value)) {
      if (oilTempLabel == NULL || oilTempUnit == NULL) return;
      int rounded = (int)lround(value);
      if (lastOilTemperatureC != rounded) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", rounded);
        setLabelValue(oilTempLabel, oilTempUnit, buf);
        lastOilTemperatureC = rounded;
      }
    }
  #endif
}

void setOilPressure(double value) {
  #if ENABLE_OIL_SENSOR
    if (!isnan(value)) {
      if (oilPressureLabel == NULL || oilPressureUnit == NULL) return;
      int tenths = (int)lround(value * 10.0);
      if (lastOilPressureTenths != tenths) {
        char buf[16];
        int whole = tenths / 10;
        int frac = abs(tenths % 10);
        snprintf(buf, sizeof(buf), "%d.%d", whole, frac);
        lv_label_set_text(oilPressureLabel, buf);
        lv_obj_align_to(oilPressureUnit, oilPressureLabel, LV_ALIGN_OUT_RIGHT_BOTTOM, 26, -3);
        lastOilPressureTenths = tenths;
      }
    }
  #endif
}

void setEgt(double value) {
  #if ENABLE_EGT_SENSOR
    if (!isnan(value)) {
      if (egtLabel == NULL || egtUnit == NULL) return;
      int rounded = (int)lround(value);
      if (lastEgtC != rounded) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", rounded);
        setLabelValue(egtLabel, egtUnit, buf);
        lastEgtC = rounded;
      }
    }
  #endif
}

void setIntercoolerTemperature(double value) {
  #if ENABLE_INTERCOOLER_SENSOR
    if (!isnan(value)) {
      if (intercoolerLabel == NULL || intercoolerUnit == NULL) return;
      int rounded = (int)lround(value);
      if (lastIntercoolerTemperatureC != rounded) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", rounded);
        setLabelValue(intercoolerLabel, intercoolerUnit, buf);
        lastIntercoolerTemperatureC = rounded;
      } 
    }
  #endif
}

void updateUI(SensorData data) {
            

  

}
