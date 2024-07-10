#include <Arduino.h>
#include <math.h>
#include <lvgl.h>
#include "constants.h"
#include "display.h"
#include "sensor.h"
#include "ui.h"

LV_IMG_DECLARE(gauge_bg);
LV_IMG_DECLARE(gauge_active);
LV_IMG_DECLARE(needle);

lv_obj_t * activeBoost;
lv_obj_t * boostGauge;
lv_obj_t * needleImg;
lv_obj_t * oilTempLabel;
lv_obj_t * oilPressureLabel;
lv_obj_t * egtLabel;
lv_obj_t * intercoolerLabel;
lv_obj_t * oilTempUnit;
lv_obj_t * oilPressureUnit;
lv_obj_t * egtUnit;
lv_obj_t * intercoolerUnit;
lv_obj_t *fps_label;

// Keep current values of all the sensors as strings, 
// So when a new sensor reading is set, it only updates the display if 
// its different to last time.
int boostPressure;
String oilPressureValue;
String oilTemperatureValue;
String egtValue;
String intercoolerTemperatureValue;

int _needle_angle;

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
  lv_obj_align_to(*unitLabel, *valueLabel, LV_ALIGN_OUT_RIGHT_BOTTOM, 2, -3); // Align units to the bottom left
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
    static uint32_t last_time = 0;
    static uint32_t frame_count = 0;

    frame_count++;
    uint32_t current_time = millis();

    if (current_time - last_time >= 1000) { // Update every second
        uint32_t fps = frame_count;
        frame_count = 0;
        last_time = current_time;

        char fps_text[16];
        snprintf(fps_text, sizeof(fps_text), "FPS: %u", fps);
        lv_label_set_text(fps_label, fps_text);
    }
}

void initUI() {

  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);

  lv_obj_t * background = lv_image_create(lv_screen_active());
  lv_obj_align(background, LV_ALIGN_CENTER, 0, 0);

  /*From variable*/
  lv_image_set_src(background, &gauge_bg);
  lv_obj_set_size(background, 480, 480);

  /*Create an Arc*/
  activeBoost = lv_arc_create(lv_screen_active());
  lv_arc_set_range(activeBoost, 0, 2000); // Set the range of the arc
  lv_obj_set_size(activeBoost, 480, 480);
  lv_arc_set_rotation(activeBoost, 135);
  lv_arc_set_bg_angles(activeBoost, 0, 270);
  lv_arc_set_value(activeBoost, 0);
  lv_obj_center(activeBoost);
  lv_obj_remove_flag(activeBoost, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/

  // Create a style for the active boost indicator
  static lv_style_t style_arc;
  lv_style_init(&style_arc);
  lv_style_set_arc_width(&style_arc, 75);
  lv_style_set_arc_rounded(&style_arc, false);
  lv_style_set_arc_color(&style_arc, lv_color_black()); // Black color for the remaining part
  lv_style_set_arc_image_src(&style_arc, &gauge_active); // Set the background image
  lv_style_set_bg_opa(&style_arc, LV_OPA_COVER); // Ensure the image is fully opaque

  // Apply the style to the arc indicator part
  lv_obj_add_style(activeBoost, &style_arc, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(activeBoost, 0, LV_PART_MAIN);
  lv_obj_remove_style(activeBoost, NULL, LV_PART_KNOB); // Remove the knob

  needleImg = lv_image_create(activeBoost);
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
  createSensorWidget(grid, "Oil Temp.", &oilTempLabel, &oilTempUnit, "°C", 1, 0);
  createSensorWidget(grid, "EGT", &egtLabel, &egtUnit, "°C", 0, 1);
  createSensorWidget(grid, "Intake", &intercoolerLabel, &intercoolerUnit, "°C", 1, 1);

  create_fps_label();

  // Create a timer to update the FPS label
  lv_timer_create(update_fps_label, 10, NULL); // Update every second

}

static void anim_rotate_cb(void *var, int32_t v) {
  lv_image_set_rotation((lv_obj_t *)var, v);
}

void setBoostPressure(double value)
{
    if (needleImg == NULL) return;
    
    int mbar = (int)(value / PSI_BAR_CONVERSION * 1000);
    if (mbar < 0) mbar = 0;
    if (mbar > 2000) mbar = 2000;
    if (boostPressure != mbar) {

      lv_arc_set_value(activeBoost, mbar);
      int needle_angle = (1350 + (int)((mbar * 2700.0) / 2000.0)); // Convert value to angle (0-270) and offset by 135 to match the scale background
      //Serial.println(needle_angle);
      lv_image_set_rotation(needleImg, needle_angle);// Set the angle (LVGL angle is in 1/10 degree units)

      //int arrow_angle = 3600 - map(voc, 0, 150, 0, 1800);
      // lv_anim_t anim_rotate_img;
      // lv_anim_init(&anim_rotate_img);
      // lv_anim_set_var(&anim_rotate_img, needleImg);
      // lv_anim_set_exec_cb(&anim_rotate_img, anim_rotate_cb);
      // lv_anim_set_duration(&anim_rotate_img, 25);
      // lv_anim_set_values(&anim_rotate_img, _needle_angle, needle_angle);
      // lv_anim_start(&anim_rotate_img);
      // _needle_angle = needle_angle;
    }
}

String formatValue(double value, unsigned int decimalPlaces) {
  char format[10];
  snprintf(format, sizeof(format), "%%.%df", decimalPlaces);
  char buffer[16];
  snprintf(buffer, sizeof(buffer), format, value);
  return String(buffer);
}

void setLabelValue(lv_obj_t **valueLabel, lv_obj_t **unitLabel, String value) {
  if (valueLabel == NULL || unitLabel == NULL) return;
  // char charArray[20];
  // value.toCharArray(charArray, sizeof(charArray));
  lv_label_set_text(*valueLabel, value.c_str());
  lv_obj_align_to(*unitLabel, *valueLabel, LV_ALIGN_OUT_RIGHT_BOTTOM, 2, -3); // Align units to the bottom left
}

void setOilTemperature(double value) {
    #if ENABLE_OIL_SENSOR
    if (!isnan(value)) {
      if (oilTempLabel == NULL || oilTempUnit == NULL) return;
      String newOilTemperatureValue = formatValue(value, 0);
      if (oilTemperatureValue != newOilTemperatureValue) {
        setLabelValue(&oilTempLabel, &oilTempUnit, newOilTemperatureValue);
        oilTemperatureValue = newOilTemperatureValue;
      }
    }
  #endif
}

void setOilPressure(double value) {
  #if ENABLE_OIL_SENSOR
    if (!isnan(value)) {           
      if (oilPressureLabel == NULL || oilPressureUnit == NULL) return;
      String newOilPressureValue = formatValue(value, 1);
      if (oilPressureValue != newOilPressureValue) {
        setLabelValue(&oilPressureLabel, &oilPressureUnit, newOilPressureValue);
        oilPressureValue = newOilPressureValue;
      }
    }
  #endif
}

void setEgt(double value) {
  #if ENABLE_EGT_SENSOR
    if (!isnan(value)) {
      if (egtLabel == NULL || egtUnit == NULL) return;
      String newEgtValue = formatValue(value, 0);
      if (egtValue != newEgtValue) {
        setLabelValue(&egtLabel, &egtUnit, newEgtValue);
        egtValue = newEgtValue;
      }
    }
  #endif
}

void setIntercoolerTemperature(double value) {
  #if ENABLE_INTERCOOLER_SENSOR
    if (!isnan(value)) {
      if (intercoolerLabel == NULL || intercoolerUnit == NULL) return;
      String newIntercoolerTemperatureValue = formatValue(value, 0);
      if (intercoolerTemperatureValue != newIntercoolerTemperatureValue) {
        setLabelValue(&intercoolerLabel, &intercoolerUnit, newIntercoolerTemperatureValue);
        intercoolerTemperatureValue = newIntercoolerTemperatureValue;
      } 
    }
  #endif
}

void updateUI(SensorData data) {
            

  

}