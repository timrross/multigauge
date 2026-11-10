#include <Arduino.h>
#include <math.h>
#include <limits.h>
#include <lvgl.h>
#include "sensor_types.h"
#include "display.h"
#include "ui.h"

// Image declarations (in assets folder)
LV_IMG_DECLARE(gauge_active_v2);
LV_IMG_DECLARE(needle);

// Screens
lv_obj_t *splash_screen = NULL;
lv_obj_t *gauge_screen = NULL;

// UI elements
lv_obj_t *needleImg;
lv_obj_t *oilTempValue;
lv_obj_t *oilPressureValue;
lv_obj_t *egtValue;
lv_obj_t *intercoolerValue;
lv_obj_t *oilTempUnit;
lv_obj_t *oilPressureUnit;
lv_obj_t *egtUnit;
lv_obj_t *intercoolerUnit;
lv_obj_t *fps_label;
lv_obj_t *connection_indicator;

// View state
static int current_view = 0;  // 0=grid, 1=oil press, 2=oil temp, 3=EGT, 4=intercooler
static lv_obj_t *circle = NULL;
static lv_obj_t *solo_value_label = NULL;

// Ghost needle (peak boost indicator)
static lv_obj_t *ghostNeedleImg = NULL;
static int peakMbar = 0;
static unsigned long peakTimestamp = 0;
static int ghostAngle = 1350;
#define PEAK_HOLD_MS 5000
// Below this the "peak" is just ADC noise at idle; keep the ghost hidden
// rather than drawing it under the resting needle.
#define GHOST_MIN_MBAR 50

// Last displayed values for change detection
int boostPressure = -1;
// One entry per numeric readout. The order matches the solo views (current_view - 1).
enum { SENSOR_OIL_PRESSURE = 0, SENSOR_OIL_TEMP, SENSOR_EGT, SENSOR_INTAKE, SENSOR_COUNT };
#define FORCE_REDRAW    INT_MIN         // nothing drawn yet for this label
#define NO_VALUE_SHOWN  (INT_MIN + 1)   // "--" is on screen
static int lastShown[SENSOR_COUNT] = { FORCE_REDRAW, FORCE_REDRAW, FORCE_REDRAW, FORCE_REDRAW };
static int8_t lastWarn[SENSOR_COUNT] = { -1, -1, -1, -1 };  // -1 unknown, 0 normal, 1 warning

// Warning thresholds: the number turns red beyond these. Tune them to the engine.
#define WARN_OIL_PRESSURE_LOW_BAR  0.8    // also red with the engine off, like an oil light
#define WARN_OIL_TEMP_HIGH_C       130.0
#define WARN_EGT_HIGH_C            700.0
#define WARN_COLOUR                0xFF2020

void createSensorWidget(lv_obj_t *parent, const char *labelText, lv_obj_t **valueLabel, lv_obj_t **unitLabel, const char *unitText, int col, int row) {
  static lv_style_t style_label;
  static lv_style_t style_value;
  static lv_style_t style_units;
  static bool styles_initialized = false;

  if (!styles_initialized) {
    lv_style_init(&style_label);
    lv_style_set_text_color(&style_label, lv_color_darken(lv_color_white(), 64));
    lv_style_set_text_font(&style_label, &lv_font_montserrat_18);

    lv_style_init(&style_value);
    lv_style_set_text_color(&style_value, lv_color_white());
    lv_style_set_text_font(&style_value, &lv_font_montserrat_34);

    lv_style_init(&style_units);
    lv_style_set_text_color(&style_units, lv_color_darken(lv_color_white(), 64));
    lv_style_set_text_font(&style_units, &lv_font_montserrat_18);

    styles_initialized = true;
  }

  // Outer container fills the grid cell, uses flex column to vertically center content
  lv_obj_t *container = lv_obj_create(parent);
  lv_obj_set_grid_cell(container, LV_GRID_ALIGN_STRETCH, col, 1, LV_GRID_ALIGN_STRETCH, row, 1);
  lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(container, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(container, LV_OBJ_FLAG_EVENT_BUBBLE);
  lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(container, 0, LV_PART_MAIN);

  // Row 1: sensor name
  lv_obj_t *label = lv_label_create(container);
  lv_label_set_text(label, labelText);
  lv_obj_add_style(label, &style_label, 0);

  // Row 2: value + unit inline
  lv_obj_t *val_row = lv_obj_create(container);
  lv_obj_set_size(val_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(val_row, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(val_row, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(val_row, 0, LV_PART_MAIN);
  lv_obj_set_flex_flow(val_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(val_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_column(val_row, 2, LV_PART_MAIN);
  lv_obj_clear_flag(val_row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(val_row, LV_OBJ_FLAG_EVENT_BUBBLE);

  *valueLabel = lv_label_create(val_row);
  lv_label_set_text(*valueLabel, "--");
  lv_obj_add_style(*valueLabel, &style_value, 0);

  *unitLabel = lv_label_create(val_row);
  lv_label_set_text(*unitLabel, unitText);
  lv_obj_add_style(*unitLabel, &style_units, 0);
  // Align to baseline: shift unit up by descent difference between value and unit fonts
  int baseline_offset = lv_font_montserrat_34.base_line - lv_font_montserrat_18.base_line;
  lv_obj_set_style_translate_y(*unitLabel, -baseline_offset, LV_PART_MAIN);
}

void create_fps_label(lv_obj_t *parent) {
  static lv_style_t style_label;
  lv_style_init(&style_label);
  lv_style_set_text_color(&style_label, lv_color_darken(lv_color_white(), 64));
  lv_style_set_text_font(&style_label, &lv_font_montserrat_14);

  fps_label = lv_label_create(parent);
  lv_label_set_text(fps_label, "FPS: 0");
  lv_obj_add_style(fps_label, &style_label, LV_PART_MAIN);
  lv_obj_align(fps_label, LV_ALIGN_CENTER, 0, 100);
}

void create_connection_indicator(lv_obj_t *parent) {
  // Small circle indicator at bottom of screen
  connection_indicator = lv_obj_create(parent);
  lv_obj_set_size(connection_indicator, 12, 12);
  lv_obj_align(connection_indicator, LV_ALIGN_CENTER, 0, 120);
  lv_obj_set_style_radius(connection_indicator, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_border_width(connection_indicator, 0, LV_PART_MAIN);
  // Start as disconnected (red)
  lv_obj_set_style_bg_color(connection_indicator, lv_color_hex(0xFF0000), LV_PART_MAIN);
}

void setConnectionStatus(bool connected) {
  if (connection_indicator == NULL) return;
  // Setting a style forces a redraw, so only do it when the state changes.
  static int8_t lastConnected = -1;
  if (lastConnected == (connected ? 1 : 0)) return;
  lastConnected = connected ? 1 : 0;
  if (connected) {
    lv_obj_set_style_bg_color(connection_indicator, lv_color_hex(0x00FF00), LV_PART_MAIN);
  } else {
    lv_obj_set_style_bg_color(connection_indicator, lv_color_hex(0xFF0000), LV_PART_MAIN);
  }
}

void update_fps_label(lv_timer_t *timer) {
  uint32_t fps = getAndResetFlushCount();
  char fps_text[16];
  snprintf(fps_text, sizeof(fps_text), "FPS: %u", (unsigned int)fps);
  lv_label_set_text(fps_label, fps_text);
}

// ============================================================
// View Switching
// ============================================================

static lv_obj_t *content_wrap = NULL;
static bool animating = false;

#define SLIDE_ANIM_TIME 200
#define SLIDE_DISTANCE  100

static lv_obj_t *createContentWrap() {
  lv_obj_t *wrap = lv_obj_create(circle);
  lv_obj_set_size(wrap, LV_PCT(100), LV_PCT(100));
  lv_obj_center(wrap);
  lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(wrap, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(wrap, 0, LV_PART_MAIN);
  lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(wrap, LV_OBJ_FLAG_EVENT_BUBBLE);
  return wrap;
}

static void createGridContent(lv_obj_t *parent) {
  static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  lv_obj_t *grid = lv_obj_create(parent);
  lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
  lv_obj_set_size(grid, LV_PCT(80), LV_PCT(80));
  lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(grid, 0, LV_PART_MAIN);
  lv_obj_align(grid, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_pad_row(grid, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_column(grid, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_all(grid, 0, LV_PART_MAIN);
  lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(grid, LV_OBJ_FLAG_EVENT_BUBBLE);

  createSensorWidget(grid, "Oil Press.", &oilPressureValue, &oilPressureUnit, "bar", 0, 0);
  createSensorWidget(grid, "Oil Temp.", &oilTempValue, &oilTempUnit, "°C", 1, 0);
  createSensorWidget(grid, "EGT", &egtValue, &egtUnit, "°C", 0, 1);
  createSensorWidget(grid, "Intake", &intercoolerValue, &intercoolerUnit, "°C", 1, 1);
}

static void createSoloContent(lv_obj_t *parent, int sensor_idx) {
  static const char *names[] = {"Oil Pressure", "Oil Temp.", "EGT", "Intake"};
  static const char *units[] = {"bar", "\xC2\xB0""C", "\xC2\xB0""C", "\xC2\xB0""C"};

  lv_obj_t *container = lv_obj_create(parent);
  lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
  lv_obj_center(container);
  lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
  lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(container, LV_OBJ_FLAG_EVENT_BUBBLE);

  // Value + unit row anchored at absolute center
  lv_obj_t *row = lv_obj_create(container);
  lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 6, LV_PART_MAIN);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(row, LV_OBJ_FLAG_EVENT_BUBBLE);
  lv_obj_align(row, LV_ALIGN_CENTER, 0, 0);

  solo_value_label = lv_label_create(row);
  lv_label_set_text(solo_value_label, "--");
  lv_obj_set_style_text_color(solo_value_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(solo_value_label, &lv_font_montserrat_48, LV_PART_MAIN);

  lv_obj_t *unit_label = lv_label_create(row);
  lv_label_set_text(unit_label, units[sensor_idx]);
  lv_obj_set_style_text_color(unit_label, lv_color_darken(lv_color_white(), 64), LV_PART_MAIN);
  lv_obj_set_style_text_font(unit_label, &lv_font_montserrat_24, LV_PART_MAIN);
  // Align to baseline: shift unit up by descent difference between value and unit fonts
  int baseline_offset = lv_font_montserrat_48.base_line - lv_font_montserrat_24.base_line;
  lv_obj_set_style_translate_y(unit_label, -baseline_offset, LV_PART_MAIN);

  // Sensor name positioned above the centered value row
  lv_obj_t *name_label = lv_label_create(container);
  lv_label_set_text(name_label, names[sensor_idx]);
  lv_obj_set_style_text_color(name_label, lv_color_darken(lv_color_white(), 64), LV_PART_MAIN);
  lv_obj_set_style_text_font(name_label, &lv_font_montserrat_24, LV_PART_MAIN);
  lv_obj_align_to(name_label, row, LV_ALIGN_OUT_TOP_MID, 0, -4);
}

static void set_translate_x(void *obj, int32_t x) {
  lv_obj_set_style_translate_x((lv_obj_t *)obj, x, LV_PART_MAIN);
}

static void slide_done_cb(lv_anim_t *a) {
  animating = false;
}

static void switchView(int view, lv_dir_t dir) {
  // Reset label pointers
  oilTempValue = NULL;
  oilPressureValue = NULL;
  egtValue = NULL;
  intercoolerValue = NULL;
  oilTempUnit = NULL;
  oilPressureUnit = NULL;
  egtUnit = NULL;
  intercoolerUnit = NULL;
  solo_value_label = NULL;

  // Reset change detection to force update on next data
  for (int i = 0; i < SENSOR_COUNT; i++) {
    lastShown[i] = FORCE_REDRAW;
    lastWarn[i] = -1;
  }

  // Stop any running animations on existing content before deleting
  if (content_wrap != NULL) {
    lv_anim_delete(content_wrap, set_translate_x);
    lv_obj_del(content_wrap);
    content_wrap = NULL;
  }

  // Create new content
  content_wrap = createContentWrap();
  if (view == 0) {
    createGridContent(content_wrap);
  } else {
    createSoloContent(content_wrap, view - 1);
  }

  // No animation for initial creation
  if (dir == LV_DIR_NONE) {
    return;
  }

  // New content slides in from the swipe direction
  int slide_from = (dir == LV_DIR_LEFT) ? SLIDE_DISTANCE : -SLIDE_DISTANCE;

  animating = true;

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, content_wrap);
  lv_anim_set_values(&a, slide_from, 0);
  lv_anim_set_time(&a, SLIDE_ANIM_TIME);
  lv_anim_set_exec_cb(&a, set_translate_x);
  lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
  lv_anim_set_completed_cb(&a, slide_done_cb);
  lv_anim_start(&a);
}

static void gesture_event_cb(lv_event_t *e) {
  if (animating) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
  if (dir == LV_DIR_LEFT) {
    current_view = (current_view + 1) % 5;
    switchView(current_view, dir);
  } else if (dir == LV_DIR_RIGHT) {
    current_view = (current_view + 4) % 5;
    switchView(current_view, dir);
  }
}

static bool isWarning(int sensor, double value) {
  if (isnan(value)) return false;
  switch (sensor) {
    case SENSOR_OIL_PRESSURE: return value < WARN_OIL_PRESSURE_LOW_BAR;
    case SENSOR_OIL_TEMP:     return value > WARN_OIL_TEMP_HIGH_C;
    case SENSOR_EGT:          return value > WARN_EGT_HIGH_C;
    default:                  return false;
  }
}

// Show a reading on a label: "--" when there is no valid value (NAN), red when it
// is past its warning threshold. Only touches LVGL when what is shown changes.
static void showSensorValue(lv_obj_t *label, int sensor, double value) {
  if (label == NULL) return;

  char buf[16];
  int key;
  if (isnan(value)) {
    snprintf(buf, sizeof(buf), "--");
    key = NO_VALUE_SHOWN;
  } else if (sensor == SENSOR_OIL_PRESSURE) {
    // Oil pressure: one decimal place
    key = (int)lround(value * 10.0);
    snprintf(buf, sizeof(buf), "%d.%d", key / 10, abs(key % 10));
  } else {
    key = (int)lround(value);
    snprintf(buf, sizeof(buf), "%d", key);
  }
  if (key != lastShown[sensor]) {
    lv_label_set_text(label, buf);
    lastShown[sensor] = key;
  }

  int8_t warn = isWarning(sensor, value) ? 1 : 0;
  if (warn != lastWarn[sensor]) {
    lv_obj_set_style_text_color(label, warn ? lv_color_hex(WARN_COLOUR) : lv_color_white(), LV_PART_MAIN);
    lastWarn[sensor] = warn;
  }
}

static void updateSoloView(const SensorData& data) {
  switch (current_view) {
    case 1: showSensorValue(solo_value_label, SENSOR_OIL_PRESSURE, data.oilPressure); break;
    case 2: showSensorValue(solo_value_label, SENSOR_OIL_TEMP, data.oilTemperature); break;
    case 3: showSensorValue(solo_value_label, SENSOR_EGT, data.egt); break;
    case 4: showSensorValue(solo_value_label, SENSOR_INTAKE, data.intercoolerTemperature); break;
    default: break;
  }
}

// ============================================================
// Splash Screen
// ============================================================

void createSplashScreen() {
  splash_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(splash_screen, lv_color_black(), LV_PART_MAIN);

  // Title text
  lv_obj_t *title = lv_label_create(splash_screen);
  lv_label_set_text(title, "MultiGauge");
  lv_obj_set_style_text_color(title, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_34, LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_CENTER, 0, -30);

  // Initializing text
  lv_obj_t *status = lv_label_create(splash_screen);
  lv_label_set_text(status, "Initializing...");
  lv_obj_set_style_text_color(status, lv_color_darken(lv_color_white(), 64), LV_PART_MAIN);
  lv_obj_set_style_text_font(status, &lv_font_montserrat_18, LV_PART_MAIN);
  lv_obj_align(status, LV_ALIGN_CENTER, 0, 20);
}

void showSplashScreen() {
  if (splash_screen == NULL) {
    createSplashScreen();
  }
  lv_screen_load(splash_screen);
}

void showGaugeScreen() {
  if (gauge_screen != NULL) {
    lv_screen_load(gauge_screen);
  }
}

// ============================================================
// Gauge Screen
// ============================================================

void createGaugeScreen() {
  gauge_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(gauge_screen, lv_color_black(), LV_PART_MAIN);

  lv_obj_t *background = lv_image_create(gauge_screen);
  lv_obj_align(background, LV_ALIGN_CENTER, 0, 0);
  lv_image_set_src(background, &gauge_active_v2);
  lv_obj_set_size(background, 480, 480);

  ghostNeedleImg = lv_image_create(gauge_screen);
  lv_image_set_src(ghostNeedleImg, &needle);
  lv_obj_align(ghostNeedleImg, LV_ALIGN_CENTER, 116, 0);
  lv_image_set_pivot(ghostNeedleImg, 3, 10);
  lv_image_set_rotation(ghostNeedleImg, 1350);
  lv_obj_set_style_image_opa(ghostNeedleImg, LV_OPA_40, LV_PART_MAIN);
  lv_obj_add_flag(ghostNeedleImg, LV_OBJ_FLAG_HIDDEN);

  needleImg = lv_image_create(gauge_screen);
  lv_image_set_src(needleImg, &needle);
  lv_obj_align(needleImg, LV_ALIGN_CENTER, 116, 0);
  lv_image_set_pivot(needleImg, 3, 10);
  lv_image_set_rotation(needleImg, 1350);

  // Create center circle
  static lv_style_t style_circle;
  lv_style_init(&style_circle);
  lv_style_set_radius(&style_circle, LV_RADIUS_CIRCLE);
  lv_style_set_bg_color(&style_circle, lv_color_black());
  lv_style_set_border_color(&style_circle, lv_color_white());
  lv_style_set_border_width(&style_circle, 4);
  lv_style_set_shadow_width(&style_circle, 10);
  lv_style_set_shadow_color(&style_circle, lv_color_black());
  lv_style_set_shadow_spread(&style_circle, 0);
  lv_style_set_shadow_ofs_x(&style_circle, 0);
  lv_style_set_shadow_ofs_y(&style_circle, 0);

  circle = lv_obj_create(gauge_screen);
  lv_obj_add_style(circle, &style_circle, 0);
  lv_obj_set_size(circle, 280, 280);
  lv_obj_center(circle);
  lv_obj_clear_flag(circle, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(circle, LV_OBJ_FLAG_EVENT_BUBBLE);

  // Create initial grid view
  content_wrap = createContentWrap();
  createGridContent(content_wrap);

  // Register gesture handler for swipe navigation
  lv_obj_add_event_cb(gauge_screen, gesture_event_cb, LV_EVENT_GESTURE, NULL);

  create_fps_label(gauge_screen);
  create_connection_indicator(gauge_screen);
  lv_timer_create(update_fps_label, 1000, NULL);
}

// ============================================================
// Public Init Function
// ============================================================

void initUI() {
  // Show splash screen immediately
  showSplashScreen();

  // Create gauge screen in background (will be shown later)
  createGaugeScreen();
}

static void updatePeakBoost(int mbar) {
  unsigned long now = millis();
  if (mbar >= peakMbar) {
    peakMbar = mbar;
    peakTimestamp = now;
  } else if (now - peakTimestamp >= PEAK_HOLD_MS) {
    peakMbar = mbar;
    peakTimestamp = now;
  }
}

static void updateGhostNeedle() {
  if (ghostNeedleImg == NULL) return;
  if (peakMbar < GHOST_MIN_MBAR) {
    if (!lv_obj_has_flag(ghostNeedleImg, LV_OBJ_FLAG_HIDDEN)) {
      lv_obj_add_flag(ghostNeedleImg, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }
  if (lv_obj_has_flag(ghostNeedleImg, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_clear_flag(ghostNeedleImg, LV_OBJ_FLAG_HIDDEN);
  }
  int angle = 1350 + (int)((peakMbar * 2700.0) / 2000.0);
  if (angle != ghostAngle) {
    lv_image_set_rotation(ghostNeedleImg, angle);
    ghostAngle = angle;
  }
}

void setBoostPressure(double value) {
  if (needleImg == NULL) return;

  // No valid reading (NAN): drop the needle to zero rather than leave it frozen.
  int mbar = isnan(value) ? 0 : (int)(value / PSI_BAR_CONVERSION * 1000);
  if (mbar < 0) mbar = 0;
  if (mbar > 2000) mbar = 2000;
  updatePeakBoost(mbar);
  updateGhostNeedle();
  if (boostPressure >= 0 && abs(mbar - boostPressure) < 5) {
    return;
  }
  if (boostPressure != mbar) {
    int needle_angle = (1350 + (int)((mbar * 2700.0) / 2000.0));
    lv_image_set_rotation(needleImg, needle_angle);
    boostPressure = mbar;
  }
}

void setOilTemperature(double value) {
  #if ENABLE_OIL_SENSOR
    showSensorValue(oilTempValue, SENSOR_OIL_TEMP, value);
  #endif
}

void setOilPressure(double value) {
  #if ENABLE_OIL_SENSOR
    showSensorValue(oilPressureValue, SENSOR_OIL_PRESSURE, value);
  #endif
}

void setEgt(double value) {
  #if ENABLE_EGT_SENSOR
    showSensorValue(egtValue, SENSOR_EGT, value);
  #endif
}

void setIntercoolerTemperature(double value) {
  #if ENABLE_INTERCOOLER_SENSOR
    showSensorValue(intercoolerValue, SENSOR_INTAKE, value);
  #endif
}

// A NAN in SensorData means "no fresh, valid reading" (sensor fault, stale frame or
// lost CAN link). It is passed through so the readout shows "--".
void updateUI(const SensorData& data) {
  #if ENABLE_BOOST_SENSOR
    setBoostPressure(data.boostPressure);
  #endif

  if (current_view == 0) {
    // Grid view: update all sensor labels
    setOilTemperature(data.oilTemperature);
    setOilPressure(data.oilPressure);
    setEgt(data.egt);
    setIntercoolerTemperature(data.intercoolerTemperature);
  } else {
    // Solo view: update the displayed sensor
    updateSoloView(data);
  }
}
