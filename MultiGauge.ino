
#include <Adafruit_MAX31855.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_GFX_Library.h>
#include <Ewma.h>
#include <lvgl.h>

#define PSI_BAR_CONVERSION 14.5038

// Pins for reading sensors
#define OIL_PRESSURE_PIN MOSI
#define BOOST_PRESSURE_PIN A0

#define STEINHART_A 0.00135721593521515000000
#define STEINHART_B 0.00024467275139054400000
#define STEINHART_C 0.00000028439705482433300

#define SEALEVELPRESSURE_HPA (1013.25)

/* These values give a boost sensor reading in psi */
#define BOOST_COEFFICIENT 12.864
#define BOOST_INTERCEPT -12.877

/*Set to your screen resolution*/
#define TFT_HOR_RES 480
#define TFT_VER_RES 480

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

LV_IMG_DECLARE(gauge_bg);
LV_IMG_DECLARE(gauge_active);
LV_IMG_DECLARE(needle);

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

// Sensors
Adafruit_MAX31855 thermocouple(SCK, SS, MISO);  // EGT Sensor module
Adafruit_BME280 bme;                            // atmosphere pressure/temp module

// Set up Exponential Weighted moving average for all the sensor readings.
Ewma boostFilter(0.1); 
Ewma oilTempFilter(0.1); 
Ewma oilPressureFilter(0.1); 
Ewma egtFilter(0.1);

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

static lv_obj_t * active_boost;
static lv_obj_t * boost_gauge;
static lv_obj_t * needle_img;
static lv_obj_t * count_label;
static lv_obj_t * boost_pressure_label;
static lv_obj_t * oil_temp_label;
static lv_obj_t * oil_pressure_label;
static lv_obj_t * egt_label;

void init_ui() {

  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);

  lv_obj_t * background = lv_image_create(lv_screen_active());
  lv_obj_align(background, LV_ALIGN_CENTER, 0, 0);

  /*From variable*/
  lv_image_set_src(background, &gauge_bg);
  lv_obj_set_size(background, 480, 480);

  static lv_style_t style;
  lv_style_init(&style);
  lv_style_set_bg_color(&style, lv_color_black());
  lv_style_set_border_width(&style, 0);
  lv_style_set_text_color(&style, lv_color_white());
  lv_style_set_text_letter_space(&style, 5);
  lv_style_set_text_line_space(&style, 20);
  lv_style_set_text_font(&style, &lv_font_montserrat_36);
  lv_style_set_text_align(&style, LV_TEXT_ALIGN_RIGHT);

  // Initialize a style variable
  static lv_style_t transparent;
  lv_style_init(&transparent);
  // Set the background opacity to transparent
  lv_style_set_opa(&transparent, LV_OPA_TRANSP);

  /*Create an Arc*/
  active_boost = lv_arc_create(lv_screen_active());
  lv_arc_set_range(active_boost, 0, 2000); // Set the range of the arc
  lv_obj_set_size(active_boost, 480, 480);
  lv_arc_set_rotation(active_boost, 135);
  lv_arc_set_bg_angles(active_boost, 0, 270);
  lv_arc_set_value(active_boost, 0);
  lv_obj_center(active_boost);
  lv_obj_remove_flag(active_boost, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/

  // Create a style for the active boost indicator
  static lv_style_t style_arc;
  lv_style_init(&style_arc);
  lv_style_set_arc_width(&style_arc, 75);
  lv_style_set_arc_rounded(&style_arc, false);
  lv_style_set_arc_color(&style_arc, lv_color_black()); // Black color for the remaining part
  lv_style_set_arc_image_src(&style_arc, &gauge_active); // Set the background image
  lv_style_set_bg_opa(&style_arc, LV_OPA_COVER); // Ensure the image is fully opaque

  // Apply the style to the arc indicator part
  lv_obj_add_style(active_boost, &style_arc, LV_PART_INDICATOR);
  //lv_obj_add_style(arc, &transparent, LV_PART_MAIN);
  lv_obj_set_style_arc_width(active_boost, 0, LV_PART_MAIN);
  //lv_obj_remove_style(arc, NULL, LV_PART_MAIN); // Remove the knob
  lv_obj_remove_style(active_boost, NULL, LV_PART_KNOB); // Remove the knob


  /* Add a scale just for the nice needle */
  boost_gauge = lv_scale_create(lv_screen_active());
  lv_obj_set_size(boost_gauge, 480, 480);
  lv_scale_set_mode(boost_gauge, LV_SCALE_MODE_ROUND_INNER);
  lv_obj_center(boost_gauge);
  // Apply the style to the object
  lv_obj_add_style(boost_gauge, &transparent, LV_PART_ITEMS);
  lv_obj_add_style(boost_gauge, &transparent, LV_PART_INDICATOR);
  lv_scale_set_range(boost_gauge, 0, 2000);

  lv_scale_set_angle_range(boost_gauge, 270);
  lv_scale_set_rotation(boost_gauge, 135);

  needle_img = lv_image_create(boost_gauge);
  lv_image_set_src(needle_img, &needle);
  // As it's aligned to center, need to set the x y to half the width/height of the needle image.
  lv_obj_align(needle_img, LV_ALIGN_CENTER, 116, 0);
  // Set the pivot to the base of the needle
  lv_image_set_pivot(needle_img, 3, 10);
  oil_temp_label = lv_label_create(lv_scr_act());
  lv_obj_set_width(oil_temp_label, LV_SIZE_CONTENT);
  lv_label_set_text(oil_temp_label, "--");
  lv_obj_align(oil_temp_label, LV_ALIGN_CENTER, 0, -40);
  lv_obj_add_style(oil_temp_label, &style, LV_PART_MAIN);

  oil_pressure_label = lv_label_create(lv_scr_act());
  lv_obj_set_width(oil_pressure_label, LV_SIZE_CONTENT);
  lv_label_set_text(oil_pressure_label, "--");
  lv_obj_align(oil_pressure_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_style(oil_pressure_label, &style, LV_PART_MAIN);

  egt_label = lv_label_create(lv_scr_act());
  lv_obj_set_width(egt_label, LV_SIZE_CONTENT);
  lv_label_set_text(egt_label, "--");
  lv_obj_align(egt_label, LV_ALIGN_CENTER, 0, 40);
  lv_obj_add_style(egt_label, &style, LV_PART_MAIN);

}

static void set_needle_value(int32_t v)
{
  lv_arc_set_value(active_boost, v);
  lv_scale_set_image_needle_value(boost_gauge, needle_img, v);
}

/** Sensor vars */

/* Vars for boost pressure */
float easing_factor = 0.1;
// Sensor readings
float boost_pressure = 0.0;

/* vars for EGT */
float egt = 0.0;

/* vars for Intercooler Temp */
float intercooler_temp = 0.0;

/* vars for atmos sensor */
float atmos_temp = 0.0;
float atmos_pressure = 0.0;

/* Vars for oil pressure */
enum PulseType {
    UNKNOWN,
    DIAGNOSTIC,
    TEMPERATURE,
    PRESSURE
};
volatile double oil_temp;
volatile double oil_pressure;
volatile byte oil_sensor_status;
volatile bool sequenceComplete = false;
volatile PulseType lastPulse = UNKNOWN;
volatile long pulseDuration = 0;
volatile long inputDuration = 0;
volatile long startTime = 0;

// PWM reading from oil temp/pressure sensor
void IRAM_ATTR oilSensorPWMInterrupt() {
  bool pinState = (GPIO.in & (1ULL << OIL_PRESSURE_PIN)) != 0;
  unsigned long currentTime = micros();
  if (pinState) {
    // Start of rising edge of next pulse.
    pulseDuration = currentTime - startTime;
    startTime = currentTime;
    if (pulseDuration < 920) {
      // if the pulse duration is too short, then it's probably jitter/noise ignore it.
      return;
    }
    if (pulseDuration < 1150 && lastPulse == UNKNOWN) {
      // We just saw a diagnostic pulse, so the next pulse will be a temperature.
      lastPulse = DIAGNOSTIC;
      double val = (1024.0 / pulseDuration) * inputDuration;
      if (val >= 231.00 && val <= 281.00) {
        oil_sensor_status = 1;
      } else if (val >= 359.00 && val <= 409.00) {
        oil_sensor_status = 2;
      } else if (val >= 487.00 && val <= 537.00) {
        oil_sensor_status = 3;
      } else if (val >= 615.00 && val <= 655.00) {
        oil_sensor_status = 4;
      }
    }
    else if (lastPulse == DIAGNOSTIC) {
      oil_temp = ((4096.0 / pulseDuration) * inputDuration - 128) / 19.2 - 40;
      lastPulse = TEMPERATURE;
    }
    else if (lastPulse == TEMPERATURE) {
      oil_pressure = (((4096.0 / pulseDuration) * inputDuration) - 128) / 384.0 + 0.5;
      lastPulse = PRESSURE;
    }
    else if (lastPulse == PRESSURE) { 
      sequenceComplete = true;
    }
    
  } else {
    // Falling edge = End of input
    inputDuration = currentTime - startTime;
  }
}

void readAtmosPressureSensor() {
  atmos_temp = bme.readTemperature();
  atmos_pressure = bme.readPressure();
}

// Read th themistor and run the equations.
void readIntercoolerTemperatureSensor() {
  double sensor_value = 200;  //analogRead(INTERCOOLER_TEMP_PIN);
  double resistor = 10000;
  //convert value to resistance
  double resistance = (4095 / sensor_value) - 1;
  resistance = resistor / resistance;
  float steinhart;                                 //steinhart equation to estimate temperature value at any resistance from curve of thermistor sensor
  steinhart = log(resistance);                     //lnR
  steinhart = pow(steinhart, 3);                   //(lnR)^3
  steinhart *= STEINHART_C;                        //C*((lnR)^3)
  steinhart += (STEINHART_B * (log(resistance)));  //B*(lnR) + C*((lnR)^3)
  steinhart += STEINHART_A;                        //Complete equation, 1/T=A+BlnR+C(lnR)^3
  steinhart = 1.0 / steinhart;                     //Inverse to isolate for T
  steinhart -= 273.15;                             //Conversion from kelvin to celcius

  intercooler_temp = steinhart;
}

void readOilSensor() {
  lastPulse = UNKNOWN;
  sequenceComplete = false;
  attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), oilSensorPWMInterrupt, CHANGE);
  // Stay here until the sequence is complete.
  while(!sequenceComplete);
  detachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN));
}

/**
 * Boost pressure sensor approximate the following equation:
 * pressure = BOOST_COEFFICIENT x voltage - BOOST_INTERCEPT
 * I used excel to calculate the equation using linear regression on the datasheet provided.
 */
void readBoostPressureSensor() {
  float volts, volts5v, boostPressureSensor;
  // read from analog in on main board.
  int raw = analogRead(BOOST_PRESSURE_PIN);
  volts = boostFilter.filter(raw) / 4095.0F * 3.4;
  // calc what the lower 3.3v signal would be in 5v using voltage divider equation
  volts5v = volts * (5600 + 10000) / 10000.0;
  boostPressureSensor = (BOOST_COEFFICIENT * volts5v + BOOST_INTERCEPT);
  boost_pressure = boostPressureSensor;// boostPressure + (targetPressure - boostPressure) * easingFactor;
}

void readEGTSensor() {
  egt = egtFilter.filter(thermocouple.readCelsius());
  if (isnan(egt)) {
    // Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    // if (e & MAX31855_FAULT_OPEN) // Serial.println("FAULT: Thermocouple is open - no connections.");
    // if (e & MAX31855_FAULT_SHORT_GND) // Serial.println("FAULT: Thermocouple is short-circuited to GND.");
    // if (e & MAX31855_FAULT_SHORT_VCC) // Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
  }
}

int count;

void setup() {
  // Serial.begin(115200);
  //while (!Serial) delay(5); // time to get // Serial running

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

    init_ui();
  }



  // Serial.println("Initializing Atmos Pressure Sensor...");
  if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
    // Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  }

  // Set up EGT thermocouple
  // Serial.println("Initializing EGT sensor...");
  if (!thermocouple.begin()) {
    // Serial.println("ERROR.");
    while (1) delay(10);
  }

  // Wait for sensors
  // Serial.println("Initializing Boost sensor...");
  pinMode(BOOST_PRESSURE_PIN, INPUT);

  // Serial.println("Initializing oil sensor...");
  // Set up oil temp/pressure sensor
  pinMode(OIL_PRESSURE_PIN, INPUT);

  count = 0;

  // Serial.println("Setup done");
}

void loop() {

  readAtmosPressureSensor();
  readIntercoolerTemperatureSensor();
  readOilSensor();
  readBoostPressureSensor();
  readEGTSensor();

  if (count % 500 == 0) {
    // Serial.print("Atmos:");
    // Serial.print(atmos_temp);
    // Serial.print(" °C ");
    // Serial.print(atmos_pressure / 100000.0F);
    // Serial.println(" Bar");

    // Serial.print("intercooler temp:");
    // Serial.print(intercooler_temp);
    // Serial.print(" °C; ");

    // Serial.print("Oil temp:");
    // Serial.print(oil_temp);
    // Serial.print("°C; ");

    // Serial.print("Oil pressure:");
    // Serial.print(oil_pressure);
    // Serial.print(" Bar; ");

    // Serial.print("EGT:");
    // Serial.print(egt);
    // Serial.print(" °C; ");
    
    // Serial.print("Boost:");
    // Serial.print(boost_pressure);
    // Serial.print(" psi (");
    // Serial.print(boost_pressure / PSI_BAR_CONVERSION);
    // Serial.println(" Bar);");
  }
  char buffer[6];
  set_needle_value((int)(boost_pressure / PSI_BAR_CONVERSION * 1000));
  // dtostrf(boost_pressure / PSI_BAR_CONVERSION, 2, 1, buffer);
  // lv_label_set_text(boost_pressure_label, buffer);
  
  dtostrf(oil_temp,2, 0, buffer);
  lv_label_set_text(oil_temp_label, buffer);

  dtostrf(oil_pressure,2, 1, buffer);
  lv_label_set_text(oil_pressure_label, buffer);
  
  dtostrf(egt,2, 0, buffer);
  lv_label_set_text(egt_label,  buffer);
  


  lv_task_handler(); /* let the GUI do its work */

  #ifdef DIRECT_MODE
    #ifdef RGB_PANEL
      gfx->flush();
    #else
      gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)draw_buf, screenWidth, screenHeight);
    #endif
  #endif  // #ifdef DIRECT_MODE
  count++;
  delay(5);
}