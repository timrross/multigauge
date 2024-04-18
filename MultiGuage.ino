
#include <Adafruit_MAX31855.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>

// Pins for reading sensors
#define OIL_PRESSURE_PIN A0

#define STEINHART_A 0.00135721593521515000000
#define STEINHART_B 0.00024467275139054400000
#define STEINHART_C 0.00000028439705482433300

#define SEALEVELPRESSURE_HPA (1013.25)

#define BOOST_COEFFICIENT 1.3538
#define BOOST_INTERCEPT 0.9388

/*Set to your screen resolution*/
#define TFT_HOR_RES 480
#define TFT_VER_RES 480

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))


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
Adafruit_ADS1115 ads;                           // ADC extender module for sensor input

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static uint32_t bufSize;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
#ifndef DIRECT_MODE
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
#endif  // #ifndef DIRECT_MODE

  lv_disp_flush_ready(disp);
}

// static lv_obj_t *boost_guage;
// static lv_meter_indicator_t *needle;
static lv_obj_t *oil_temp_label;
static lv_obj_t *oil_pressure_label;
static lv_obj_t *egt_label;

void init_ui() {

  // boost_guage = lv_meter_create(lv_scr_act());
  // lv_obj_center(boost_guage);
  // lv_obj_set_size(boost_guage, 470, 470);

  // static lv_style_t style;
  // lv_style_init(&style);
  // lv_style_set_bg_color(&style, lv_color_black());
  // lv_style_set_border_width(&style, 0);
  // lv_style_set_text_color(&style, lv_color_white());
  // lv_style_set_text_letter_space(&style, 5);
  // lv_style_set_text_line_space(&style, 20);
  // lv_style_set_text_font(&style, &lv_font_montserrat_24);
  // lv_style_set_text_align(&style, LV_TEXT_ALIGN_CENTER);
  // lv_obj_add_style(boost_guage, &style, LV_PART_MAIN);
  // lv_obj_add_style(boost_guage, &style, LV_PART_TICKS);

  /*Add a scale first*/
  // lv_meter_scale_t *scale = lv_meter_add_scale(boost_guage);
  // lv_meter_set_scale_ticks(boost_guage, scale, 41, 2, 15, lv_color_white());
  // lv_meter_set_scale_major_ticks(boost_guage, scale, 10, 4, 30, lv_color_white(), 30);
  // lv_meter_set_scale_range(boost_guage, scale, 0, 200, 270, 135);

  // lv_meter_indicator_t *indic;

  // /*Add a red arc to the end*/
  // indic = lv_meter_add_arc(boost_guage, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
  // lv_meter_set_indicator_start_value(boost_guage, indic, 150);
  // lv_meter_set_indicator_end_value(boost_guage, indic, 200);

  /*Make the tick lines red at the end of the scale*/
  // indic = lv_meter_add_scale_lines(boost_guage, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
  // lv_meter_set_indicator_start_value(boost_guage, indic, 150);
  // lv_meter_set_indicator_end_value(boost_guage, indic, 200);

  oil_temp_label = lv_label_create(lv_scr_act());
  lv_obj_set_width(oil_temp_label, LV_SIZE_CONTENT);
  lv_label_set_text(oil_temp_label, "--");
  lv_obj_align(oil_temp_label, LV_ALIGN_CENTER, 0, 40);
  // lv_obj_add_style(oil_temp_label, &style, LV_PART_MAIN);

  oil_pressure_label = lv_label_create(lv_scr_act());
  lv_obj_set_width(oil_pressure_label, LV_SIZE_CONTENT);
  lv_label_set_text(oil_pressure_label, "--");
  lv_obj_align(oil_pressure_label, LV_ALIGN_CENTER, 0, 80);
  // lv_obj_add_style(oil_pressure_label, &style, LV_PART_MAIN);

  egt_label = lv_label_create(lv_scr_act());
  lv_obj_set_width(egt_label, LV_SIZE_CONTENT);
  lv_label_set_text(egt_label, "--");
  lv_obj_align(egt_label, LV_ALIGN_CENTER, 0, 120);
  // lv_obj_add_style(egt_label, &style, LV_PART_MAIN);

  /*Add a needle line indicator*/
  // needle = lv_meter_add_needle_line(boost_guage, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);
}

/* Vars for oil pressure */
// Keep track of the current and last timer.
volatile unsigned long timer[2];
// keep trackk of the time spent on a channel.
volatile unsigned long ch[3];
// keep track of the old level
volatile byte level;
volatile int input[3];
volatile int pulse = 0;
volatile double oil_temp;
volatile double oil_pressure;
volatile byte oil_sensor_status;

/* Vars for boost pressure */
double easingFactor = 0.1;
// Sensor readings
double boostPressure = 0.0;

/* vars for EGT */
double egt = 0.0;

double intercooler_temp = 0.0;

/* vars for atmos sensor */
float atmos_temp = 0.0;
float atmos_pressure = 0.0;


// PWM reading from oil temp/pressure sensor
void IRAM_ATTR change_isr() {
  bool level = (GPIO.in & (1ULL << OIL_PRESSURE_PIN)) != 0;
  timer[0] = micros();
  if (level) {
    ch[pulse] = timer[0] - timer[1];
    timer[1] = timer[0];
    if (ch[pulse] < 3000) {
      pulse = 0;
    } else {
      pulse++;
    }
  } else {
    input[pulse] = timer[0] - timer[1];
    if (pulse == 0) {
      oil_temp = ((4096.0 / ch[0]) * input[0] - 128) / 19.2 - 40;
    } else if (pulse == 1) {
      oil_pressure = (((4096.0 / ch[1]) * input[1]) - 128) / 384.0 + 0.5;
    } else if (pulse == 2) {
      double val = (1024.0 / ch[2]) * input[2];
      if (val >= 231.00 && val <= 281.00) {
        oil_sensor_status = 1;
      } else if (val >= 359.00 && val <= 409.00) {
        oil_sensor_status = 2;
      } else if (val >= 487.00 && val <= 537.00) {
        oil_sensor_status = 4;
      } else if (val >= 615.00 && val <= 655.00) {
        oil_sensor_status = 4;
      }
    }
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
  // Take a snapshot of the oil sensor reading and use that for
}

/**
 * Boost pressure sensor approximate the following equation:
 * pressure = BOOST_COEFFICIENT x voltage - BOOST_INTERCEPT
 * I used excel to calculate the equation using linear regression on the datasheet provided.
 */
void readBoostPressureSensor() {
  int16_t adc0;
  float volts;
  // read from analog in on main board.
  //int boostPressureSensor = analogRead(BOOST_PRESSURE_PIN);
  // Read from ADC module
  adc0 = ads.readADC_SingleEnded(2);
  volts = ads.computeVolts(adc0);
  // Do the voltage calc
  float boostPressureSensor = BOOST_COEFFICIENT * volts - BOOST_INTERCEPT;
  // interestingly, the pressure ADC works out as a range between -1000 and 3100 hpa (-1 - 3.1 bar)
  // So I can just use the ADC value directly and it should be pretty close.
  float targetPressure = boostPressureSensor;  //map(boostPressureSensor, 0, 4095, 0, 410);

  boostPressure = boostPressureSensor;  // boostPressure + (targetPressure - boostPressure) * easingFactor;
}

void readEGTSensor() {
  egt = thermocouple.readCelsius();
  if (isnan(egt)) {
    Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
  }
}


void setup() {
  Serial.begin(115200);

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

  #ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  #endif

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  bufSize = screenWidth * 40;


  #ifdef ESP32
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf) {
      // remove MALLOC_CAP_INTERNAL flag try again
      disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_8BIT);
    }
  #else
    disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * bufSize);
  #endif

  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);

    init_ui();
  }

  while (!Serial)
    ;  // time to get serial running

  Serial.println("Initializing Atmos Pressure Sensor...");
  if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  }

  Serial.println("Initializing ADC Module...");
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1) delay(10);
  }

  // Set up EGT thermocouple
  Serial.println("Initializing EGT sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("Initializing Boost sensor...");
  //pinMode(BOOST_PRESSURE_PIN, INPUT);

  // Wait for sensors
  Serial.println("Initializing oil sensor...");
  // Set up oil temp/pressure sensor
  pinMode(OIL_PRESSURE_PIN, INPUT);
  // Wait for the sensor to wake up
  // Add rising and falling interrupts
  attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), change_isr, CHANGE);

  Serial.println("Setup done");
}

int count = 0;

void loop() {



  readAtmosPressureSensor();
  readIntercoolerTemperatureSensor();
  readOilSensor();
  readBoostPressureSensor();
  readEGTSensor();

  if (count % 500) {

    Serial.print("Atmos:");
    Serial.print(atmos_temp);
    Serial.println(" °C ");
    Serial.print(atmos_pressure / 100.0F);
    Serial.println(" hPa");

    Serial.print("intercooler temp:");
    Serial.print(intercooler_temp);
    Serial.print("; ");
    Serial.print("Oil temp:");
    Serial.print(oil_temp);
    Serial.print("; ");
    Serial.print("Oil pressure:");
    Serial.print(oil_pressure);
    Serial.print("; ");
    Serial.print("EGT:");
    Serial.print(egt);
    Serial.print("; ");
    Serial.print("Boost:");
    Serial.print(boostPressure);
    Serial.println("; ");
  }
  lv_label_set_text_fmt(oil_temp_label, "%.1f", oil_temp);
  lv_label_set_text_fmt(oil_pressure_label, "%.1f", oil_pressure);
  lv_label_set_text_fmt(egt_label, "%.1f", egt);
  // lv_meter_set_indicator_value(boost_guage, needle, boostPressure);
  lv_timer_handler(); /* let the GUI do its work */
  count++;
  delay(10);
}