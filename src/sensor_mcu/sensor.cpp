#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31855.h>
#include <Ewma.h>
#include "sensor_types.h"
#include "sensor.h"

// ============================================================
// Sensor Objects
// ============================================================

Adafruit_ADS1115 ads;

// EGT sensor using software SPI on ESP32-C3 pins
Adafruit_MAX31855 thermocouple(EGT_SCK_PIN, EGT_CS_PIN, EGT_MISO_PIN);

// ============================================================
// EWMA Filters
// ============================================================

static const double easingFactor = 0.05;

Ewma boostFilter(easingFactor);
Ewma oilTempFilter(easingFactor);
Ewma oilPressureFilter(easingFactor);
Ewma egtFilter(easingFactor);
Ewma intakeTempFilter(easingFactor);

// ============================================================
// ADS1115 Mutex
// Boost task (100ms) and sensor task (500ms) both access the
// ADS1115 over I2C; a mutex prevents concurrent access.
// ============================================================

static SemaphoreHandle_t adsMutex = NULL;

// ============================================================
// Sensor Initialization
// ============================================================

void initSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!ads.begin(ADS1115_I2C_ADDR, &Wire)) {
    Serial.println("[SENSOR] ADS1115 not found! Check wiring and I2C address.");
    while (1) delay(10);
  }
  ads.setGain(GAIN_ONE);  // ±4.096V full-scale, 0.125 mV/bit
  Serial.println("[SENSOR] ADS1115 OK");

  adsMutex = xSemaphoreCreateMutex();
  if (adsMutex == NULL) {
    Serial.println("[SENSOR] Failed to create ADS1115 mutex — out of heap!");
    while (1) delay(10);
  }

  #if ENABLE_EGT_SENSOR
    if (!thermocouple.begin()) {
      Serial.println("[SENSOR] MAX31855 not found!");
      while (1) delay(10);
    }
    Serial.println("[SENSOR] MAX31855 EGT OK");
  #endif
}

// ============================================================
// ADS1115 Helper
// ============================================================

// Read a single-ended channel voltage. Returns NAN if the mutex
// cannot be acquired or the read times out.
static float readADSVolts(uint8_t channel) {
  if (adsMutex == NULL) return NAN;
  if (xSemaphoreTake(adsMutex, pdMS_TO_TICKS(50)) != pdTRUE) return NAN;
  int16_t raw = ads.readADC_SingleEnded(channel);
  float volts = ads.computeVolts(raw);
  xSemaphoreGive(adsMutex);
  return volts;
}

// ============================================================
// Thermistor Helpers
// ============================================================

// Circuit: Vcc -- R_pullup -- [ADS input] -- Thermistor -- GND
//   R_thermistor = R_pullup * V_adc / (Vcc - V_adc)
static float thermistorResistance(float volts) {
  float denom = THERMISTOR_VCC - volts;
  if (denom < 0.01f || volts < 0.001f) return -1.0f;
  return THERMISTOR_PULLUP * volts / denom;
}

// Steinhart-Hart equation: 1/T(K) = A + B*ln(R) + C*ln(R)^3
static float steinhartTemp(float resistance, double A, double B, double C) {
  if (resistance <= 0.0f) return NAN;
  double logR = log((double)resistance);
  double tempK = 1.0 / (A + B * logR + C * logR * logR * logR);
  return (float)(tempK - 273.15);
}

// ============================================================
// Sensor Reading Functions
// ============================================================

// Atmospheric sensor stubs — BMP280 is on the display MCU via I2C
float readAtmosPressureSensor()    { return NAN; }
float readAtmosTemperatureSensor() { return NAN; }

OilStatusData readOilSensor() {
  OilStatusData data;
  data.oilTemperature = NAN;
  data.oilPressure    = NAN;

  // --- Oil pressure transducer (ADS1115 AIN0) ---
  // Sensor: 0-150 psi, 0.5-4.5V output (at 5V supply)
  // After 10k/15k voltage divider: V_adc = V_sensor * 0.6
  float vPressure = readADSVolts(ADS_CHAN_OIL_PRESSURE);
  if (!isnan(vPressure)) {
    float vSensor = vPressure / OIL_PRESS_DIVIDER_RATIO;
    if (vSensor >= OIL_PRESS_SENSOR_VMIN && vSensor <= OIL_PRESS_SENSOR_VMAX + 0.1f) {
      float psi = (vSensor - OIL_PRESS_SENSOR_VMIN) /
                  (OIL_PRESS_SENSOR_VMAX - OIL_PRESS_SENSOR_VMIN) * OIL_PRESS_MAX_PSI;
      psi = constrain(psi, 0.0f, OIL_PRESS_MAX_PSI);
      double bar = (double)psi / PSI_BAR_CONVERSION;
      data.oilPressure = oilPressureFilter.filter(bar);
    }
    // vSensor < 0.5V indicates open circuit / sensor fault — leave as NAN
  }

  // --- Oil temp NTC thermistor (ADS1115 AIN1) ---
  // 10k pullup to 3.3V; Steinhart-Hart from manufacturer R/T table
  float vTemp = readADSVolts(ADS_CHAN_OIL_TEMP);
  if (!isnan(vTemp)) {
    float Rt = thermistorResistance(vTemp);
    float tempC = steinhartTemp(Rt, OIL_TEMP_STEINHART_A,
                                    OIL_TEMP_STEINHART_B,
                                    OIL_TEMP_STEINHART_C);
    if (!isnan(tempC) && tempC > -40.0f && tempC < 250.0f) {
      data.oilTemperature = oilTempFilter.filter((double)tempC);
    }
  }

  return data;
}

double readBoostPressureSensor() {
  // Boost sensor: existing 5.6k/10k voltage divider
  // V_sensor = V_adc * (R1 + R2) / R2
  float vAdc = readADSVolts(ADS_CHAN_BOOST);
  if (isnan(vAdc)) return NAN;
  double vSensor = (double)vAdc * (BOOST_R1 + BOOST_R2) / BOOST_R2;
  double psi = BOOST_COEFFICIENT * vSensor + BOOST_INTERCEPT;
  return boostFilter.filter(psi);
}

double readIntercoolerTemperatureSensor() {
  // Intake air temp NTC thermistor, 10k pullup to 3.3V
  float vAdc = readADSVolts(ADS_CHAN_INTAKE_TEMP);
  if (isnan(vAdc)) return NAN;
  float Rt = thermistorResistance(vAdc);
  float tempC = steinhartTemp(Rt, INTAKE_TEMP_STEINHART_A,
                                   INTAKE_TEMP_STEINHART_B,
                                   INTAKE_TEMP_STEINHART_C);
  if (isnan(tempC) || tempC <= -40.0f || tempC >= 250.0f) return NAN;
  return intakeTempFilter.filter((double)tempC);
}

double readEgtSensor() {
  double egt = thermocouple.readCelsius();
  if (isnan(egt)) {
    thermocouple.readError();  // Clear error state
    return NAN;
  }
  // The MAX31855 can return non-NAN garbage on certain fault conditions
  // (e.g. bad cold-junction reference). Guard before it enters the EWMA
  // filter — a single bad value takes ~60 readings to wash out at alpha=0.05.
  if (egt < 0.0 || egt > 1200.0) return NAN;
  return egtFilter.filter(egt);
}
