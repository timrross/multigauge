#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <Ewma.h>
#include "sensor_types.h"
#include "sensor.h"

// ============================================================
// Sensor Objects
// ============================================================

// EGT sensor using software SPI on ESP32-C3 pins
Adafruit_MAX31855 thermocouple(EGT_SCK_PIN, EGT_CS_PIN, EGT_MISO_PIN);

// ============================================================
// EWMA Filters
// ============================================================

static double easingFactor = 0.05;

Ewma boostFilter(easingFactor);
Ewma oilTempFilter(easingFactor);
Ewma oilPressureFilter(easingFactor);
Ewma egtFilter(easingFactor);
Ewma intercoolerFilter(easingFactor);

// ============================================================
// Oil Sensor State
// ============================================================

volatile double oilTemperature = 0.0;
volatile double oilPressure = 0.0;
volatile uint8_t oilSensorStatus = 0;

// Oil pressure ambient baseline
static double oilAmbientBar = NAN;

static inline double toGaugeBar(double rawBar) {
  const double amb = isnan(oilAmbientBar) ? 0.5 : oilAmbientBar;
  double g = rawBar - amb;
  if (g < 0.0) g = 0.0;
  return g;
}

void setOilAmbientBar(double bar) { oilAmbientBar = bar; }
double getOilAmbientBar() { return oilAmbientBar; }

bool calibrateOilZero(uint16_t samples, uint32_t timeoutMs) {
  double sum = 0.0;
  uint16_t ok = 0;
  uint32_t t0 = millis();
  while (ok < samples && (millis() - t0) < timeoutMs) {
    OilStatusData s = readOilSensor();
    if (s.oilSensorStatus == 1 && !isnan(s.oilPressure) && s.oilPressure < 2.0) {
      sum += s.oilPressure;
      ok++;
    }
    delay(5);
  }
  if (ok) {
    oilAmbientBar = sum / ok;
    return true;
  }
  return false;
}

// ============================================================
// Oil Sensor PWM Decoding (ISR-based)
// ============================================================

// Forward declaration
void oilSensorPWMInterrupt();

// State machine stages
enum PulseStage : uint8_t {
  STAGE_SYNC,
  STAGE_T1,
  STAGE_T2
};

// ISR state variables
volatile bool sequenceComplete = false;
volatile PulseStage pulseStage = STAGE_SYNC;
volatile bool captureEnabled = false;
volatile bool haveHighTime = false;
volatile uint32_t lastRiseUs = 0;
volatile uint32_t lastHighUs = 0;

// Timing constants
static constexpr uint32_t kS1PeriodNomUs = 1024;
static constexpr uint32_t kT1PeriodNomUs = 4096;
static constexpr uint32_t kT2PeriodNomUs = 4096;

static constexpr uint32_t kS1PeriodMinUs = (kS1PeriodNomUs * 9) / 10;
static constexpr uint32_t kS1PeriodMaxUs = (kS1PeriodNomUs * 11) / 10;
static constexpr uint32_t kT1PeriodMinUs = (kT1PeriodNomUs * 9) / 10;
static constexpr uint32_t kT1PeriodMaxUs = (kT1PeriodNomUs * 11) / 10;
static constexpr uint32_t kT2PeriodMinUs = (kT2PeriodNomUs * 9) / 10;
static constexpr uint32_t kT2PeriodMaxUs = (kT2PeriodNomUs * 11) / 10;

// Diagnostic thresholds
static constexpr uint32_t kDiagTolUs = 25;
static constexpr uint32_t kDiagOkUs = 256;
static constexpr uint32_t kDiagPressureFailUs = 384;
static constexpr uint32_t kDiagTempFailUs = 512;
static constexpr uint32_t kDiagHardwareFailUs = 640;

// Data value bounds
static constexpr uint32_t kValueMinUs = 128;
static constexpr uint32_t kValueMaxUs = 3968;
static constexpr uint32_t kValueMarginUs = 16;

static inline bool in_range_u32(uint32_t value, uint32_t min_value, uint32_t max_value) {
  return value >= min_value && value <= max_value;
}

static inline uint32_t scale_high_us(uint32_t high_us, uint32_t period_us, uint32_t nominal_period_us) {
  if (period_us == 0) return 0;
  return (uint32_t)(((uint64_t)high_us * nominal_period_us + (period_us / 2)) / period_us);
}

static uint8_t decode_diag_state(uint32_t scaled_s1_us) {
  if (in_range_u32(scaled_s1_us, kDiagOkUs - kDiagTolUs, kDiagOkUs + kDiagTolUs)) return 1;
  if (in_range_u32(scaled_s1_us, kDiagPressureFailUs - kDiagTolUs, kDiagPressureFailUs + kDiagTolUs)) return 2;
  if (in_range_u32(scaled_s1_us, kDiagTempFailUs - kDiagTolUs, kDiagTempFailUs + kDiagTolUs)) return 3;
  if (in_range_u32(scaled_s1_us, kDiagHardwareFailUs - kDiagTolUs, kDiagHardwareFailUs + kDiagTolUs)) return 4;
  return 0;
}

static inline double scaled_to_temperature(uint32_t scaled_us) {
  double temp = ((double)scaled_us - 128.0) / 19.2 - 40.0;
  if (temp < -40.0) return -40.0;
  if (temp > 160.0) return 160.0;
  return temp;
}

static inline double scaled_to_pressure(uint32_t scaled_us) {
  double pressure = ((double)scaled_us - 128.0) / 384.0 + 0.5;
  if (pressure < 0.5) return 0.5;
  if (pressure > 10.5) return 10.5;
  return pressure;
}

void IRAM_ATTR oilSensorPWMInterrupt() {
  if (!captureEnabled) return;

  bool pinState = digitalRead(OIL_PRESSURE_PIN) == HIGH;
  uint32_t currentTime = micros();

  if (pinState) {
    // Rising edge
    if (lastRiseUs == 0) {
      lastRiseUs = currentTime;
      return;
    }

    uint32_t periodUs = currentTime - lastRiseUs;
    lastRiseUs = currentTime;

    if (!haveHighTime) {
      pulseStage = STAGE_SYNC;
      return;
    }

    uint32_t highUs = lastHighUs;
    haveHighTime = false;

    if (periodUs < kS1PeriodMinUs || periodUs > kT2PeriodMaxUs) {
      pulseStage = STAGE_SYNC;
      return;
    }

    if (highUs == 0 || highUs > periodUs) {
      pulseStage = STAGE_SYNC;
      return;
    }

    if (pulseStage == STAGE_SYNC) {
      if (!in_range_u32(periodUs, kS1PeriodMinUs, kS1PeriodMaxUs)) return;
      uint32_t scaledS1 = scale_high_us(highUs, periodUs, kS1PeriodNomUs);
      uint8_t diagState = decode_diag_state(scaledS1);
      if (diagState == 0) return;
      oilSensorStatus = diagState;
      pulseStage = STAGE_T1;
      return;
    }

    if (pulseStage == STAGE_T1) {
      if (!in_range_u32(periodUs, kT1PeriodMinUs, kT1PeriodMaxUs)) {
        pulseStage = STAGE_SYNC;
        return;
      }
      uint32_t scaledT1 = scale_high_us(highUs, periodUs, kT1PeriodNomUs);
      if (!in_range_u32(scaledT1, kValueMinUs - kValueMarginUs, kValueMaxUs + kValueMarginUs)) {
        pulseStage = STAGE_SYNC;
        return;
      }
      oilTemperature = scaled_to_temperature(scaledT1);
      pulseStage = STAGE_T2;
      return;
    }

    if (pulseStage == STAGE_T2) {
      if (!in_range_u32(periodUs, kT2PeriodMinUs, kT2PeriodMaxUs)) {
        pulseStage = STAGE_SYNC;
        return;
      }
      uint32_t scaledT2 = scale_high_us(highUs, periodUs, kT2PeriodNomUs);
      if (!in_range_u32(scaledT2, kValueMinUs - kValueMarginUs, kValueMaxUs + kValueMarginUs)) {
        pulseStage = STAGE_SYNC;
        return;
      }
      oilPressure = scaled_to_pressure(scaledT2);
      sequenceComplete = true;
      captureEnabled = false;
      pulseStage = STAGE_SYNC;
      return;
    }

  } else {
    // Falling edge
    if (lastRiseUs == 0) return;
    lastHighUs = currentTime - lastRiseUs;
    haveHighTime = true;
  }
}

// ============================================================
// Sensor Initialization
// ============================================================

void initSensors() {
  // Note: Atmospheric sensor (BMP280) is on display MCU, not sensor MCU

  #if ENABLE_EGT_SENSOR
    if (!thermocouple.begin()) {
      while (1) delay(10);
    }
  #endif

  #if ENABLE_INTERCOOLER_SENSOR
    pinMode(INTERCOOLER_TEMP_PIN, INPUT_PULLDOWN);
  #endif

  #if ENABLE_BOOST_SENSOR
    pinMode(BOOST_PRESSURE_PIN, INPUT_PULLDOWN);
  #endif

  #if ENABLE_OIL_SENSOR
    pinMode(OIL_PRESSURE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), oilSensorPWMInterrupt, CHANGE);
  #endif
}

// ============================================================
// Sensor Reading Functions
// ============================================================

// Atmospheric sensor is on display MCU - these are stubs
float readAtmosPressureSensor() {
  return NAN;
}

float readAtmosTemperatureSensor() {
  return NAN;
}

float calculate_thermistor_resistance(int adcValue) {
  if (10 >= adcValue) return -1.0;
  float Rt = 10000.0 * (ADC_MAX_VALUE / float(adcValue) - 1.0);
  return Rt;
}

float calculate_thermistor_temperature(float resistance) {
  if (resistance <= 0) return -273.15;
  float logR = log(resistance);
  float tempK = 1.0 / (STEINHART_A + STEINHART_B * logR + STEINHART_C * pow(logR, 3));
  float tempC = tempK - 273.15;
  return tempC;
}

double readIntercoolerTemperatureSensor() {
  double sensor_value = analogRead(INTERCOOLER_TEMP_PIN);
  double Rt = calculate_thermistor_resistance(sensor_value);
  return intercoolerFilter.filter(calculate_thermistor_temperature(Rt));
}

OilStatusData readOilSensor() {
  OilStatusData status;

  // Disable interrupt while resetting ISR state to avoid race conditions
  detachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN));

  // Reset ISR state
  pulseStage = STAGE_SYNC;
  sequenceComplete = false;
  haveHighTime = false;
  lastRiseUs = 0;
  lastHighUs = 0;
  oilSensorStatus = 0;
  oilTemperature = NAN;
  oilPressure = NAN;

  // Re-enable interrupt and start capture
  captureEnabled = true;
  attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), oilSensorPWMInterrupt, CHANGE);

  // Wait for sequence completion
  const uint32_t timeout_us = 30000;
  uint32_t start_wait = micros();
  while (!sequenceComplete) {
    if ((uint32_t)(micros() - start_wait) > timeout_us) break;
    delayMicroseconds(100);
  }

  captureEnabled = false;

  if (!sequenceComplete || oilSensorStatus == 0) {
    status.oilPressure = NAN;
    status.oilTemperature = NAN;
    status.oilSensorStatus = 0;
  } else {
    double temp = oilTemperature;
    double pressure = oilPressure;

    if (oilSensorStatus == 2) pressure = NAN;
    else if (oilSensorStatus == 3) temp = NAN;
    else if (oilSensorStatus == 4) { temp = NAN; pressure = NAN; }

    if (!isnan(temp)) temp = oilTempFilter.filter(temp);
    if (!isnan(pressure)) {
      pressure = oilPressureFilter.filter(pressure);
      pressure = toGaugeBar(pressure);
    }

    status.oilPressure = pressure;
    status.oilTemperature = temp;
    status.oilSensorStatus = oilSensorStatus;
  }
  return status;
}

double readBoostPressureSensor() {
  double r1 = 5600.0;
  double r2 = 10000.0;
  int raw = analogRead(BOOST_PRESSURE_PIN);
  double volts = raw / 4095.0 * 3.3;
  double volts5v = volts * (r1 + r2) / r2;
  double boostPressureSensor = (BOOST_COEFFICIENT * volts5v + BOOST_INTERCEPT);
  return boostFilter.filter(boostPressureSensor);
}

double readEgtSensor() {
  double egt = thermocouple.readCelsius();
  if (isnan(egt)) {
    thermocouple.readError();  // Clear error
    return NAN;  // Don't pass NaN through filter
  }
  return egtFilter.filter(egt);
}
