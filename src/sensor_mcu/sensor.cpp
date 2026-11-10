#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31855.h>
#include "sensor.h"

// ============================================================
// Sensor Objects
// ============================================================

static Adafruit_ADS1115 ads;

// EGT sensor using software SPI
static Adafruit_MAX31855 thermocouple(EGT_SCK_PIN, EGT_CS_PIN, EGT_MISO_PIN);

static Smoother boostFilter(BOOST_FILTER_ALPHA);
static Smoother oilPressureFilter(OIL_PRESS_FILTER_ALPHA);
static Smoother oilTempFilter(OIL_TEMP_FILTER_ALPHA);
static Smoother egtFilter(EGT_FILTER_ALPHA);
static Smoother intakeTempFilter(INTAKE_FILTER_ALPHA);

// ============================================================
// ADS1115 access
// The boost task (100ms) and the sensor task (500ms) share the ADC, so every
// conversion is done under a mutex.
// ============================================================

static SemaphoreHandle_t adsMutex = NULL;
static volatile bool adsPresent = false;
static uint8_t adsFailCount = 0;

#define ADS_CONVERSION_TIMEOUT_MS  30   // one conversion takes about 8ms at 128 samples/s
#define ADS_MAX_CONSECUTIVE_FAILS  5

static const uint16_t ADS_MUX[4] = {
  ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
  ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3
};

// Caller must hold adsMutex.
static bool probeAds() {
  if (!ads.begin(ADS1115_I2C_ADDR, &Wire)) return false;
  ads.setGain(GAIN_ONE);  // +/-4.096V full scale, 0.125 mV per bit
  adsFailCount = 0;
  return true;
}

// Read one of the ADS1115's 16-bit registers. False if the I2C transaction failed.
// (The library's own register reads cannot report a failure.)
static bool readAdsRegister(uint8_t reg, uint16_t* out) {
  Wire.beginTransmission((uint8_t)ADS1115_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)ADS1115_I2C_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  *out = (uint16_t)((hi << 8) | lo);
  return true;
}

#define ADS_REG_CONVERSION  0x00
#define ADS_REG_CONFIG      0x01
#define ADS_CONFIG_DONE     0x8000   // conversion finished
#define ADS_CONFIG_MUX_MASK 0x7000   // which input the conversion is for

// Start one conversion and wait for it. Caller must hold adsMutex.
// Fails (returns false) on any I2C error, if the ADC is not converting the channel
// that was asked for (the start command was lost), or after ADS_CONVERSION_TIMEOUT_MS.
// (The library's own readADC_SingleEnded() waits forever if the module stops answering.)
static bool convertChannel(uint8_t channel, int16_t* counts) {
  ads.startADCReading(ADS_MUX[channel], /*continuous=*/false);
  uint32_t start = millis();
  while (millis() - start < ADS_CONVERSION_TIMEOUT_MS) {
    vTaskDelay(pdMS_TO_TICKS(2));
    uint16_t config;
    if (!readAdsRegister(ADS_REG_CONFIG, &config)) return false;
    if ((config & ADS_CONFIG_MUX_MASK) != ADS_MUX[channel]) return false;
    if (config & ADS_CONFIG_DONE) {
      uint16_t raw;
      if (!readAdsRegister(ADS_REG_CONVERSION, &raw)) return false;
      *counts = (int16_t)raw;
      return true;
    }
  }
  return false;
}

// Read one single-ended channel in volts. Returns NAN if the ADC is missing,
// the mutex cannot be taken, or the conversion fails.
static float readADSVolts(uint8_t channel) {
  if (adsMutex == NULL || !adsPresent || channel > 3) return NAN;
  if (xSemaphoreTake(adsMutex, pdMS_TO_TICKS(100)) != pdTRUE) return NAN;

  float volts = NAN;
  if (adsPresent) {
    int16_t counts;
    if (convertChannel(channel, &counts)) {
      volts = ads.computeVolts(counts);
      adsFailCount = 0;
    } else if (++adsFailCount >= ADS_MAX_CONSECUTIVE_FAILS) {
      adsPresent = false;
      Serial.println("[SENSOR] ADS1115 stopped answering");
    }
  }

  xSemaphoreGive(adsMutex);
  return volts;
}

// ============================================================
// Init / maintenance
// ============================================================

void initSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(50);

  adsMutex = xSemaphoreCreateMutex();
  if (adsMutex == NULL) {
    Serial.println("[SENSOR] Failed to create ADS1115 mutex");
    return;  // adcOk() stays false, every analogue reading reports FAULT_ADC
  }

  adsPresent = probeAds();
  Serial.println(adsPresent ? "[SENSOR] ADS1115 OK"
                            : "[SENSOR] ADS1115 not found, will keep trying");

  #if ENABLE_EGT_SENSOR
    // With software SPI begin() cannot detect a missing chip. readEgt() does that.
    thermocouple.begin();
  #endif
}

void maintainSensors() {
  if (adsPresent || adsMutex == NULL) return;
  if (xSemaphoreTake(adsMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  if (probeAds()) {
    adsPresent = true;
    Serial.println("[SENSOR] ADS1115 found");
  }
  xSemaphoreGive(adsMutex);
}

bool adcOk() {
  return adsPresent;
}

// ============================================================
// Sensor Reading Functions
// ============================================================

Reading readOilPressure() {
  return oilPressureFilter.apply(convertOilPressure(readADSVolts(ADS_CHAN_OIL_PRESSURE)));
}

Reading readOilTemperature() {
  return oilTempFilter.apply(convertOilTemp(readADSVolts(ADS_CHAN_OIL_TEMP)));
}

Reading readBoostPressure() {
  return boostFilter.apply(convertBoost(readADSVolts(ADS_CHAN_BOOST)));
}

Reading readIntakeTemperature() {
  return intakeTempFilter.apply(convertIntakeTemp(readADSVolts(ADS_CHAN_INTAKE_TEMP)));
}

Reading readEgt() {
  // Check the fault bits first: readCelsius() only says "NAN" without saying why.
  uint8_t chipFault = thermocouple.readError();
  double tempC = NAN, internalC = NAN;
  if ((chipFault & 0x07) == 0) {
    tempC = thermocouple.readCelsius();
    internalC = thermocouple.readInternal();
  }
  return egtFilter.apply(convertEgt(chipFault, tempC, internalC));
}
