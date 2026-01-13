#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_BME280.h>
#include <Ewma.h>
#if defined(ESP32)
#include "soc/gpio_struct.h"
#endif
#include "constants.h"
#include "sensor.h"

// Sensors
Adafruit_MAX31855 thermocouple(SCK, SS, MISO);  // EGT Sensor module
Adafruit_BME280 bme;                            // atmosphere pressure/temp module

int count = 0;

double easingFactor = 0.05;

/* vars for oil sensor */
volatile double oilTemperature = 0.0;
volatile double oilPressure = 0.0;
volatile uint8_t oilSensorStatus = 0;

// Set up Exponential Weighted moving average for all the sensor readings.
Ewma boostFilter(easingFactor); // Make the boost needle move a bit faster. 
Ewma oilTempFilter(easingFactor); 
Ewma oilPressureFilter(easingFactor); 
Ewma egtFilter(easingFactor);
Ewma intercoolerFilter(easingFactor);

void initSensors() {

  // Set up atmos sensor
  #if ENABLE_ATMOS_SENSOR
    // Serial.println("Initializing Atmos Pressure Sensor...");
    if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
      // Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      while (1) delay(10);
    }
  #endif

  // Set up EGT thermocouple
  #if ENABLE_EGT_SENSOR
    // Serial.println("Initializing EGT sensor...");
    if (!thermocouple.begin()) {
      // Serial.println("ERROR.");
      while (1) delay(10);
    }
  #endif

  // Set up Boost sensor
  #if ENABLE_INTERCOOLER_SENSOR
    // Serial.println("Initializing Intercooler sensor...");
    pinMode(INTERCOOLER_TEMP_PIN, INPUT_PULLDOWN);
  #endif

  // Set up Boost sensor
  #if ENABLE_BOOST_SENSOR
    // Serial.println("Initializing Boost sensor...");
    pinMode(BOOST_PRESSURE_PIN, INPUT_PULLDOWN);
  #endif

  // Set up oil temp/pressure sensor
  #if ENABLE_OIL_SENSOR
    // Serial.println("Initializing oil sensor...");
    pinMode(OIL_PRESSURE_PIN, INPUT);
  #endif

}

enum PulseStage : uint8_t {
  STAGE_SYNC,
  STAGE_T1,
  STAGE_T2
};
volatile bool sequenceComplete = false;
volatile PulseStage pulseStage = STAGE_SYNC;
volatile bool captureEnabled = false;
volatile bool haveHighTime = false;
volatile uint32_t lastRiseUs = 0;
volatile uint32_t lastHighUs = 0;

static constexpr uint32_t kS1PeriodNomUs = 1024;
static constexpr uint32_t kT1PeriodNomUs = 4096;
static constexpr uint32_t kT2PeriodNomUs = 4096;
static constexpr uint32_t kS1PeriodMinUs = (kS1PeriodNomUs * 9) / 10;
static constexpr uint32_t kS1PeriodMaxUs = (kS1PeriodNomUs * 11) / 10;
static constexpr uint32_t kT1PeriodMinUs = (kT1PeriodNomUs * 9) / 10;
static constexpr uint32_t kT1PeriodMaxUs = (kT1PeriodNomUs * 11) / 10;
static constexpr uint32_t kT2PeriodMinUs = (kT2PeriodNomUs * 9) / 10;
static constexpr uint32_t kT2PeriodMaxUs = (kT2PeriodNomUs * 11) / 10;

static constexpr uint32_t kDiagTolUs = 25;
static constexpr uint32_t kDiagOkUs = 256;
static constexpr uint32_t kDiagPressureFailUs = 384;
static constexpr uint32_t kDiagTempFailUs = 512;
static constexpr uint32_t kDiagHardwareFailUs = 640;

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
  if (in_range_u32(scaled_s1_us, kDiagOkUs - kDiagTolUs, kDiagOkUs + kDiagTolUs)) {
    return 1;
  }
  if (in_range_u32(scaled_s1_us, kDiagPressureFailUs - kDiagTolUs, kDiagPressureFailUs + kDiagTolUs)) {
    return 2;
  }
  if (in_range_u32(scaled_s1_us, kDiagTempFailUs - kDiagTolUs, kDiagTempFailUs + kDiagTolUs)) {
    return 3;
  }
  if (in_range_u32(scaled_s1_us, kDiagHardwareFailUs - kDiagTolUs, kDiagHardwareFailUs + kDiagTolUs)) {
    return 4;
  }
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

// PWM reading from oil temp/pressure sensor
void IRAM_ATTR oilSensorPWMInterrupt() {
  if (!captureEnabled) return;
#if defined(ESP32)
  bool pinState = (GPIO.in & (1ULL << OIL_PRESSURE_PIN)) != 0;
#else
  bool pinState = digitalRead(OIL_PRESSURE_PIN) == HIGH;
#endif
  uint32_t currentTime = micros();
  if (pinState) {
    // Rising edge: evaluate the previous pulse using its stored high time.
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
      if (!in_range_u32(periodUs, kS1PeriodMinUs, kS1PeriodMaxUs)) {
        return;
      }
      uint32_t scaledS1 = scale_high_us(highUs, periodUs, kS1PeriodNomUs);
      uint8_t diagState = decode_diag_state(scaledS1);
      if (diagState == 0) {
        return;
      }
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
    // Falling edge: record the high time for the current pulse.
    if (lastRiseUs == 0) return;
    lastHighUs = currentTime - lastRiseUs;
    haveHighTime = true;
  }
}

float readAtmosPressureSensor() {
  return bme.readPressure();
}

float readAtmosTemperatureSensor() {
  return bme.readTemperature();
}

// Function to calculate the resistance of the thermistor using a voltage divider
float calculate_thermistor_resistance(int adcValue) {
  if (10 >= adcValue) {
    // Sensor is not valid and/or Avoid division by zero
    return -1.0;  // Error value, adjust as needed
  }
  float Rt = 10000.0 * (ADC_MAX_VALUE / float(adcValue) - 1.0);
  return Rt;
}

// Function to calculate temperature from resistance using Steinhart-Hart equation
float calculate_thermistor_temperature(float resistance) {
  // If the resistance is an error, then just return absolute zero so it's
  // Obvious the reading was not working.
  if (resistance <= 0) {
    return -273.15;
  }
  float logR = log(resistance);
  float tempK = 1.0 / (STEINHART_A + STEINHART_B * logR + STEINHART_C * pow(logR, 3));
  float tempC = tempK - 273.15;  // Convert Kelvin to Celsius
  return tempC;
}

// Read th themistor and run the equations.
double readIntercoolerTemperatureSensor() {
  double sensor_value = analogRead(INTERCOOLER_TEMP_PIN);
  double Rt = calculate_thermistor_resistance(sensor_value);
  return intercoolerFilter.filter(calculate_thermistor_temperature(Rt));
}

OilStatusData readOilSensor() {
  OilStatusData status;
  pulseStage = STAGE_SYNC;
  sequenceComplete = false;
  captureEnabled = true;
  haveHighTime = false;
  lastRiseUs = 0;
  lastHighUs = 0;
  oilSensorStatus = 0;
  oilTemperature = NAN;
  oilPressure = NAN;
  attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), oilSensorPWMInterrupt, CHANGE);
  // Stay here until the sequence is complete.
  const uint32_t timeout_us = 30000;
  uint32_t start_wait = micros();
  while(!sequenceComplete) {
    if ((uint32_t)(micros() - start_wait) > timeout_us) {
      break;
    }
    delayMicroseconds(100);
  }
  captureEnabled = false;
  detachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN));
  // Set the status struct, and return it. 
  if (!sequenceComplete || oilSensorStatus == 0) {
    status.oilPressure = NAN;
    status.oilTemperature = NAN;
    status.oilSensorStatus = 0;
  } else {
    double temp = oilTemperature;
    double pressure = oilPressure;

    if (oilSensorStatus == 2) {
      pressure = NAN;
    } else if (oilSensorStatus == 3) {
      temp = NAN;
    } else if (oilSensorStatus == 4) {
      temp = NAN;
      pressure = NAN;
    }

    if (!isnan(temp)) {
      temp = oilTempFilter.filter(temp);
    }
    if (!isnan(pressure)) {
      pressure = oilPressureFilter.filter(pressure);
    }

    status.oilPressure = pressure;
    status.oilTemperature = temp;
    status.oilSensorStatus = oilSensorStatus;
  }
  return status;
}

/**
 * Boost pressure sensor approximate the following equation:
 * pressure = BOOST_COEFFICIENT x voltage - BOOST_INTERCEPT
 * I used excel to calculate the equation using linear regression on the datasheet provided.
 */
double readBoostPressureSensor() {
  double r1, r2, volts, volts5v, boostPressureSensor;
  r1 = 5600.0;
  r2 = 10000.0;
  // read from analog in on main board.
  int raw = analogRead(BOOST_PRESSURE_PIN);
  volts = raw / 4095.0F * 3.4;
  // calc what the lower 3.3v signal would be in 5v using voltage divider equation
  volts5v = volts * (r1 + r2) / r2;
  boostPressureSensor = (BOOST_COEFFICIENT * volts5v + BOOST_INTERCEPT);
  return boostFilter.filter(boostPressureSensor);
}

double readEgtSensor() {
  double egt = thermocouple.readCelsius();
  if (isnan(egt)) {
    // Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    // if (e & MAX31855_FAULT_OPEN) // Serial.println("FAULT: Thermocouple is open - no connections.");
    // if (e & MAX31855_FAULT_SHORT_GND) // Serial.println("FAULT: Thermocouple is short-circuited to GND.");
    // if (e & MAX31855_FAULT_SHORT_VCC) // Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
  }
  return egtFilter.filter(egt);
}

void readSensors() {

  #if ENABLE_ATMOS_SENSOR
    readAtmosPressureSensor();
    readAtmosTemperatureSensor();
  #endif

  #if ENABLE_INTERCOOLER_SENSOR
    readIntercoolerTemperatureSensor();
  #endif

  #if ENABLE_OIL_SENSOR
    readOilSensor();
  #endif

  #if ENABLE_BOOST_SENSOR
    readBoostPressureSensor();
  #endif

  #if ENABLE_EGT_SENSOR
    readEgtSensor();
  #endif

  #if DEBUG
  // if (count % 100 == 0) {

  //   #if ENABLE_ATMOS_SENSOR
  //     Serial.print("Atmos:");
  //     Serial.print(atmosTemp);
  //     Serial.print(" °C ");
  //     Serial.print(atmosPressure / 100000.0F);
  //     Serial.println(" Bar");
  //   #endif

  //   #if ENABLE_INTERCOOLER_SENSOR
  //     Serial.print("intercooler temp:");
  //     Serial.print(intercoolerTemp);
  //     Serial.print(" °C; ");
  //   #endif

  //   #if ENABLE_OIL_SENSOR
  //     Serial.print("Oil temp:");
  //     Serial.print(oilTemp);
  //     Serial.print("°C; ");

  //     Serial.print("Oil pressure:");
  //     Serial.print(oilPressure);
  //     Serial.print(" Bar; ");
  //   #endif

  //   #if ENABLE_EGT_SENSOR 
  //     Serial.print("EGT:");
  //     Serial.print(egt);
  //     Serial.print(" °C; ");
  //   #endif

  //   #if ENABLE_BOOST_SENSOR
  //     Serial.print("Boost:");
  //     Serial.print(boostPressure);
  //     Serial.print(" psi (");
  //     Serial.print(boostPressure / PSI_BAR_CONVERSION);
  //     Serial.print(" Bar);");
  //   #endif

  //   Serial.println();
  // }
  #endif
}
