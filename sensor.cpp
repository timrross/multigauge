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
volatile double oilSensorStatus = 0;

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

enum PulseType {
    UNKNOWN,
    DIAGNOSTIC,
    TEMPERATURE,
    PRESSURE
};
volatile bool sequenceComplete = false;
volatile PulseType lastPulse = UNKNOWN;
volatile long pulseDuration = 0;
volatile long inputDuration = 0;
volatile long startTime = 0;

// PWM reading from oil temp/pressure sensor
void IRAM_ATTR oilSensorPWMInterrupt() {
#if defined(ESP32)
  bool pinState = (GPIO.in & (1ULL << OIL_PRESSURE_PIN)) != 0;
#else
  bool pinState = digitalRead(OIL_PRESSURE_PIN) == HIGH;
#endif
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
        oilSensorStatus = 1;
      } else if (val >= 359.00 && val <= 409.00) {
        oilSensorStatus = 2;
      } else if (val >= 487.00 && val <= 537.00) {
        oilSensorStatus = 3;
      } else if (val >= 615.00 && val <= 655.00) {
        oilSensorStatus = 4;
      }
    }
    else if (lastPulse == DIAGNOSTIC) {
      oilTemperature = ((4096.0 / pulseDuration) * inputDuration - 128) / 19.2 - 40;
      lastPulse = TEMPERATURE;
    }
    else if (lastPulse == TEMPERATURE) {
      oilPressure = (((4096.0 / pulseDuration) * inputDuration) - 128) / 384.0 + 0.5;
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
  lastPulse = UNKNOWN;
  sequenceComplete = false;
  oilSensorStatus = 0;
  pulseDuration = 0;
  inputDuration = 0;
  startTime = micros();
  attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), oilSensorPWMInterrupt, CHANGE);
  // Stay here until the sequence is complete.
  const uint32_t timeout_us = 100000;
  uint32_t start_wait = micros();
  while(!sequenceComplete) {
    if ((uint32_t)(micros() - start_wait) > timeout_us) {
      break;
    }
    delay(1);
  }
  detachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN));
  // Set the status struct, and return it. 
  if (!sequenceComplete) {
    status.oilPressure = NAN;
    status.oilTemperature = NAN;
    status.oilSensorStatus = 0;
  } else {
    status.oilPressure = oilPressure;
    status.oilTemperature = oilTemperature;
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
