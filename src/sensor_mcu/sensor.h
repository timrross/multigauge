#ifndef SENSOR_MCU_SENSOR_H
#define SENSOR_MCU_SENSOR_H

#include <Arduino.h>
#include "sensor_config.h"   // pins, channels, calibration constants
#include "conversions.h"     // Reading, FAULT_* codes

// Set up I2C, the ADS1115 and the MAX31855. A missing ADS1115 is NOT fatal:
// the node still starts, sends heartbeats and reports the fault, and
// maintainSensors() keeps looking for the module.
void initSensors();

// Call about once a second. Re-probes the ADS1115 if it has stopped answering.
void maintainSensors();

// True while the ADS1115 is answering.
bool adcOk();

// Each returns a filtered value, or NAN with a fault code.
Reading readOilPressure();        // bar
Reading readOilTemperature();     // C
Reading readBoostPressure();      // psi (gauge)
Reading readIntakeTemperature();  // C
Reading readEgt();                // C, fault is an EGT_FAULT_* bit mask

#endif // SENSOR_MCU_SENSOR_H
