#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <math.h>

// Sensor enable flags (both MCUs need to agree)
#define ENABLE_BOOST_SENSOR 1
#define ENABLE_INTERCOOLER_SENSOR 1
#define ENABLE_EGT_SENSOR 1
#define ENABLE_OIL_SENSOR 1

// BMP280 atmospheric sensor - would connect to the display MCU (Qualia) via I2C.
// Not implemented yet: nothing reads it on either node.
#define ENABLE_ATMOS_SENSOR 0

// Conversion constants
#define PSI_BAR_CONVERSION 14.5038

// Sensor data structure (for local use on display MCU).
// A value is NAN whenever there is no fresh, valid reading for it.
struct SensorData {
  double atmosPressure;
  double atmosTemperature;
  double oilTemperature;
  double oilPressure;
  double boostPressure;
  double intercoolerTemperature;
  double egt;
  // When the latest boost reading arrived (millis on the display). The needle
  // glide uses it to time its movement between readings.
  uint32_t boostRxMs;

  SensorData() :
    atmosPressure(NAN),
    atmosTemperature(NAN),
    oilTemperature(NAN),
    oilPressure(NAN),
    boostPressure(NAN),
    intercoolerTemperature(NAN),
    egt(NAN),
    boostRxMs(0)
  {}
};

#endif // SENSOR_TYPES_H
