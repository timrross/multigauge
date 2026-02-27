#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <math.h>

// Sensor enable flags (both MCUs need to agree)
#define ENABLE_BOOST_SENSOR true
#define ENABLE_INTERCOOLER_SENSOR true
#define ENABLE_EGT_SENSOR true
#define ENABLE_OIL_SENSOR true

// BMP280 atmospheric sensor - connected to display MCU (Qualia) via I2C
// Can be used for altitude compensation of boost readings if needed
#define ENABLE_ATMOS_SENSOR false

// Conversion constants
#define PSI_BAR_CONVERSION 14.5038

// Oil sensor status codes
#define OIL_STATUS_TIMEOUT  0  // No valid reading (sensor fault, open circuit, etc.)
#define OIL_STATUS_OK       1  // Both pressure and temperature readings valid

// Sensor data structure (for local use on display MCU)
struct SensorData {
  double atmosPressure;
  double atmosTemperature;
  double oilTemperature;
  double oilPressure;
  double boostPressure;
  double intercoolerTemperature;
  double egt;

  SensorData() :
    atmosPressure(NAN),
    atmosTemperature(NAN),
    oilTemperature(NAN),
    oilPressure(NAN),
    boostPressure(NAN),
    intercoolerTemperature(NAN),
    egt(NAN)
  {}
};

#endif // SENSOR_TYPES_H
