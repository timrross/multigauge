#ifndef CONSTANTS_H
#define CONSTANTS_H

#define DEBUG true

#define ENABLE_BOOST_SENSOR true
#define ENABLE_INTERCOOLER_SENSOR false
#define ENABLE_EGT_SENSOR true
#define ENABLE_OIL_SENSOR true
#define ENABLE_ATMOS_SENSOR false

#define PSI_BAR_CONVERSION 14.5038

struct SensorData {
  double atmosPressure;
  double atmosTemperature;
  double oilTemperature;
  double oilPressure;
  double boostPressure;
  double intercoolerTemperature;
  double egt;

  // Constructor to initialize all values to NAN
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

#endif