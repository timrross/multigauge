#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

#define ADC_MAX_VALUE 4095  // Max ADC value for ESP32 12-bit resolution
#define VREF 3.3

// Pins for reading sensors
#define OIL_PRESSURE_PIN MOSI
#define BOOST_PRESSURE_PIN A0
#define INTERCOOLER_TEMP_PIN A1

#define STEINHART_A 0.00135721593521515000000
#define STEINHART_B 0.00024467275139054400000
#define STEINHART_C 0.00000028439705482433300

#define SEALEVELPRESSURE_HPA (1013.25)

/* These values give a boost sensor reading in psi */
#define BOOST_COEFFICIENT 12.864
#define BOOST_INTERCEPT -12.877

extern int count;

extern double easingFactor;

typedef struct {
  double oilTemperature;
  double oilPressure;
  byte oilSensorStatus;
} OilStatusData;

void initSensors();
OilStatusData readOilSensor();
double readBoostPressureSensor();
double readEgtSensor();
float readAtmosPressureSensor();
float readAtmosTemperatureSensor();
double readIntercoolerTemperatureSensor();

// Oil pressure ambient (tare) helpers
void setOilAmbientBar(double bar);
double getOilAmbientBar();
bool calibrateOilZero(uint16_t samples = 64, uint32_t timeoutMs = 2000);

#endif