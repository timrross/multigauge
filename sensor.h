#ifndef SENSOR_H
#define SENSOR_H

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

#define PSI_BAR_CONVERSION 14.5038

void initSensors();
void readSensors();

extern double easingFactor;

/* vars for oil sensor */
extern volatile double oilTemp;
extern volatile double oilPressure;
extern volatile byte oilSensorStatus;

/* Vars for boost pressure */
extern double boostPressure;

/* vars for EGT */
extern double egt;

/* vars for intercooler Temp */
extern double intercoolerTemp;

/* vars for atmos sensor */
extern double atmosTemp;
extern double atmosPressure;

#endif