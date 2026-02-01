#ifndef SENSOR_MCU_SENSOR_H
#define SENSOR_MCU_SENSOR_H

#include <Arduino.h>

// ============================================================
// Waveshare ESP32-C3-Zero Pin Assignments (GPIO 0-6 available)
// ============================================================

// TWAI (CAN) pins - directly connected to SN65HVD230
#define CAN_TX_PIN  3
#define CAN_RX_PIN  2

// SPI pins for MAX31855 EGT sensor
#define EGT_SCK_PIN   5
#define EGT_MISO_PIN  4
#define EGT_CS_PIN    6

// Oil sensor PWM input (interrupt-capable)
#define OIL_PRESSURE_PIN  1

// Analog inputs
#define BOOST_PRESSURE_PIN    0   // ADC1_CH0
#define INTERCOOLER_TEMP_PIN  0   // Shared with boost (not enough pins for both)

// ============================================================
// Sensor Configuration
// ============================================================

#define ADC_MAX_VALUE 4095  // ESP32 12-bit ADC
#define VREF 3.3

// Steinhart-Hart coefficients for thermistor
#define STEINHART_A 0.00135721593521515000000
#define STEINHART_B 0.00024467275139054400000
#define STEINHART_C 0.00000028439705482433300

// Boost sensor calibration (from linear regression)
#define BOOST_COEFFICIENT 12.864
#define BOOST_INTERCEPT -12.877

// ============================================================
// Data Structures
// ============================================================

typedef struct {
  double oilTemperature;
  double oilPressure;
  uint8_t oilSensorStatus;
} OilStatusData;

// ============================================================
// Function Declarations
// ============================================================

void initSensors();

// Sensor reading functions
OilStatusData readOilSensor();
double readBoostPressureSensor();
double readEgtSensor();
float readAtmosPressureSensor();
float readAtmosTemperatureSensor();
double readIntercoolerTemperatureSensor();

// Oil pressure calibration
void setOilAmbientBar(double bar);
double getOilAmbientBar();
bool calibrateOilZero(uint16_t samples = 64, uint32_t timeoutMs = 2000);

#endif // SENSOR_MCU_SENSOR_H
