#ifndef SENSOR_MCU_SENSOR_H
#define SENSOR_MCU_SENSOR_H

#include <Arduino.h>

// ============================================================
// Waveshare ESP32-C3-Zero Pin Assignments
// ============================================================
//
// GPIO 0-5 (left side) match the Qualia perfboard pin order so the
// sensor board plugs straight in without re-wiring:
//
//   Qualia pin:  GND  3.3V  A0   A1   CS   MOSI  MISO  SCK
//   C3 GPIO:     GND  3.3V  0    1    2    3     4     5
//   Function:                Boost ICT  EGT  Oil   EGT   EGT
//                            ADC   ADC  CS   PWM   MISO  SCK
//
// CAN transceiver is on the other side of the board (GPIO 6-7).
// ============================================================

// TWAI (CAN) pins - directly connected to SN65HVD230 (right side)
#define CAN_TX_PIN  7
#define CAN_RX_PIN  6

// Sensor pins (left side - matches Qualia perfboard layout)
// SPI pins for MAX31855 EGT sensor
#define EGT_SCK_PIN   5
#define EGT_MISO_PIN  4
#define EGT_CS_PIN    2

// Oil sensor PWM input (interrupt-capable)
#define OIL_PRESSURE_PIN  3

// Analog inputs
#define BOOST_PRESSURE_PIN    0   // ADC1_CH0
#define INTERCOOLER_TEMP_PIN  1   // ADC1_CH1

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
