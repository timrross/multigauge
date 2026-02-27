#ifndef SENSOR_MCU_SENSOR_H
#define SENSOR_MCU_SENSOR_H

#include <Arduino.h>

// ============================================================
// Waveshare ESP32-C3-Zero Pin Assignments
// ============================================================
//
//   GPIO 2/4/5:  EGT thermocouple (MAX31855, software SPI)
//   GPIO 6/7:    CAN bus (TWAI peripheral)
//   GPIO 8/9:    I2C SDA/SCL → ADS1115 ADC board
//
// All analog sensors connect to the ADS1115 via its four
// single-ended inputs. The ESP32-C3 ADC is not used for sensors.
// ============================================================

// TWAI (CAN) pins
#define CAN_TX_PIN  7
#define CAN_RX_PIN  6

// SPI pins for MAX31855 EGT sensor (software SPI)
#define EGT_SCK_PIN   5
#define EGT_MISO_PIN  4
#define EGT_CS_PIN    2

// I2C pins for ADS1115
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

// ADS1115 I2C address (ADDR pin tied to GND → 0x48)
#define ADS1115_I2C_ADDR  0x48

// ADS1115 channel assignments
#define ADS_CHAN_OIL_PRESSURE  0  // Oil pressure transducer (10k/15k voltage divider)
#define ADS_CHAN_OIL_TEMP      1  // Oil temp NTC thermistor (10k pullup)
#define ADS_CHAN_BOOST         2  // Boost pressure sensor (5.6k/10k voltage divider)
#define ADS_CHAN_INTAKE_TEMP   3  // Intake air temp NTC thermistor (10k pullup)

// ============================================================
// Oil Pressure Transducer
// ============================================================
// Sensor: 0-150 psi, 0.5-4.5V output, 5V supply
// Voltage divider: R_upper=10k, R_lower=15k
//   V_adc = V_sensor * 15/(10+15) = V_sensor * 0.6
//   V_sensor = V_adc / 0.6
//   psi = (V_sensor - 0.5) / (4.5 - 0.5) * 150
// Output is converted to bar before CAN transmission.
#define OIL_PRESS_DIVIDER_RATIO  0.6f   // R_lower / (R_upper + R_lower)
#define OIL_PRESS_SENSOR_VMIN    0.5f   // Sensor output voltage at 0 bar
#define OIL_PRESS_SENSOR_VMAX    4.5f   // Sensor output voltage at full scale
#define OIL_PRESS_MAX_PSI        150.0f // Sensor full-scale in psi

// ============================================================
// Boost Pressure Sensor Voltage Divider
// ============================================================
// R1=5600, R2=10000 → V_sensor = V_adc * (R1+R2)/R2 = V_adc * 1.56
#define BOOST_R1  5600.0
#define BOOST_R2  10000.0

// Boost sensor calibration (linear regression, units: psi)
#define BOOST_COEFFICIENT  12.864
#define BOOST_INTERCEPT   -12.877

// ============================================================
// Thermistor Configuration
// ============================================================
// Circuit: Vcc -- R_pullup -- [ADS1115 input] -- Thermistor -- GND
//   R_thermistor = R_pullup * V_adc / (Vcc - V_adc)
#define THERMISTOR_PULLUP  10000.0f  // 10k ohm pullup resistor
#define THERMISTOR_VCC     3.3f      // Pullup supply voltage (ADS1115 VDD)

// Steinhart-Hart coefficients for OIL TEMP thermistor
// Derived from manufacturer R/T table (20-170°C, ±5%)
// Verified at 20°C=2031Ω, 90°C=146.7Ω, 170°C=19.8Ω
#define OIL_TEMP_STEINHART_A  0.00151740
#define OIL_TEMP_STEINHART_B  0.00024720
#define OIL_TEMP_STEINHART_C  0.00000002634

// Steinhart-Hart coefficients for INTAKE AIR TEMP thermistor
// TODO: Replace with values from calibration run
#define INTAKE_TEMP_STEINHART_A  0.00135721593521515
#define INTAKE_TEMP_STEINHART_B  0.00024467275139054
#define INTAKE_TEMP_STEINHART_C  0.00000028439705482

// ============================================================
// Data Structures
// ============================================================

typedef struct OilStatusData {
  double oilTemperature;  // degrees C (ADS1115 AIN1, NTC thermistor)
  double oilPressure;     // bar (ADS1115 AIN0, pressure transducer)
} OilStatusData;

// ============================================================
// Function Declarations
// ============================================================

void initSensors();

OilStatusData readOilSensor();
double readBoostPressureSensor();
double readEgtSensor();
float readAtmosPressureSensor();
float readAtmosTemperatureSensor();
double readIntercoolerTemperatureSensor();

#endif // SENSOR_MCU_SENSOR_H
