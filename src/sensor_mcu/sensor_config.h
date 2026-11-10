#ifndef SENSOR_MCU_SENSOR_CONFIG_H
#define SENSOR_MCU_SENSOR_CONFIG_H

// ============================================================
// Sensor node carrier board, revision 2
// See docs/SENSOR_NODE_HANDOFF.md sections 4 and 5.
// Every board-specific number lives in this file. No Arduino includes here,
// so the conversion maths can also be compiled and tested on a PC.
// ============================================================

// ---- Waveshare ESP32-C3-Zero pins ----
// GP2, GP8, GP9 (boot strapping), GP10 (RGB LED), GP18/19 (USB), GP20/21 are left free.
#define CAN_TX_PIN    0   // -> CAN module "CAN TX". If the bus stays silent, swap these two.
#define CAN_RX_PIN    1   // -> CAN module "CAN RX"
#define EGT_MISO_PIN  3   // MAX31855 DO
#define I2C_SDA_PIN   4   // ADS1115 SDA
#define I2C_SCL_PIN   5   // ADS1115 SCL
#define EGT_SCK_PIN   6   // MAX31855 CLK
#define EGT_CS_PIN    7   // MAX31855 CS

// ---- ADS1115 (ADDR tied to GND) ----
#define ADS1115_I2C_ADDR       0x48
#define ADS_CHAN_OIL_PRESSURE  0   // A0: 10k / 15k divider
#define ADS_CHAN_BOOST         1   // A1: 10k / 15k divider
#define ADS_CHAN_OIL_TEMP      2   // A2: 330R pull-up, 10k series
#define ADS_CHAN_INTAKE_TEMP   3   // A3: 2k2 pull-up, 10k series

// ---- Supply rails: measure the real board with a multimeter and update ----
#define V_3V3_RAIL  3.30f   // NTC pull-up supply (C3-Zero 3V3 pin)
#define V_5V_RAIL   5.00f   // Pressure sensor supply, measured at J1 pin 1
// 1 = the pressure sensors' output scales with their supply (most 0.5-4.5V sensors do),
// so readings are corrected for V_5V_RAIL. 0 = the output is absolute.
#define PRESSURE_SENSORS_RATIOMETRIC 1

// ---- Oil pressure: 0-150 psi, 0.5-4.5V ----
#define OIL_PRESS_DIVIDER_RATIO  0.6f    // R2 / (R1 + R2) = 15k / 25k. Trim against a multimeter if needed.
#define OIL_PRESS_SENSOR_VMIN    0.5f    // Sensor output at 0 psi
#define OIL_PRESS_SENSOR_VMAX    4.5f    // Sensor output at full scale
#define OIL_PRESS_MAX_PSI        150.0f
#define OIL_PRESS_FAULT_LOW_V    0.25f   // Below this (at the sensor): open wire or dead sensor
#define OIL_PRESS_FAULT_HIGH_V   4.75f   // Above this: shorted to supply

// ---- Boost pressure: round 3-wire sensor, part number unknown ----
#define BOOST_DIVIDER_RATIO   0.6f      // R4 / (R3 + R4) = 15k / 25k
// Existing calibration (linear regression on the sensor's datasheet, in psi, on sensor volts).
#define BOOST_COEFFICIENT     12.864
#define BOOST_INTERCEPT      -12.877
// Atmospheric pressure reads about 1.0V, so anything near 0V means the sensor is unplugged.
// These limits are a best guess until the sensor is confirmed on the bench.
#define BOOST_FAULT_LOW_V     0.20f
#define BOOST_FAULT_HIGH_V    4.90f

// ---- NTC thermistors: 3V3 -- pull-up -- [node] -- NTC -- GND, node -- 10k -- ADC ----
#define OIL_TEMP_PULLUP     330.0f
#define INTAKE_TEMP_PULLUP  2200.0f
// Open circuit (sender unplugged, wire broken): the ADC pin sits at the 3V3 rail.
// The limit is a fraction of V_3V3_RAIL. A very cold sender also reads close to the rail,
// so with the 330R pull-up the oil limit has to sit high: 0.985 is about -24C. That only
// works if V_3V3_RAIL is set from a real measurement (within about 1%).
#define OIL_TEMP_OPEN_FRACTION     0.985f
#define INTAKE_TEMP_OPEN_FRACTION  0.970f
#define NTC_SHORT_V         0.10f   // Below this: wire shorted to ground
#define NTC_MIN_C          -40.0f   // Plausibility limits on the converted temperature
#define NTC_MAX_C           250.0f

// Oil temp sender, 2031R at 20C, 108R at 100C.
// Least squares fit to the manufacturer's 16-point table (20-170C), within 0.5C.
#define OIL_TEMP_STEINHART_A  1.511193e-3
#define OIL_TEMP_STEINHART_B  2.489549e-4
#define OIL_TEMP_STEINHART_C  1.298416e-8

// Intake temp sender, about 2.1k at 25C. Tim's own three-point fit.
#define INTAKE_TEMP_STEINHART_A  1.357216e-3
#define INTAKE_TEMP_STEINHART_B  2.446728e-4
#define INTAKE_TEMP_STEINHART_C  2.843971e-7

// ---- EGT ----
#define EGT_MIN_C   -50.0
#define EGT_MAX_C   1300.0

// ---- Smoothing (exponential filter). Higher alpha = faster response, less smoothing. ----
// Time to show 90% of a step = dt * ln(0.1) / ln(1 - alpha), where dt is the sample period.
#define BOOST_FILTER_ALPHA      0.40   // 10 Hz -> 90% in about 0.45 s
#define OIL_PRESS_FILTER_ALPHA  0.50   //  2 Hz -> 90% in about 1.7 s
#define OIL_TEMP_FILTER_ALPHA   0.25   //  2 Hz -> 90% in about 4 s
#define EGT_FILTER_ALPHA        0.40   //  2 Hz -> 90% in about 2.3 s
#define INTAKE_FILTER_ALPHA     0.30   //  1 Hz -> 90% in about 6.5 s

#endif // SENSOR_MCU_SENSOR_CONFIG_H
