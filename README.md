# MultiGauge

MultiGauge is an ESP32-based, LVGL-driven round gauge display for automotive
sensors (boost, oil pressure/temp, EGT, and intake temperature). It reads
sensors on dedicated tasks, smooths readings with EWMA filters, and renders a
480x480 round UI with a needle and numeric labels.

## Purpose
- Provide a single, glanceable display for key engine metrics on a round TFT.
- Support fast boost updates alongside slower sensor reads.
- Calibrate oil pressure ambient (tare) at startup for a gauge-style reading.

## Physical Dependencies (Hardware)
- ESP32-class MCU (target wiring matches the Adafruit Qualia ESP32-S3 RGB board).
- 480x480 round RGB panel (TL028WVC01) with a GT911 touch controller.
- Combined oil pressure/temperature PWM sensor (Hella 6PR 010 378-207).
- Boost pressure sensor with 5V analog output and a 5.6k/10k voltage divider.
- EGT thermocouple with a MAX31855 interface.
- Optional: BME280 for atmospheric pressure/temperature.
- Optional: intake/intercooler thermistor (10k divider; coefficients in
  `sensor.h`).

## Wiring Notes (Defaults)
- Oil sensor PWM input: `OIL_PRESSURE_PIN` (mapped to `MOSI`).
- Boost analog input: `BOOST_PRESSURE_PIN` (`A0`).
- Intercooler thermistor: `INTERCOOLER_TEMP_PIN` (`A1`).
- MAX31855: `SCK`, `SS`, `MISO` (hardware SPI).
- GT911 + XCA9554: `SDA`/`SCL` (I2C).

Update pins for your board in `sensor.h` and any board variant headers.

## Software Dependencies
- Arduino framework for ESP32
- LVGL
- Arduino_GFX_Library
- TAMC_GT911
- Adafruit_MAX31855
- Adafruit_BME280 (only if atmos sensor enabled)
- Ewma

## Configuration
- Enable/disable sensors in `constants.h`.
- Boost sensor calibration in `sensor.h` (`BOOST_COEFFICIENT`,
  `BOOST_INTERCEPT`); the spreadsheet `boost_sensor_equation_calc.xlsx`
  documents the regression.
- Oil sensor ambient (tare) is calibrated at startup in `setup()`; fallback is
  0.5 bar if calibration fails.
- UI assets live in `gauge_bg.c` and `needle.c`.

## Build/Upload
Use Arduino IDE or PlatformIO with the ESP32 board package installed, then
install the libraries above and upload `MultiGauge.ino`.
