#ifndef SENSOR_MCU_CONVERSIONS_H
#define SENSOR_MCU_CONVERSIONS_H

// Pure conversion maths: ADC volts in, engineering value + fault code out.
// No hardware access and no Arduino includes, so this can be tested on a PC.

#include <math.h>
#include <stdint.h>
#include "can_protocol.h"     // FAULT_* codes
#include "sensor_types.h"     // PSI_BAR_CONVERSION
#include "sensor_config.h"

struct Reading {
  double value;    // NAN whenever fault != FAULT_NONE
  uint8_t fault;   // FAULT_* code (EGT_FAULT_* bit mask for the EGT reading)
};

static inline Reading faultReading(uint8_t fault) {
  Reading r = { NAN, fault };
  return r;
}

// Scale a pressure sensor's output back to what it would be on exactly 5.00V.
static inline float normalisePressureVolts(float vSensor) {
#if PRESSURE_SENSORS_RATIOMETRIC
  return vSensor * 5.0f / V_5V_RAIL;
#else
  return vSensor;
#endif
}

// Oil pressure in bar from the voltage at ADS1115 A0.
static inline Reading convertOilPressure(float vAdc) {
  if (isnan(vAdc)) return faultReading(FAULT_ADC);
  float vSensor = vAdc / OIL_PRESS_DIVIDER_RATIO;
  if (vSensor < OIL_PRESS_FAULT_LOW_V) return faultReading(FAULT_SIGNAL_LOW);
  if (vSensor > OIL_PRESS_FAULT_HIGH_V) return faultReading(FAULT_SIGNAL_HIGH);
  float v = normalisePressureVolts(vSensor);
  float psi = (v - OIL_PRESS_SENSOR_VMIN) / (OIL_PRESS_SENSOR_VMAX - OIL_PRESS_SENSOR_VMIN) * OIL_PRESS_MAX_PSI;
  // A healthy sensor at 0 psi sits right on 0.5V and noise takes it either side: clamp, do not fault.
  if (psi < 0.0f) psi = 0.0f;
  if (psi > OIL_PRESS_MAX_PSI) psi = OIL_PRESS_MAX_PSI;
  Reading r = { (double)psi / PSI_BAR_CONVERSION, FAULT_NONE };
  return r;
}

// Boost in psi (gauge) from the voltage at ADS1115 A1.
static inline Reading convertBoost(float vAdc) {
  if (isnan(vAdc)) return faultReading(FAULT_ADC);
  float vSensor = vAdc / BOOST_DIVIDER_RATIO;
  if (vSensor < BOOST_FAULT_LOW_V) return faultReading(FAULT_SIGNAL_LOW);
  if (vSensor > BOOST_FAULT_HIGH_V) return faultReading(FAULT_SIGNAL_HIGH);
  double v = (double)normalisePressureVolts(vSensor);
  Reading r = { BOOST_COEFFICIENT * v + BOOST_INTERCEPT, FAULT_NONE };
  return r;
}

// NTC temperature in C from the voltage at the ADC pin.
// R = R_pullup * V / (V_3v3 - V), then Steinhart-Hart: 1/T = A + B ln(R) + C ln(R)^3
static inline Reading convertNtc(float vAdc, float rPullup, float openFraction, double A, double B, double C) {
  if (isnan(vAdc)) return faultReading(FAULT_ADC);
  if (vAdc > openFraction * V_3V3_RAIL) return faultReading(FAULT_SIGNAL_HIGH);
  if (vAdc < NTC_SHORT_V) return faultReading(FAULT_SIGNAL_LOW);
  double r = (double)rPullup * vAdc / ((double)V_3V3_RAIL - vAdc);
  double lnR = log(r);
  double tempC = 1.0 / (A + B * lnR + C * lnR * lnR * lnR) - 273.15;
  if (isnan(tempC) || tempC < NTC_MIN_C || tempC > NTC_MAX_C) return faultReading(FAULT_RANGE);
  Reading out = { tempC, FAULT_NONE };
  return out;
}

static inline Reading convertOilTemp(float vAdc) {
  return convertNtc(vAdc, OIL_TEMP_PULLUP, OIL_TEMP_OPEN_FRACTION,
                    OIL_TEMP_STEINHART_A, OIL_TEMP_STEINHART_B, OIL_TEMP_STEINHART_C);
}

static inline Reading convertIntakeTemp(float vAdc) {
  return convertNtc(vAdc, INTAKE_TEMP_PULLUP, INTAKE_TEMP_OPEN_FRACTION,
                    INTAKE_TEMP_STEINHART_A, INTAKE_TEMP_STEINHART_B, INTAKE_TEMP_STEINHART_C);
}

// EGT from one MAX31855 read: fault bits, thermocouple C and cold-junction C.
static inline Reading convertEgt(uint8_t chipFaultBits, double thermocoupleC, double internalC) {
  if (chipFaultBits & 0x07) return faultReading(chipFaultBits & 0x07);
  if (isnan(thermocoupleC)) return faultReading(EGT_FAULT_OPEN);
  // An absent or unpowered MAX31855 clocks out all zeros, which decodes as a "valid" 0.00 C
  // on both the thermocouple and the cold junction.
  if (thermocoupleC == 0.0 && internalC == 0.0) return faultReading(EGT_FAULT_NO_CHIP);
  if (thermocoupleC < EGT_MIN_C || thermocoupleC > EGT_MAX_C) return faultReading(EGT_FAULT_RANGE);
  Reading r = { thermocoupleC, 0 };
  return r;
}

// Exponential smoothing. The first sample after start-up or after a fault is taken as-is,
// so a returning sensor does not ramp up from a stale value.
struct Smoother {
  double alpha;
  bool primed;
  double state;

  explicit Smoother(double a) : alpha(a), primed(false), state(0.0) {}

  Reading apply(Reading in) {
    if (in.fault != 0 || isnan(in.value)) {
      primed = false;
      return in;
    }
    if (!primed) {
      state = in.value;
      primed = true;
    } else {
      state += alpha * (in.value - state);
    }
    in.value = state;
    return in;
  }
};

#endif // SENSOR_MCU_CONVERSIONS_H
