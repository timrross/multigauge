#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_BME280.h>
#include <Ewma.h>
#if defined(ESP32)
#include "soc/gpio_struct.h"
#endif
#include "constants.h"
#include "sensor.h"

// Sensors
Adafruit_MAX31855 thermocouple(SCK, SS, MISO);  // EGT Sensor module
Adafruit_BME280 bme;                            // atmosphere pressure/temp module

int count = 0;

double easingFactor = 0.05;

/* vars for oil sensor */
volatile double oilTemperature = 0.0;
volatile double oilPressure = 0.0;
volatile uint8_t oilSensorStatus = 0;

// Set up Exponential Weighted moving average for all the sensor readings.

Ewma boostFilter(easingFactor); // Make the boost needle move a bit faster. 
Ewma oilTempFilter(easingFactor); 
Ewma oilPressureFilter(easingFactor); 
Ewma egtFilter(easingFactor);
Ewma intercoolerFilter(easingFactor);

// --- Oil pressure ambient baseline (tare) helpers ---
static double oilAmbientBar = NAN;  // retained baseline; set via calibration or manually

static inline double toGaugeBar(double rawBar) {
  // Use 0.5 bar as fallback ambient until calibrated (common floor in spec)
  const double amb = isnan(oilAmbientBar) ? 0.5 : oilAmbientBar;
  double g = rawBar - amb;
  if (g < 0.0) g = 0.0;        // clamp negative to zero for display
  // Optional: clamp upper bound to 10.5-ambient if desired
  return g;
}

// Optional: manual accessors if you want to store/load from NVS
void setOilAmbientBar(double bar) { oilAmbientBar = bar; }
double getOilAmbientBar() { return oilAmbientBar; }

// Optional: one-shot calibration when engine is definitely off
bool calibrateOilZero(uint16_t samples, uint32_t timeoutMs) {
  double sum = 0.0; uint16_t ok = 0;
  uint32_t t0 = millis();
  while (ok < samples && (millis() - t0) < timeoutMs) {
    OilStatusData s = readOilSensor();
    if (s.oilSensorStatus == 1 && !isnan(s.oilPressure) && s.oilPressure < 2.0) {
      sum += s.oilPressure;
      ok++;
    }
    delay(5);
  }
  if (ok) { oilAmbientBar = sum / ok; return true; }
  return false;
}

void initSensors() {

  // Set up atmos sensor
  #if ENABLE_ATMOS_SENSOR
    // Serial.println("Initializing Atmos Pressure Sensor...");
    if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
      // Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      while (1) delay(10);
    }
  #endif

  // Set up EGT thermocouple
  #if ENABLE_EGT_SENSOR
    // Serial.println("Initializing EGT sensor...");
    if (!thermocouple.begin()) {
      // Serial.println("ERROR.");
      while (1) delay(10);
    }
  #endif

  // Set up Boost sensor
  #if ENABLE_INTERCOOLER_SENSOR
    // Serial.println("Initializing Intercooler sensor...");
    pinMode(INTERCOOLER_TEMP_PIN, INPUT_PULLDOWN);
  #endif

  // Set up Boost sensor
  #if ENABLE_BOOST_SENSOR
    // Serial.println("Initializing Boost sensor...");
    pinMode(BOOST_PRESSURE_PIN, INPUT_PULLDOWN);
  #endif

  // Set up oil temp/pressure sensor
  #if ENABLE_OIL_SENSOR
    // Serial.println("Initializing oil sensor...");
    pinMode(OIL_PRESSURE_PIN, INPUT);
  #endif

}

/*
 * =============================================================================
 * HELLA 6PR 010 378-207 OIL SENSOR PWM PROTOCOL
 * =============================================================================
 *
 * This sensor outputs a continuous PWM signal encoding both oil temperature
 * and pressure in a 3-pulse repeating sequence. The protocol uses pulse-width
 * modulation where the HIGH time of each pulse carries the data value.
 *
 * SIGNAL TIMING DIAGRAM:
 * ----------------------
 *
 *         |<-- S1 -->|<------ T1 ------>|<------ T2 ------>|
 *         |  ~1024µs |     ~4096µs      |     ~4096µs      |
 *         |          |                  |                  |
 *     ____      ______        __________        __________
 *    |    |    |      |      |          |      |          |
 *  __|    |____|      |______|          |______|          |_____ ...
 *
 *    ^         ^             ^                 ^
 *    |         |             |                 |
 *    SYNC    SYNC+T1       T1+T2             T2+SYNC (next cycle)
 *
 *
 * PULSE SEQUENCE:
 * ---------------
 *
 *   1. SYNC PULSE (S1): Period ~1024µs
 *      - High time encodes DIAGNOSTIC STATUS
 *      - Used to synchronize the decoder state machine
 *
 *   2. TEMPERATURE PULSE (T1): Period ~4096µs
 *      - High time encodes OIL TEMPERATURE
 *      - Range: -40°C to +160°C
 *
 *   3. PRESSURE PULSE (T2): Period ~4096µs
 *      - High time encodes OIL PRESSURE
 *      - Range: 0.5 to 10.5 bar (absolute)
 *
 *
 * DIAGNOSTIC STATUS ENCODING (SYNC pulse high time):
 * --------------------------------------------------
 *
 *   High Time (µs)  |  Status Code  |  Meaning
 *   ----------------|---------------|---------------------------
 *        256        |       1       |  OK - sensor healthy
 *        384        |       2       |  Pressure sensor failure
 *        512        |       3       |  Temperature sensor failure
 *        640        |       4       |  Hardware failure
 *
 *   Tolerance: ±25µs around nominal values
 *
 *
 * DATA VALUE ENCODING (T1 and T2 pulse high times):
 * -------------------------------------------------
 *
 *   High time range: 128µs (minimum) to 3968µs (maximum)
 *   Values are scaled relative to the nominal period to compensate
 *   for timing variations in the sensor's oscillator.
 *
 *   Scaling formula:
 *     scaled_us = (high_us * nominal_period_us) / actual_period_us
 *
 *   Temperature conversion (T1):
 *     temp_C = (scaled_us - 128) / 19.2 - 40
 *     Output range: -40°C to +160°C
 *
 *   Pressure conversion (T2):
 *     pressure_bar = (scaled_us - 128) / 384 + 0.5
 *     Output range: 0.5 to 10.5 bar (absolute pressure)
 *
 *
 * STATE MACHINE:
 * --------------
 *
 *                    ┌─────────────────────────────────────┐
 *                    │                                     │
 *                    ▼                                     │
 *              ┌───────────┐                               │
 *      ───────►│STAGE_SYNC │◄────────────────┐            │
 *    (start)   └─────┬─────┘                 │            │
 *                    │                       │            │
 *                    │ valid S1 pulse        │ invalid    │
 *                    │ (diag status OK)      │ period     │
 *                    ▼                       │            │
 *              ┌───────────┐                 │            │
 *              │ STAGE_T1  │─────────────────┘            │
 *              └─────┬─────┘                              │
 *                    │                                    │
 *                    │ valid T1 pulse                     │
 *                    │ (temp data)                        │
 *                    ▼                                    │
 *              ┌───────────┐                              │
 *              │ STAGE_T2  │──────────────────────────────┘
 *              └─────┬─────┘      invalid period
 *                    │
 *                    │ valid T2 pulse (pressure data)
 *                    │ → sequenceComplete = true
 *                    │
 *                    └──────► (cycle restarts)
 *
 *
 * ISR IMPLEMENTATION NOTES:
 * -------------------------
 *
 *   - Interrupt triggers on BOTH edges (CHANGE mode)
 *   - Rising edge: Calculate period, process previous pulse's data
 *   - Falling edge: Record high time for current pulse
 *   - IRAM_ATTR ensures ISR code is in fast RAM on ESP32
 *   - Direct GPIO register read avoids digitalRead() overhead
 *   - ±10% tolerance on period validation handles oscillator drift
 *
 * =============================================================================
 */

// State machine stages for PWM protocol decoding
enum PulseStage : uint8_t {
  STAGE_SYNC,   // Waiting for/processing sync pulse (diagnostic status)
  STAGE_T1,     // Processing temperature pulse
  STAGE_T2      // Processing pressure pulse
};

// ISR communication flags and state variables (volatile for ISR access)
volatile bool sequenceComplete = false;    // Set true when full S1→T1→T2 captured
volatile PulseStage pulseStage = STAGE_SYNC;
volatile bool captureEnabled = false;      // Gate to enable/disable ISR processing
volatile bool haveHighTime = false;        // Flag: high time recorded for current pulse
volatile uint32_t lastRiseUs = 0;          // Timestamp of last rising edge (µs)
volatile uint32_t lastHighUs = 0;          // Recorded high time of current pulse (µs)

// Nominal pulse periods (microseconds) - used for value scaling
static constexpr uint32_t kS1PeriodNomUs = 1024;   // SYNC pulse nominal period
static constexpr uint32_t kT1PeriodNomUs = 4096;   // Temperature pulse nominal period
static constexpr uint32_t kT2PeriodNomUs = 4096;   // Pressure pulse nominal period

// Period validation bounds (±10% tolerance for oscillator drift)
static constexpr uint32_t kS1PeriodMinUs = (kS1PeriodNomUs * 9) / 10;   // 921µs
static constexpr uint32_t kS1PeriodMaxUs = (kS1PeriodNomUs * 11) / 10;  // 1126µs
static constexpr uint32_t kT1PeriodMinUs = (kT1PeriodNomUs * 9) / 10;   // 3686µs
static constexpr uint32_t kT1PeriodMaxUs = (kT1PeriodNomUs * 11) / 10;  // 4505µs
static constexpr uint32_t kT2PeriodMinUs = (kT2PeriodNomUs * 9) / 10;   // 3686µs
static constexpr uint32_t kT2PeriodMaxUs = (kT2PeriodNomUs * 11) / 10;  // 4505µs

// Diagnostic status high-time thresholds (SYNC pulse)
static constexpr uint32_t kDiagTolUs = 25;              // Tolerance band ±25µs
static constexpr uint32_t kDiagOkUs = 256;              // Status 1: Sensor OK
static constexpr uint32_t kDiagPressureFailUs = 384;    // Status 2: Pressure sensor failed
static constexpr uint32_t kDiagTempFailUs = 512;        // Status 3: Temperature sensor failed
static constexpr uint32_t kDiagHardwareFailUs = 640;    // Status 4: Hardware failure

// Data value bounds (T1/T2 pulse high times)
static constexpr uint32_t kValueMinUs = 128;     // Minimum valid data high time
static constexpr uint32_t kValueMaxUs = 3968;    // Maximum valid data high time
static constexpr uint32_t kValueMarginUs = 16;   // Extra margin for validation

// Helper: Check if value falls within [min, max] inclusive
static inline bool in_range_u32(uint32_t value, uint32_t min_value, uint32_t max_value) {
  return value >= min_value && value <= max_value;
}

/*
 * Scale the measured high time to compensate for oscillator drift.
 *
 * The sensor's internal oscillator may run slightly fast or slow, causing
 * the actual pulse period to differ from nominal. By scaling the high time
 * proportionally, we normalize readings as if the period were exactly nominal.
 *
 * Formula: scaled = high_us * (nominal_period / actual_period)
 * The +period/2 provides rounding to nearest integer.
 */
static inline uint32_t scale_high_us(uint32_t high_us, uint32_t period_us, uint32_t nominal_period_us) {
  if (period_us == 0) return 0;
  return (uint32_t)(((uint64_t)high_us * nominal_period_us + (period_us / 2)) / period_us);
}

/*
 * Decode diagnostic status from SYNC pulse high time.
 * Returns: 1=OK, 2=Pressure fail, 3=Temp fail, 4=Hardware fail, 0=Unknown
 */
static uint8_t decode_diag_state(uint32_t scaled_s1_us) {
  if (in_range_u32(scaled_s1_us, kDiagOkUs - kDiagTolUs, kDiagOkUs + kDiagTolUs)) {
    return 1;  // Sensor OK
  }
  if (in_range_u32(scaled_s1_us, kDiagPressureFailUs - kDiagTolUs, kDiagPressureFailUs + kDiagTolUs)) {
    return 2;  // Pressure sensor failed
  }
  if (in_range_u32(scaled_s1_us, kDiagTempFailUs - kDiagTolUs, kDiagTempFailUs + kDiagTolUs)) {
    return 3;  // Temperature sensor failed
  }
  if (in_range_u32(scaled_s1_us, kDiagHardwareFailUs - kDiagTolUs, kDiagHardwareFailUs + kDiagTolUs)) {
    return 4;  // Hardware failure
  }
  return 0;    // Unknown/invalid diagnostic state
}

/*
 * Convert scaled T1 pulse high time to temperature in Celsius.
 *
 * Formula from Hella datasheet:
 *   temp_C = (scaled_us - 128) / 19.2 - 40
 *
 * At 128µs  → -40°C (minimum)
 * At 3968µs → +160°C (maximum)
 */
static inline double scaled_to_temperature(uint32_t scaled_us) {
  double temp = ((double)scaled_us - 128.0) / 19.2 - 40.0;
  if (temp < -40.0) return -40.0;    // Clamp to sensor range
  if (temp > 160.0) return 160.0;
  return temp;
}

/*
 * Convert scaled T2 pulse high time to pressure in bar (absolute).
 *
 * Formula from Hella datasheet:
 *   pressure_bar = (scaled_us - 128) / 384 + 0.5
 *
 * At 128µs  → 0.5 bar (minimum, ~atmospheric)
 * At 3968µs → 10.5 bar (maximum)
 *
 * Note: This is ABSOLUTE pressure. Gauge pressure (relative to atmosphere)
 * is calculated later by subtracting the calibrated ambient baseline.
 */
static inline double scaled_to_pressure(uint32_t scaled_us) {
  double pressure = ((double)scaled_us - 128.0) / 384.0 + 0.5;
  if (pressure < 0.5) return 0.5;    // Clamp to sensor range
  if (pressure > 10.5) return 10.5;
  return pressure;
}

/*
 * =============================================================================
 * OIL SENSOR PWM INTERRUPT SERVICE ROUTINE
 * =============================================================================
 *
 * This ISR fires on BOTH edges (rising and falling) of the PWM signal.
 * It implements a state machine that decodes the S1→T1→T2 pulse sequence.
 *
 * Edge handling:
 *   - FALLING edge: Record the high time (pulse was HIGH, now going LOW)
 *   - RISING edge:  Calculate period, process the completed pulse's data
 *
 * The reason we process on rising edge (not falling) is that we need the
 * complete period measurement, which requires waiting for the next rising edge.
 *
 * IRAM_ATTR ensures this code stays in fast internal RAM on ESP32 for
 * minimal interrupt latency (~1µs vs ~5µs from flash).
 */
void IRAM_ATTR oilSensorPWMInterrupt() {
  // Early exit if capture is disabled (between readings or after completion)
  if (!captureEnabled) return;

  // Read pin state using direct register access for speed (ESP32)
  // digitalRead() is slower due to function call overhead
#if defined(ESP32)
  bool pinState = (GPIO.in & (1ULL << OIL_PRESSURE_PIN)) != 0;
#else
  bool pinState = digitalRead(OIL_PRESSURE_PIN) == HIGH;
#endif

  uint32_t currentTime = micros();

  if (pinState) {
    // =========================================================================
    // RISING EDGE: A new pulse is starting
    // Process the PREVIOUS pulse using its recorded high time and period
    // =========================================================================

    // First rising edge after enable - just record timestamp, can't measure yet
    if (lastRiseUs == 0) {
      lastRiseUs = currentTime;
      return;
    }

    // Calculate period: time between this rising edge and the previous one
    uint32_t periodUs = currentTime - lastRiseUs;
    lastRiseUs = currentTime;

    // Sanity check: we should have recorded a high time on the falling edge
    if (!haveHighTime) {
      pulseStage = STAGE_SYNC;  // Lost sync, restart
      return;
    }

    uint32_t highUs = lastHighUs;
    haveHighTime = false;  // Consume the recorded high time

    // Validate period is within any expected pulse type's range
    if (periodUs < kS1PeriodMinUs || periodUs > kT2PeriodMaxUs) {
      pulseStage = STAGE_SYNC;  // Invalid period, restart
      return;
    }

    // Sanity check: high time should be positive and less than period
    if (highUs == 0 || highUs > periodUs) {
      pulseStage = STAGE_SYNC;
      return;
    }

    // -------------------------------------------------------------------------
    // STATE: STAGE_SYNC - Looking for valid SYNC (S1) pulse
    // -------------------------------------------------------------------------
    if (pulseStage == STAGE_SYNC) {
      // SYNC pulse should have ~1024µs period
      if (!in_range_u32(periodUs, kS1PeriodMinUs, kS1PeriodMaxUs)) {
        return;  // Not a SYNC pulse, keep waiting
      }

      // Scale high time and decode diagnostic status
      uint32_t scaledS1 = scale_high_us(highUs, periodUs, kS1PeriodNomUs);
      uint8_t diagState = decode_diag_state(scaledS1);

      if (diagState == 0) {
        return;  // Unrecognized diagnostic state, keep waiting for valid SYNC
      }

      // Valid SYNC received - save status and advance to T1
      oilSensorStatus = diagState;
      pulseStage = STAGE_T1;
      return;
    }

    // -------------------------------------------------------------------------
    // STATE: STAGE_T1 - Expecting temperature (T1) pulse
    // -------------------------------------------------------------------------
    if (pulseStage == STAGE_T1) {
      // T1 pulse should have ~4096µs period
      if (!in_range_u32(periodUs, kT1PeriodMinUs, kT1PeriodMaxUs)) {
        pulseStage = STAGE_SYNC;  // Wrong period, lost sync
        return;
      }

      // Scale high time and validate data range
      uint32_t scaledT1 = scale_high_us(highUs, periodUs, kT1PeriodNomUs);
      if (!in_range_u32(scaledT1, kValueMinUs - kValueMarginUs, kValueMaxUs + kValueMarginUs)) {
        pulseStage = STAGE_SYNC;  // Invalid data value
        return;
      }

      // Convert to temperature and advance to T2
      oilTemperature = scaled_to_temperature(scaledT1);
      pulseStage = STAGE_T2;
      return;
    }

    // -------------------------------------------------------------------------
    // STATE: STAGE_T2 - Expecting pressure (T2) pulse
    // -------------------------------------------------------------------------
    if (pulseStage == STAGE_T2) {
      // T2 pulse should have ~4096µs period
      if (!in_range_u32(periodUs, kT2PeriodMinUs, kT2PeriodMaxUs)) {
        pulseStage = STAGE_SYNC;  // Wrong period, lost sync
        return;
      }

      // Scale high time and validate data range
      uint32_t scaledT2 = scale_high_us(highUs, periodUs, kT2PeriodNomUs);
      if (!in_range_u32(scaledT2, kValueMinUs - kValueMarginUs, kValueMaxUs + kValueMarginUs)) {
        pulseStage = STAGE_SYNC;  // Invalid data value
        return;
      }

      // Convert to pressure - SEQUENCE COMPLETE!
      oilPressure = scaled_to_pressure(scaledT2);
      sequenceComplete = true;   // Signal to main code that data is ready
      captureEnabled = false;    // Stop processing until next read request
      pulseStage = STAGE_SYNC;   // Reset for next sequence
      return;
    }

  } else {
    // =========================================================================
    // FALLING EDGE: Pulse going from HIGH to LOW
    // Record the high time for processing on next rising edge
    // =========================================================================
    if (lastRiseUs == 0) return;  // No rising edge recorded yet
    lastHighUs = currentTime - lastRiseUs;  // Duration pulse was HIGH
    haveHighTime = true;                     // Flag that we have valid data
  }
}

float readAtmosPressureSensor() {
  return bme.readPressure();
}

float readAtmosTemperatureSensor() {
  return bme.readTemperature();
}

// Function to calculate the resistance of the thermistor using a voltage divider
float calculate_thermistor_resistance(int adcValue) {
  if (10 >= adcValue) {
    // Sensor is not valid and/or Avoid division by zero
    return -1.0;  // Error value, adjust as needed
  }
  float Rt = 10000.0 * (ADC_MAX_VALUE / float(adcValue) - 1.0);
  return Rt;
}

// Function to calculate temperature from resistance using Steinhart-Hart equation
float calculate_thermistor_temperature(float resistance) {
  // If the resistance is an error, then just return absolute zero so it's
  // Obvious the reading was not working.
  if (resistance <= 0) {
    return -273.15;
  }
  float logR = log(resistance);
  float tempK = 1.0 / (STEINHART_A + STEINHART_B * logR + STEINHART_C * pow(logR, 3));
  float tempC = tempK - 273.15;  // Convert Kelvin to Celsius
  return tempC;
}

// Read th themistor and run the equations.
double readIntercoolerTemperatureSensor() {
  double sensor_value = analogRead(INTERCOOLER_TEMP_PIN);
  double Rt = calculate_thermistor_resistance(sensor_value);
  return intercoolerFilter.filter(calculate_thermistor_temperature(Rt));
}

/*
 * Read oil temperature and pressure from Hella sensor.
 *
 * This function:
 *   1. Resets the ISR state machine
 *   2. Enables interrupt capture on the sensor pin
 *   3. Waits (with timeout) for a complete S1→T1→T2 sequence
 *   4. Disables interrupt and processes results
 *   5. Applies EWMA filtering and gauge pressure correction
 *
 * Returns OilStatusData struct with:
 *   - oilTemperature: Filtered temperature in °C (NAN if failed)
 *   - oilPressure: Filtered GAUGE pressure in bar (NAN if failed)
 *   - oilSensorStatus: 0=timeout, 1=OK, 2=pressure fail, 3=temp fail, 4=hw fail
 *
 * Typical timing: ~10-15ms for complete sequence (3 pulses)
 * Timeout: 30ms (allows ~3 complete sequence attempts)
 */
OilStatusData readOilSensor() {
  OilStatusData status;

  // Reset ISR state machine to known initial state
  pulseStage = STAGE_SYNC;
  sequenceComplete = false;
  captureEnabled = true;
  haveHighTime = false;
  lastRiseUs = 0;
  lastHighUs = 0;
  oilSensorStatus = 0;
  oilTemperature = NAN;
  oilPressure = NAN;

  // Attach interrupt - fires on BOTH edges for PWM decoding
  attachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN), oilSensorPWMInterrupt, CHANGE);

  // Wait for ISR to capture complete sequence (or timeout)
  // The ISR sets sequenceComplete=true after valid S1→T1→T2
  const uint32_t timeout_us = 30000;  // 30ms timeout
  uint32_t start_wait = micros();
  while(!sequenceComplete) {
    if ((uint32_t)(micros() - start_wait) > timeout_us) {
      break;  // Timeout - sensor may be disconnected or malfunctioning
    }
    delayMicroseconds(100);  // Yield briefly to avoid busy-waiting
  }

  // Disable capture and detach interrupt
  captureEnabled = false;
  detachInterrupt(digitalPinToInterrupt(OIL_PRESSURE_PIN));

  // Process results and build return struct
  if (!sequenceComplete || oilSensorStatus == 0) {
    // Timeout or invalid sequence - return NAN values
    status.oilPressure = NAN;
    status.oilTemperature = NAN;
    status.oilSensorStatus = 0;
  } else {
    // Valid sequence captured - process based on diagnostic status
    double temp = oilTemperature;
    double pressure = oilPressure;

    // Invalidate specific readings based on sensor-reported failures
    if (oilSensorStatus == 2) {
      pressure = NAN;  // Pressure sensor failed, temp still valid
    } else if (oilSensorStatus == 3) {
      temp = NAN;      // Temperature sensor failed, pressure still valid
    } else if (oilSensorStatus == 4) {
      temp = NAN;      // Hardware failure - both invalid
      pressure = NAN;
    }

    // Apply EWMA low-pass filtering for smooth gauge display
    if (!isnan(temp)) {
      temp = oilTempFilter.filter(temp);
    }
    if (!isnan(pressure)) {
      pressure = oilPressureFilter.filter(pressure);
      // Convert from absolute to gauge pressure (subtract ambient baseline)
      // Uses 0.5 bar fallback if calibration hasn't been performed
      pressure = toGaugeBar(pressure);
    }

    status.oilPressure = pressure;
    status.oilTemperature = temp;
    status.oilSensorStatus = oilSensorStatus;
  }
  return status;
}

/**
 * Boost pressure sensor approximate the following equation:
 * pressure = BOOST_COEFFICIENT x voltage - BOOST_INTERCEPT
 * I used excel to calculate the equation using linear regression on the datasheet provided.
 */
double readBoostPressureSensor() {
  double r1, r2, volts, volts5v, boostPressureSensor;
  r1 = 5600.0;
  r2 = 10000.0;
  // read from analog in on main board.
  int raw = analogRead(BOOST_PRESSURE_PIN);
  volts = raw / 4095.0F * 3.4;
  // calc what the lower 3.3v signal would be in 5v using voltage divider equation
  volts5v = volts * (r1 + r2) / r2;
  boostPressureSensor = (BOOST_COEFFICIENT * volts5v + BOOST_INTERCEPT);
  return boostFilter.filter(boostPressureSensor);
}

double readEgtSensor() {
  double egt = thermocouple.readCelsius();
  if (isnan(egt)) {
    // Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    // if (e & MAX31855_FAULT_OPEN) // Serial.println("FAULT: Thermocouple is open - no connections.");
    // if (e & MAX31855_FAULT_SHORT_GND) // Serial.println("FAULT: Thermocouple is short-circuited to GND.");
    // if (e & MAX31855_FAULT_SHORT_VCC) // Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
  }
  return egtFilter.filter(egt);
}

void readSensors() {

  #if ENABLE_ATMOS_SENSOR
    readAtmosPressureSensor();
    readAtmosTemperatureSensor();
  #endif

  #if ENABLE_INTERCOOLER_SENSOR
    readIntercoolerTemperatureSensor();
  #endif

  #if ENABLE_OIL_SENSOR
    readOilSensor();
  #endif

  #if ENABLE_BOOST_SENSOR
    readBoostPressureSensor();
  #endif

  #if ENABLE_EGT_SENSOR
    readEgtSensor();
  #endif

  #if DEBUG
  // if (count % 100 == 0) {

  //   #if ENABLE_ATMOS_SENSOR
  //     Serial.print("Atmos:");
  //     Serial.print(atmosTemp);
  //     Serial.print(" °C ");
  //     Serial.print(atmosPressure / 100000.0F);
  //     Serial.println(" Bar");
  //   #endif

  //   #if ENABLE_INTERCOOLER_SENSOR
  //     Serial.print("intercooler temp:");
  //     Serial.print(intercoolerTemp);
  //     Serial.print(" °C; ");
  //   #endif

  //   #if ENABLE_OIL_SENSOR
  //     Serial.print("Oil temp:");
  //     Serial.print(oilTemp);
  //     Serial.print("°C; ");

  //     Serial.print("Oil pressure:");
  //     Serial.print(oilPressure);
  //     Serial.print(" Bar; ");
  //   #endif

  //   #if ENABLE_EGT_SENSOR 
  //     Serial.print("EGT:");
  //     Serial.print(egt);
  //     Serial.print(" °C; ");
  //   #endif

  //   #if ENABLE_BOOST_SENSOR
  //     Serial.print("Boost:");
  //     Serial.print(boostPressure);
  //     Serial.print(" psi (");
  //     Serial.print(boostPressure / PSI_BAR_CONVERSION);
  //     Serial.print(" Bar);");
  //   #endif

  //   Serial.println();
  // }
  #endif
}
