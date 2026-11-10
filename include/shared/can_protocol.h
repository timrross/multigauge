#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <math.h>

// CAN bus: 500 kbit/s, 11-bit IDs, every frame 8 bytes, little-endian fields.
// Both nodes are ESP32 (little-endian), so the packed structs are sent as-is.

// CAN Message IDs (11-bit standard)
#define CAN_ID_BOOST        0x100
#define CAN_ID_OIL          0x101
#define CAN_ID_EGT          0x102
#define CAN_ID_INTERCOOLER  0x103
#define CAN_ID_ATMOS        0x104
#define CAN_ID_HEARTBEAT    0x1FF

// Status byte: bit 0 = value is valid.
#define CAN_STATUS_VALID        0x01
// Oil frame carries two values, each with its own valid bit in the status byte.
#define CAN_OIL_TEMP_VALID      0x01
#define CAN_OIL_PRESS_VALID     0x02

// Per-sensor fault codes (0 = no fault). Sent alongside each value for diagnostics.
#define FAULT_NONE          0x00
#define FAULT_ADC           0x01  // ADC did not answer (I2C fault or module missing)
#define FAULT_SIGNAL_LOW    0x02  // Signal below the valid range (open wire on a pressure sensor, shorted NTC)
#define FAULT_SIGNAL_HIGH   0x03  // Signal above the valid range (short to supply, open NTC)
#define FAULT_RANGE         0x04  // Converted value is not physically plausible
// EGT fault byte is a bit mask. Bits 0-2 are the MAX31855's own fault bits.
#define EGT_FAULT_OPEN        0x01  // Thermocouple open circuit
#define EGT_FAULT_SHORT_GND   0x02  // Thermocouple shorted to GND (also: grounded-junction probe)
#define EGT_FAULT_SHORT_VCC   0x04  // Thermocouple shorted to VCC
#define EGT_FAULT_NO_CHIP     0x08  // MAX31855 absent or unpowered (all-zero reply)
#define EGT_FAULT_RANGE       0x10  // Reading outside -50 to 1300 C

// Heartbeat fault_flags: one bit per sensor that is currently faulted.
#define HB_FAULT_ADC          0x01
#define HB_FAULT_OIL_PRESS    0x02
#define HB_FAULT_OIL_TEMP     0x04
#define HB_FAULT_BOOST        0x08
#define HB_FAULT_INTAKE       0x10
#define HB_FAULT_EGT          0x20

// Scaling factors
#define SCALE_PRESSURE_100   100   // 0.01 bar/psi resolution
#define SCALE_TEMP_10        10    // 0.1 C resolution

// ============================================================
// CAN Message Structures (packed, 8 bytes each)
// ============================================================

#pragma pack(push, 1)

// 0x100: Boost pressure (updated at 100ms)
typedef struct {
    int16_t boost_psi_x100;    // Boost in psi * 100 (range: -14.7 to +50.0)
    uint8_t status;            // CAN_STATUS_VALID
    uint8_t fault;             // FAULT_* code
    uint8_t reserved[4];
} CanMsgBoost;

// 0x101: Oil temperature and pressure (updated at 500ms)
typedef struct {
    int16_t temp_c_x10;         // Temperature in C * 10 (range: -40 to +250)
    uint16_t pressure_bar_x100; // Pressure in bar * 100 (range: 0 to 10.5)
    uint8_t sensor_status;      // (temp FAULT_* code << 4) | pressure FAULT_* code
    uint8_t status;             // CAN_OIL_TEMP_VALID | CAN_OIL_PRESS_VALID
    uint8_t reserved[2];
} CanMsgOil;

// 0x102: Exhaust gas temperature (updated at 500ms)
typedef struct {
    int16_t temp_c;            // EGT in C (range: -50 to 1300)
    uint8_t status;            // CAN_STATUS_VALID
    uint8_t fault;             // EGT_FAULT_* bit mask
    uint8_t reserved[4];
} CanMsgEgt;

// 0x103: Intercooler/intake temperature (updated at 1000ms)
typedef struct {
    int16_t temp_c_x10;        // Temperature in C * 10
    uint8_t status;            // CAN_STATUS_VALID
    uint8_t fault;             // FAULT_* code
    uint8_t reserved[4];
} CanMsgIntercooler;

// 0x104: Atmospheric pressure and temperature (reserved, nothing sends it yet)
typedef struct {
    uint32_t pressure_pa;      // Pressure in Pa (range: 80000-120000)
    int16_t temp_c_x10;        // Temperature in C * 10
    uint8_t status;
    uint8_t reserved;
} CanMsgAtmos;

// 0x1FF: Heartbeat (updated at 1000ms)
typedef struct {
    uint8_t counter;           // Rolling counter 0-255
    uint32_t uptime_ms;        // Uptime in milliseconds (goes backwards if the sensor node reboots)
    uint8_t fault_flags;       // HB_FAULT_* bit mask of sensors currently faulted
    uint8_t reserved[2];
} CanMsgHeartbeat;

#pragma pack(pop)

#ifdef __cplusplus
static_assert(sizeof(CanMsgBoost) == 8, "CAN frame must be 8 bytes");
static_assert(sizeof(CanMsgOil) == 8, "CAN frame must be 8 bytes");
static_assert(sizeof(CanMsgEgt) == 8, "CAN frame must be 8 bytes");
static_assert(sizeof(CanMsgIntercooler) == 8, "CAN frame must be 8 bytes");
static_assert(sizeof(CanMsgAtmos) == 8, "CAN frame must be 8 bytes");
static_assert(sizeof(CanMsgHeartbeat) == 8, "CAN frame must be 8 bytes");
#endif

// ============================================================
// Encoding Functions (Sensor MCU)
// ============================================================

// Round to nearest and clamp, so an out-of-range value can never wrap round.
static inline int16_t canClampI16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lround(v);
}

static inline uint16_t canClampU16(double v) {
    if (v > 65535.0) return 65535;
    if (v < 0.0) return 0;
    return (uint16_t)lround(v);
}

static inline void encodeBoostMsg(CanMsgBoost* msg, double boost_psi, uint8_t fault) {
    msg->fault = fault;
    if (fault == FAULT_NONE && !isnan(boost_psi)) {
        msg->boost_psi_x100 = canClampI16(boost_psi * SCALE_PRESSURE_100);
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->boost_psi_x100 = 0;
        msg->status = 0;
    }
}

// Temperature and pressure are valid independently: a fault on one does not blank the other.
static inline void encodeOilMsg(CanMsgOil* msg, double temp_c, uint8_t temp_fault,
                                double pressure_bar, uint8_t pressure_fault) {
    msg->status = 0;
    msg->temp_c_x10 = 0;
    msg->pressure_bar_x100 = 0;
    msg->sensor_status = (uint8_t)((temp_fault << 4) | (pressure_fault & 0x0F));
    if (temp_fault == FAULT_NONE && !isnan(temp_c)) {
        msg->temp_c_x10 = canClampI16(temp_c * SCALE_TEMP_10);
        msg->status |= CAN_OIL_TEMP_VALID;
    }
    if (pressure_fault == FAULT_NONE && !isnan(pressure_bar)) {
        msg->pressure_bar_x100 = canClampU16(pressure_bar * SCALE_PRESSURE_100);
        msg->status |= CAN_OIL_PRESS_VALID;
    }
}

static inline void encodeEgtMsg(CanMsgEgt* msg, double temp_c, uint8_t fault) {
    msg->fault = fault;
    if (fault == 0 && !isnan(temp_c)) {
        msg->temp_c = canClampI16(temp_c);
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->temp_c = 0;
        msg->status = 0;
    }
}

static inline void encodeIntercoolerMsg(CanMsgIntercooler* msg, double temp_c, uint8_t fault) {
    msg->fault = fault;
    if (fault == FAULT_NONE && !isnan(temp_c)) {
        msg->temp_c_x10 = canClampI16(temp_c * SCALE_TEMP_10);
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->temp_c_x10 = 0;
        msg->status = 0;
    }
}

static inline void encodeHeartbeatMsg(CanMsgHeartbeat* msg, uint8_t counter, uint32_t uptime,
                                      uint8_t fault_flags) {
    msg->counter = counter;
    msg->uptime_ms = uptime;
    msg->fault_flags = fault_flags;
}

// ============================================================
// Decoding Functions (Display MCU). Each returns NAN when the value is not valid.
// ============================================================

static inline double decodeBoostPsi(const CanMsgBoost* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->boost_psi_x100 / SCALE_PRESSURE_100;
    }
    return NAN;
}

static inline double decodeOilTempC(const CanMsgOil* msg) {
    if (msg->status & CAN_OIL_TEMP_VALID) {
        return (double)msg->temp_c_x10 / SCALE_TEMP_10;
    }
    return NAN;
}

static inline double decodeOilPressureBar(const CanMsgOil* msg) {
    if (msg->status & CAN_OIL_PRESS_VALID) {
        return (double)msg->pressure_bar_x100 / SCALE_PRESSURE_100;
    }
    return NAN;
}

static inline double decodeEgtC(const CanMsgEgt* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->temp_c;
    }
    return NAN;
}

static inline double decodeIntercoolerTempC(const CanMsgIntercooler* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->temp_c_x10 / SCALE_TEMP_10;
    }
    return NAN;
}

static inline double decodeAtmosPressurePa(const CanMsgAtmos* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->pressure_pa;
    }
    return NAN;
}

static inline double decodeAtmosTempC(const CanMsgAtmos* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->temp_c_x10 / SCALE_TEMP_10;
    }
    return NAN;
}

#endif // CAN_PROTOCOL_H
