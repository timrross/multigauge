#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <math.h>

// CAN Bus Configuration
#define CAN_BAUD_RATE 500  // 500 kbit/s (TWAI uses kbit/s units)

// CAN Message IDs (11-bit standard)
#define CAN_ID_BOOST        0x100
#define CAN_ID_OIL          0x101
#define CAN_ID_EGT          0x102
#define CAN_ID_INTERCOOLER  0x103
#define CAN_ID_ATMOS        0x104
#define CAN_ID_HEARTBEAT    0x1FF

// Status byte bit definitions
#define CAN_STATUS_VALID     0x01  // Bit 0: Data is valid
#define CAN_STATUS_ERROR     0x02  // Bit 1: Sensor error detected

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
    uint8_t status;
    uint8_t reserved[5];
} CanMsgBoost;

// 0x101: Oil temperature and pressure (updated at 500ms)
typedef struct {
    int16_t temp_c_x10;        // Temperature in C * 10 (range: -40 to +160)
    uint16_t pressure_bar_x100; // Pressure in bar * 100 (range: 0 to 10.5)
    uint8_t sensor_status;     // Oil sensor status code (0-4)
    uint8_t status;            // CAN status byte
    uint8_t reserved[2];
} CanMsgOil;

// 0x102: Exhaust gas temperature (updated at 500ms)
typedef struct {
    int16_t temp_c;            // EGT in C (range: 0 to 1200)
    uint8_t status;
    uint8_t reserved[5];
} CanMsgEgt;

// 0x103: Intercooler/intake temperature (updated at 1000ms)
typedef struct {
    int16_t temp_c_x10;        // Temperature in C * 10
    uint8_t status;
    uint8_t reserved[5];
} CanMsgIntercooler;

// 0x104: Atmospheric pressure and temperature (updated at 1000ms)
typedef struct {
    uint32_t pressure_pa;      // Pressure in Pa (range: 80000-120000)
    int16_t temp_c_x10;        // Temperature in C * 10
    uint8_t status;
    uint8_t reserved;
} CanMsgAtmos;

// 0x1FF: Heartbeat (updated at 1000ms)
typedef struct {
    uint8_t counter;           // Rolling counter 0-255
    uint32_t uptime_ms;        // Uptime in milliseconds
    uint8_t reserved[3];
} CanMsgHeartbeat;

#pragma pack(pop)

// ============================================================
// Encoding Functions (Sensor MCU)
// ============================================================

static inline void encodeBoostMsg(CanMsgBoost* msg, double boost_psi, bool valid) {
    if (valid && !isnan(boost_psi)) {
        msg->boost_psi_x100 = (int16_t)(boost_psi * SCALE_PRESSURE_100);
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->boost_psi_x100 = 0;
        msg->status = 0;
    }
}

static inline void encodeOilMsg(CanMsgOil* msg, double temp_c, double pressure_bar,
                                 uint8_t oil_status, bool valid) {
    if (valid && !isnan(temp_c) && !isnan(pressure_bar)) {
        msg->temp_c_x10 = (int16_t)(temp_c * SCALE_TEMP_10);
        msg->pressure_bar_x100 = (uint16_t)(pressure_bar * SCALE_PRESSURE_100);
        msg->sensor_status = oil_status;
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->temp_c_x10 = 0;
        msg->pressure_bar_x100 = 0;
        msg->sensor_status = oil_status;
        msg->status = CAN_STATUS_ERROR;
    }
}

static inline void encodeEgtMsg(CanMsgEgt* msg, double temp_c, bool valid) {
    if (valid && !isnan(temp_c)) {
        msg->temp_c = (int16_t)temp_c;
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->temp_c = 0;
        msg->status = 0;
    }
}

static inline void encodeIntercoolerMsg(CanMsgIntercooler* msg, double temp_c, bool valid) {
    if (valid && !isnan(temp_c)) {
        msg->temp_c_x10 = (int16_t)(temp_c * SCALE_TEMP_10);
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->temp_c_x10 = 0;
        msg->status = 0;
    }
}

static inline void encodeAtmosMsg(CanMsgAtmos* msg, double pressure_pa, double temp_c, bool valid) {
    if (valid && !isnan(pressure_pa) && !isnan(temp_c)) {
        msg->pressure_pa = (uint32_t)pressure_pa;
        msg->temp_c_x10 = (int16_t)(temp_c * SCALE_TEMP_10);
        msg->status = CAN_STATUS_VALID;
    } else {
        msg->pressure_pa = 0;
        msg->temp_c_x10 = 0;
        msg->status = 0;
    }
}

static inline void encodeHeartbeatMsg(CanMsgHeartbeat* msg, uint8_t counter, uint32_t uptime) {
    msg->counter = counter;
    msg->uptime_ms = uptime;
}

// ============================================================
// Decoding Functions (Display MCU)
// ============================================================

static inline double decodeBoostPsi(const CanMsgBoost* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->boost_psi_x100 / SCALE_PRESSURE_100;
    }
    return NAN;
}

static inline double decodeOilTempC(const CanMsgOil* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->temp_c_x10 / SCALE_TEMP_10;
    }
    return NAN;
}

static inline double decodeOilPressureBar(const CanMsgOil* msg) {
    if (msg->status & CAN_STATUS_VALID) {
        return (double)msg->pressure_bar_x100 / SCALE_PRESSURE_100;
    }
    return NAN;
}

static inline uint8_t decodeOilSensorStatus(const CanMsgOil* msg) {
    return msg->sensor_status;
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
