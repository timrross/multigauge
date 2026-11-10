#ifndef CAN_RX_H
#define CAN_RX_H

#include <Arduino.h>
#include "can_protocol.h"
#include "sensor_types.h"

// ESP32-S3 Qualia CAN pins (using analog header)
#define CAN_TX_PIN  16  // A1 (GPIO16)
#define CAN_RX_PIN  17  // A0 (GPIO17)

// Initialize TWAI (CAN) peripheral for receiving
bool initCAN();

// Check for and process incoming CAN messages, and blank (NAN) any value that is
// invalid or has stopped arriving. Returns true if any message was received.
bool processCANMessages(SensorData* data);

// Call about once a second. Brings the bus back after a bus-off or a stop.
void maintainCAN();

// HB_FAULT_* bits from the sensor node's latest heartbeat
uint8_t getSensorFaultFlags();

// Get last received heartbeat counter
uint8_t getLastHeartbeatCounter();

// Get time since last heartbeat (ms) - for connection monitoring
uint32_t getTimeSinceLastHeartbeat();

// Check if sensor MCU connection is active (heartbeat within timeout)
#define HEARTBEAT_TIMEOUT_MS 3000
bool isSensorConnected();

#endif // CAN_RX_H
