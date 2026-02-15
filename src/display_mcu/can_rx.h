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

// Check for and process incoming CAN messages
// Returns true if any message was received
bool processCANMessages(SensorData* data);

// Get last received heartbeat counter
uint8_t getLastHeartbeatCounter();

// Get time since last heartbeat (ms) - for connection monitoring
uint32_t getTimeSinceLastHeartbeat();

// Check if sensor MCU connection is active (heartbeat within timeout)
#define HEARTBEAT_TIMEOUT_MS 3000
bool isSensorConnected();

#endif // CAN_RX_H
