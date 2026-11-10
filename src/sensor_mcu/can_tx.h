#ifndef CAN_TX_H
#define CAN_TX_H

#include <Arduino.h>
#include "can_protocol.h"
#include "sensor.h"

// Initialize TWAI (CAN) peripheral
bool initCAN();

// Call about once a second. Brings the bus back after a bus-off or a stop.
void maintainCAN();

// Number of frames that could not be queued for sending since boot.
uint32_t canTxFailCount();

// Transmit sensor data over CAN bus
void sendBoostMessage(const Reading& boost);
void sendOilMessage(const Reading& temp, const Reading& pressure);
void sendEgtMessage(const Reading& egt);
void sendIntercoolerMessage(const Reading& temp);
void sendHeartbeat(uint8_t faultFlags);

#endif // CAN_TX_H
