#ifndef CAN_TX_H
#define CAN_TX_H

#include <Arduino.h>
#include "can_protocol.h"
#include "sensor.h"

// Initialize TWAI (CAN) peripheral
bool initCAN();

// Call periodically to detect and recover from BUS_OFF state.
// BUS_OFF occurs after 32 unacknowledged TX frames (e.g. receiver not connected).
void maintainCAN();

// Transmit sensor data over CAN bus
void sendBoostMessage(double boost_psi, bool valid);
void sendOilMessage(double temp_c, double pressure_bar, uint8_t oil_status, bool valid);
void sendEgtMessage(double temp_c, bool valid);
void sendIntercoolerMessage(double temp_c, bool valid);
void sendAtmosMessage(double pressure_pa, double temp_c, bool valid);
void sendHeartbeat();

#endif // CAN_TX_H
