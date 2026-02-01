#ifndef UI_H
#define UI_H

#include "sensor_types.h"

void initUI();

// Transition from splash to gauge screen
void showGaugeScreen();

// Update individual sensor displays
void setBoostPressure(double value);
void setOilTemperature(double value);
void setOilPressure(double value);
void setEgt(double value);
void setIntercoolerTemperature(double value);

// Update connection status indicator
void setConnectionStatus(bool connected);

// Update all sensors from SensorData struct
void updateUI(const SensorData& data);

#endif // UI_H
