#ifndef UI_H
#define UI_H

#include "sensor_types.h"

void initUI();

// Transition from splash to gauge screen
void showGaugeScreen();

// Update individual sensor displays
// Boost is called on every screen update so the needle can glide between readings.
// readingMs is when the reading arrived (millis).
void setBoostPressure(double value, uint32_t readingMs);
void setOilTemperature(double value);
void setOilPressure(double value);
void setEgt(double value);
void setIntercoolerTemperature(double value);

// Update connection status indicator
void setConnectionStatus(bool connected);

// Update all sensors from SensorData struct
void updateUI(const SensorData& data);

#ifdef DEMO_MODE
// Demo builds only: change view as a left swipe would. Call with the GUI mutex held.
void demoNextView();
#endif

#endif // UI_H
