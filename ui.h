#ifndef UI_H
#define UI_H

#include <stdint.h>

void initUI();
void updateUI(SensorData data);
void setBoostPressure(double value);
void setOilTemperature(double value);
void setOilPressure(double value);
void setEgt(double value);
void setIntercoolerTemperature(double value);
void updateTouchDebugDots(uint8_t count, const int16_t *xs, const int16_t *ys);

#endif // UI_H
