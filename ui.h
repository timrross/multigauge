#ifndef UI_H
#define UI_H

void initUI();
void updateUI(SensorData data);
void setBoostPressure(double value);
void setOilTemperature(double value);
void setOilPressure(double value);
void setEgt(double value);
void setIntercoolerTemperature(double value);

#endif // UI_H