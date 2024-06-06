#include "constants.h"
#include "display.h"
#include "sensor.h"
#include "ui.h"

void setup() {

  #if DEBUG
    Serial.begin(115200);
  #endif 
  initDisplay();
  initUI();
  initSensors();
  count = 0;
}

void loop() {

  readSensors();
  updateUI();
  flushDisplay();

  count++;
  delay(5);
}