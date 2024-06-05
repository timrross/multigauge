#include "display.h"
#include "sensor.h"
#include "ui.h"

#define ENABLE_BOOST_SENSOR true
#define ENABLE_INTERCOOLER_SENSOR false
#define ENABLE_EGT_SENSOR true
#define ENABLE_OIL_SENSOR true
#define ENABLE_ATMOS_SENSOR true
#define DEBUG false

int count;

void setup() {

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