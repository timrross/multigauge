#include <Arduino.h>
#include "sensor_types.h"
#include "sensor.h"
#include "can_tx.h"

// ============================================================
// FreeRTOS Task Handles
// ============================================================

TaskHandle_t taskBoostHandle = NULL;
TaskHandle_t taskSensorsHandle = NULL;
TaskHandle_t taskHeartbeatHandle = NULL;

// ============================================================
// Task: Read Boost Sensor (high frequency - 100ms)
// ============================================================

void Task_Read_Boost(void *pvParameters) {
  uint32_t cycle = 0;
  while (1) {
    #if ENABLE_BOOST_SENSOR
      double boost = readBoostPressureSensor();
      sendBoostMessage(boost, !isnan(boost));

      // Print every 10th cycle (~1s) to avoid flooding serial
      if (cycle % 10 == 0) {
        Serial.printf("[BOOST] %.2f psi\n", boost);
      }
    #endif

    cycle++;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================================================
// Task: Read Other Sensors (lower frequency - 500-1000ms)
// ============================================================

void Task_Read_Sensors(void *pvParameters) {
  uint32_t cycle = 0;

  while (1) {
    // Oil sensor - every 500ms (every cycle)
    #if ENABLE_OIL_SENSOR
      OilStatusData oilData = readOilSensor();
      sendOilMessage(
        oilData.oilTemperature,
        oilData.oilPressure,
        oilData.oilSensorStatus,
        oilData.oilSensorStatus == 1
      );
      Serial.printf("[OIL] status=%u  temp=%.1f C  pressure=%.2f bar\n",
        oilData.oilSensorStatus, oilData.oilTemperature, oilData.oilPressure);
    #endif

    // EGT sensor - every 500ms (every cycle)
    #if ENABLE_EGT_SENSOR
      double egt = readEgtSensor();
      sendEgtMessage(egt, !isnan(egt));
      Serial.printf("[EGT] %.1f C\n", egt);
    #endif

    // Slower sensors - every 1000ms (every other cycle)
    if (cycle % 2 == 0) {
      #if ENABLE_INTERCOOLER_SENSOR
        double intercooler = readIntercoolerTemperatureSensor();
        sendIntercoolerMessage(intercooler, !isnan(intercooler));
        Serial.printf("[INTERCOOLER] %.1f C\n", intercooler);
      #endif

      // Note: Atmospheric sensor (BMP280) is on display MCU via I2C
      // If enabled, display MCU reads it directly - no CAN message needed
    }

    cycle++;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ============================================================
// Task: Heartbeat (1000ms)
// ============================================================

void Task_Heartbeat(void *pvParameters) {
  while (1) {
    sendHeartbeat();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("MultiGauge Sensor MCU starting...");

  // Initialize CAN bus
  if (!initCAN()) {
    Serial.println("CAN initialization failed!");
    while (1) delay(1000);
  }
  Serial.println("CAN initialized at 500 kbit/s");

  // Initialize sensors
  initSensors();
  Serial.println("Sensors initialized");

  // Calibrate oil sensor ambient pressure
  #if ENABLE_OIL_SENSOR
    Serial.println("Calibrating oil sensor ambient (engine off)...");
    delay(300);
    bool oilCalOK = calibrateOilZero(64, 2000);
    if (oilCalOK) {
      Serial.printf("Oil ambient baseline: %.3f bar\n", getOilAmbientBar());
    } else {
      Serial.println("Oil calibration failed; using fallback (0.5 bar)");
    }
  #endif

  // Create FreeRTOS tasks
  // ESP32-C3 is single-core, so no core pinning needed
  xTaskCreate(
    Task_Read_Boost,
    "Task_Boost",
    2048,
    NULL,
    2,  // Higher priority for boost (fast updates)
    &taskBoostHandle
  );

  xTaskCreate(
    Task_Read_Sensors,
    "Task_Sensors",
    4096,  // Larger stack for oil sensor ISR processing
    NULL,
    1,
    &taskSensorsHandle
  );

  xTaskCreate(
    Task_Heartbeat,
    "Task_Heartbeat",
    1024,
    NULL,
    0,  // Lowest priority
    &taskHeartbeatHandle
  );

  Serial.println("Tasks started");
}

// ============================================================
// Loop (unused - FreeRTOS handles everything)
// ============================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
