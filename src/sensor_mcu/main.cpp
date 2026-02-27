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
    // Oil sensors — every 500ms (every cycle)
    #if ENABLE_OIL_SENSOR
      OilStatusData oilData = readOilSensor();
      bool oilValid = !isnan(oilData.oilTemperature) && !isnan(oilData.oilPressure);
      sendOilMessage(
        oilData.oilTemperature,
        oilData.oilPressure,
        oilValid ? OIL_STATUS_OK : OIL_STATUS_TIMEOUT,
        oilValid
      );
      if (oilValid) {
        Serial.printf("[OIL] temp=%.1f C  pressure=%.2f bar\n",
          oilData.oilTemperature, oilData.oilPressure);
      } else {
        Serial.println("[OIL] sensor invalid");
      }
    #endif

    // EGT sensor — every 500ms (every cycle)
    #if ENABLE_EGT_SENSOR
      double egt = readEgtSensor();
      sendEgtMessage(egt, !isnan(egt));
      Serial.printf("[EGT] %.1f C\n", egt);
    #endif

    // Slower sensors — every 1000ms (every other cycle)
    if (cycle % 2 == 0) {
      #if ENABLE_INTERCOOLER_SENSOR
        double intake = readIntercoolerTemperatureSensor();
        sendIntercoolerMessage(intake, !isnan(intake));
        Serial.printf("[INTAKE] %.1f C\n", intake);
      #endif

      // Note: Atmospheric sensor (BMP280) is on display MCU via I2C
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
    maintainCAN();  // Detect and recover from BUS_OFF (e.g. display not yet powered)
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

  // Initialize sensors (ADS1115 + MAX31855)
  initSensors();
  Serial.println("Sensors initialized");

  // Create FreeRTOS tasks — halt on creation failure so the fault is obvious
  // rather than silently missing CAN frames or heartbeats.
  // ESP32-C3 is single-core, so no core pinning needed.
  if (xTaskCreate(Task_Read_Boost, "Task_Boost", 2048, NULL, 2,
                  &taskBoostHandle) != pdPASS) {
    Serial.println("FATAL: failed to create Task_Boost");
    while (1) delay(1000);
  }

  if (xTaskCreate(Task_Read_Sensors, "Task_Sensors", 4096, NULL, 1,
                  &taskSensorsHandle) != pdPASS) {
    Serial.println("FATAL: failed to create Task_Sensors");
    while (1) delay(1000);
  }

  if (xTaskCreate(Task_Heartbeat, "Task_Heartbeat", 1024, NULL, 0,
                  &taskHeartbeatHandle) != pdPASS) {
    Serial.println("FATAL: failed to create Task_Heartbeat");
    while (1) delay(1000);
  }

  Serial.println("Tasks started");
}

// ============================================================
// Loop (unused - FreeRTOS handles everything)
// ============================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
