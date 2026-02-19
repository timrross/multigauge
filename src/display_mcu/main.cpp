#include <Arduino.h>
#include <lvgl.h>
#include "sensor_types.h"
#include "display.h"
#include "touch.h"
#include "ui.h"
#include "can_rx.h"

// ============================================================
// FreeRTOS Synchronization
// ============================================================

SemaphoreHandle_t gui_mutex;
SemaphoreHandle_t sensor_data_mutex;
unsigned long lastTickMillis = 0;

// Shared sensor data (protected by sensor_data_mutex)
SensorData sensorData;

// ============================================================
// Task: LVGL Handler (Core 0)
// ============================================================

void Task_LVGL(void *pvParameters) {
  initDisplay();
  initLVGL();
  initUI();  // Shows splash screen and creates gauge screen
  initTouch();

  // Run LVGL briefly to display splash screen
  lastTickMillis = millis();
  for (int i = 0; i < 100; i++) {  // ~500ms of splash
    unsigned int tickPeriod = millis() - lastTickMillis;
    lv_tick_inc(tickPeriod);
    lastTickMillis = millis();
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  // Transition to gauge screen
  showGaugeScreen();

  // Main LVGL loop
  while (1) {
    unsigned int tickPeriod = millis() - lastTickMillis;
    lv_tick_inc(tickPeriod);
    lastTickMillis = millis();

    if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {
      lv_timer_handler();
      xSemaphoreGive(gui_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================
// Task: CAN Receiver (Core 1)
// ============================================================

void Task_CAN_Receive(void *pvParameters) {
  while (1) {
    // Process incoming CAN messages (protected by mutex)
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      processCANMessages(&sensorData);
      xSemaphoreGive(sensor_data_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Check every 10ms
  }
}

// ============================================================
// Task: Screen Update (Core 1)
// ============================================================

void Task_Screen_Update(void *pvParameters) {
  while (1) {
    // Copy sensor data under mutex protection
    SensorData localData;
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      localData = sensorData;
      xSemaphoreGive(sensor_data_mutex);
    }

    // Check connection status
    bool connected = isSensorConnected();

    // Update UI with local copy (GUI mutex for LVGL)
    if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {
      updateUI(localData);
      setConnectionStatus(connected);
      xSemaphoreGive(gui_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(25));  // Update UI at ~40Hz
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  #if DEBUG
    Serial.begin(115200);
    Serial.println("MultiGauge Display MCU starting...");
  #endif

  // Create mutexes
  gui_mutex = xSemaphoreCreateMutex();
  sensor_data_mutex = xSemaphoreCreateMutex();
  if (gui_mutex == NULL || sensor_data_mutex == NULL) {
    Serial.println("Mutex creation failed!");
    while (1) delay(1000);
  }

  // Initialize CAN bus
  if (!initCAN()) {
    Serial.println("CAN initialization failed!");
    while (1) delay(1000);
  }
  Serial.println("CAN initialized at 500 kbit/s");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    Task_LVGL,
    "Task_LVGL",
    1024 * 10,
    NULL,
    4,
    NULL,
    0  // Core 0 for LVGL
  );

  xTaskCreatePinnedToCore(
    Task_CAN_Receive,
    "Task_CAN_RX",
    1024 * 2,
    NULL,
    2,
    NULL,
    1  // Core 1 for CAN
  );

  xTaskCreatePinnedToCore(
    Task_Screen_Update,
    "Task_Update",
    1024 * 3,
    NULL,
    3,
    NULL,
    1  // Core 1 for UI updates
  );

  Serial.println("Tasks started");
}

// ============================================================
// Loop (unused - FreeRTOS handles everything)
// ============================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
