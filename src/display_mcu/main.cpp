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

// Shared sensor data (protected by sensor_data_mutex)
SensorData sensorData;

// ============================================================
// Task: LVGL Handler (Core 0)
// ============================================================

void Task_LVGL(void *pvParameters) {
  // Hold the GUI mutex through init: Task_Screen_Update starts immediately on
  // the other core and calls into LVGL under this mutex, so widget creation
  // here must not interleave with it.
  xSemaphoreTake(gui_mutex, portMAX_DELAY);
  initDisplay();
  initLVGL();
  initUI();  // Shows splash screen and creates gauge screen
  initTouch();
  xSemaphoreGive(gui_mutex);

  // Run LVGL briefly to display splash screen.
  // Timing comes from lv_tick_set_cb(millis_cb) set in initLVGL(), so we must
  // NOT also call lv_tick_inc() here (that would double-count elapsed time).
  for (int i = 0; i < 100; i++) {  // ~500ms of splash
    // Hold the GUI mutex: Task_Screen_Update is already running and also
    // calls into LVGL, which is not thread-safe.
    if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {
      lv_timer_handler();
      xSemaphoreGive(gui_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  // Transition to gauge screen (under the GUI mutex, like every other LVGL call)
  if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {
    showGaugeScreen();
    xSemaphoreGive(gui_mutex);
  }

  // Main LVGL loop
  while (1) {
    uint32_t idle_ms = 5;
    if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {
      idle_ms = lv_timer_handler();  // returns time until next scheduled work
      xSemaphoreGive(gui_mutex);
    }

    // Sleep until LVGL next needs servicing, but cap it so touch/gesture
    // polling and needle updates stay responsive. Floor at 1 tick to yield.
    if (idle_ms < 2) idle_ms = 2;
    if (idle_ms > 16) idle_ms = 16;
    vTaskDelay(pdMS_TO_TICKS(idle_ms));
  }
}

// ============================================================
// Task: CAN Receiver (Core 1)
// ============================================================

void Task_CAN_Receive(void *pvParameters) {
  uint32_t cycle = 0;
  while (1) {
    // Process incoming CAN messages (protected by mutex)
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      processCANMessages(&sensorData);
      xSemaphoreGive(sensor_data_mutex);
    }
    if (++cycle % 100 == 0) {
      maintainCAN();  // About once a second: recover from BUS_OFF / STOPPED
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Check every 10ms
  }
}

// ============================================================
// Task: Screen Update (Core 1)
// ============================================================

void Task_Screen_Update(void *pvParameters) {
  // Kept between loops: if the mutex is busy for one cycle, the previous copy is
  // shown again. (A fresh SensorData is all NAN and would flash "--".)
  SensorData localData;
  while (1) {
    // Copy sensor data under mutex protection
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

// Serial is only started in DEBUG builds, so only print in DEBUG builds.
#if DEBUG
  #define DEBUG_PRINTLN(msg) Serial.println(msg)
#else
  #define DEBUG_PRINTLN(msg) do {} while (0)
#endif

// Start-up failed in a way that cannot be worked around: reboot and try again
// rather than sit on a blank screen.
static void fatal(const char* msg) {
  DEBUG_PRINTLN(msg);
  delay(3000);
  ESP.restart();
}

void setup() {
  #if DEBUG
    Serial.begin(115200);
    Serial.println("MultiGauge Display MCU starting...");
  #endif

  // Create mutexes
  gui_mutex = xSemaphoreCreateMutex();
  sensor_data_mutex = xSemaphoreCreateMutex();
  if (gui_mutex == NULL || sensor_data_mutex == NULL) {
    fatal("Mutex creation failed");
  }

  // Initialize CAN bus
  if (!initCAN()) {
    fatal("CAN initialization failed");
  }
  DEBUG_PRINTLN("CAN initialized at 500 kbit/s");

  // Create FreeRTOS tasks
  bool ok = xTaskCreatePinnedToCore(
    Task_LVGL,
    "Task_LVGL",
    1024 * 10,
    NULL,
    4,
    NULL,
    0  // Core 0 for LVGL
  ) == pdPASS;

  ok = ok && xTaskCreatePinnedToCore(
    Task_CAN_Receive,
    "Task_CAN_RX",
    1024 * 4,  // prints in debug builds, and printf alone needs about 1.5 KB
    NULL,
    2,
    NULL,
    1  // Core 1 for CAN
  ) == pdPASS;

  ok = ok && xTaskCreatePinnedToCore(
    Task_Screen_Update,
    "Task_Update",
    1024 * 3,
    NULL,
    3,
    NULL,
    1  // Core 1 for UI updates
  ) == pdPASS;

  if (!ok) {
    fatal("Task creation failed");
  }
  DEBUG_PRINTLN("Tasks started");
}

// ============================================================
// Loop (unused - FreeRTOS handles everything)
// ============================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
