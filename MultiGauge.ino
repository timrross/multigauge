#include <lvgl.h>
#include "constants.h"
#include "display.h"
#include "sensor.h"
#include "ui.h"

SemaphoreHandle_t gui_mutex;
unsigned long lastTickMillis = 0;

// Define Queue handle
QueueHandle_t QueueHandle;
const int QueueElementSize = 5;
typedef struct {
  float boostPressure;
} message_t;

void setup() {

  #if DEBUG
    Serial.begin(115200);
  #endif

  // Create the queue which will have <QueueElementSize> number of elements, each of size `message_t` and pass the address to <QueueHandle>.
  QueueHandle = xQueueCreate(QueueElementSize, sizeof(message_t));

  // Check if the queue was successfully created
  if (QueueHandle == NULL) {
    Serial.println("Queue could not be created. Halt.");
    while (1) delay(1000);  // Halt at this point as is not possible to continue
  }

  gui_mutex = xSemaphoreCreateMutex();
  if (gui_mutex == NULL) {
    // Handle semaphore creation failure
    Serial.println("semaphore creation failure");
    return;
  }

  xTaskCreatePinnedToCore(Task_LVGL,    // Pointer to the task entry function.
                          "Task_LVGL",  // A descriptive name for the task.
                          1024 * 10,    // The size of the task stack specified as the number of bytes
                          NULL,         // Pointer that will be used as the parameter for the task being created.
                          3,            // The priority at which the task should run.
                          NULL,         // Used to pass back a handle by which the created task can be referenced.
                          0);           // The core to which the task is pinned to, or tskNO_AFFINITY if the task has no core affinity.

}

void loop() {}

void Task_LVGL(void *pvParameters) {
  // Initialize LVGL
  initDisplay();

  initLVGL();

  initUI();

  // Main LVGL loop
  while (1) {
    unsigned int tickPeriod = millis() - lastTickMillis;
    lv_tick_inc(tickPeriod);
    lastTickMillis = millis();

    // Take the semaphore to access LVGL resources
    if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {
      // Call LVGL's main task handler
      lv_timer_handler();

      // Release the semaphore after LVGL operations
      xSemaphoreGive(gui_mutex);
    }

    // Delay to control LVGL's refresh rate
    vTaskDelay(pdMS_TO_TICKS(5));  // Adjust as needed
  }
}