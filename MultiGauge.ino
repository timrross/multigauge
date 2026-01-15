#include <lvgl.h>
#include "constants.h"
#include "display.h"
#include "sensor.h"
#include "ui.h"

SemaphoreHandle_t gui_mutex;
unsigned long lastTickMillis = 0;

// Define Queue handle
QueueHandle_t QueueHandle;
const int QueueElementSize = 10;

void setup() {

  #if DEBUG
    Serial.begin(115200);
  #endif

  // Create the queue which will have <QueueElementSize> number of elements, each of size `SensorData` and pass the address to <QueueHandle>.
  QueueHandle = xQueueCreate(QueueElementSize, sizeof(SensorData));

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

  initSensors();

  #if ENABLE_OIL_SENSOR
    #if DEBUG
      Serial.println("Calibrating oil sensor ambient (engine off)…");
    #endif
    // Give the sensor a brief settling time
    delay(300);

    // Try to calibrate the ambient baseline (tare)
    bool oilCalOK = calibrateOilZero(64, 2000);  // ~64 good frames, max 2s

    #if DEBUG
      if (oilCalOK) {
        Serial.printf("Oil ambient baseline set to %.3f bar\n", getOilAmbientBar());
      } else {
        Serial.println("Oil ambient calibration failed; using fallback (0.5 bar)");
      }
    #endif
  #endif

  xTaskCreatePinnedToCore(Task_LVGL,    // Pointer to the task entry function.
                          "Task_LVGL",  // A descriptive name for the task.
                          1024 * 10,    // The size of the task stack specified as the number of bytes
                          NULL,         // Pointer that will be used as the parameter for the task being created.
                          4,            // The priority at which the task should run.
                          NULL,         // Used to pass back a handle by which the created task can be referenced.
                          0);           // The core to which the task is pinned to, or tskNO_AFFINITY if the task has no core affinity.
  xTaskCreatePinnedToCore(Task_Read_Sensors,
                          "Task_Read_Sensors",
                          1024 * 2,
                          NULL,
                          1,
                          NULL,
                          1);
  xTaskCreatePinnedToCore(Task_Read_Boost_Sensor,
                          "Task_Read_Boost_Sensor",
                          1024 * 2,
                          NULL,
                          2,
                          NULL,
                          1);
  xTaskCreatePinnedToCore(Task_Screen_Update,
                          "Task_Screen_Update",
                          1024 * 3,
                          NULL,
                          3,
                          NULL,
                          1);

}

void loop() {}

void Task_LVGL(void *pvParameters) {
  // Initialize LVGL
  initDisplay();

  initLVGL();

  initUI();

  initTouch();

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

// Boost sensor read needs to be more often than the other sensors.
void Task_Read_Boost_Sensor(void *pvParameters) {
  SensorData data;
  while (1) {

    #if ENABLE_BOOST_SENSOR
      data.boostPressure = readBoostPressureSensor();
    #endif
    
    int ret = xQueueSend(QueueHandle, (void *)&data, 0);
    if (ret == pdTRUE) {
      // The message was successfully sent.
      //Serial.println("The message was successfully sent.");
    } else if (ret == errQUEUE_FULL) {
      // Since we are checking uxQueueSpacesAvailable this should not occur, however if more than one task should
      //   write into the same queue it can fill-up between the test and actual send attempt
      Serial.println("Task_Read_Boost_Sensor was unable to send data into the Queue");
    }  // Queue send check

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


void Task_Read_Sensors(void *pvParameters) {
  SensorData data;
  OilStatusData oilStatus;
  while (1) {
    // Read Sensors
      #if ENABLE_ATMOS_SENSOR
        data.atmosPressure = readAtmosPressureSensor();
        data.atmosTemperature = readAtmosTemperatureSensor();
      #endif

      #if ENABLE_INTERCOOLER_SENSOR
        data.intercoolerTemperature = readIntercoolerTemperatureSensor();
      #endif

      #if ENABLE_OIL_SENSOR
        oilStatus = readOilSensor();
        data.oilPressure = oilStatus.oilPressure;
        data.oilTemperature = oilStatus.oilTemperature;
      #endif

      #if ENABLE_EGT_SENSOR
        data.egt = readEgtSensor();
      #endif
    
    int ret = xQueueSend(QueueHandle, (void *)&data, 0);
    if (ret == pdTRUE) {
      // The message was successfully sent.
      //Serial.println("The message was successfully sent.");
    } else if (ret == errQUEUE_FULL) {
      // Since we are checking uxQueueSpacesAvailable this should not occur, however if more than one task should
      //   write into the same queue it can fill-up between the test and actual send attempt
      Serial.println("Task_Read_Sensors was unable to send data into the Queue");
    }  // Queue send check

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void Task_Screen_Update(void *pvParameters) {

  SensorData data;
  char label_char[100];

  while (1) {
    //Serial.printf("\n[Task_Screen_Update] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
    if (QueueHandle != NULL) {  // Sanity check just to make sure the queue actually exists
      if (xQueueReceive(QueueHandle, &data, portMAX_DELAY)) {
        //Serial.println("[Task_Screen_Update] The data was successfully received.");
        if (xSemaphoreTake(gui_mutex, portMAX_DELAY) == pdTRUE) {

            #if ENABLE_BOOST_SENSOR
              if (!isnan(data.boostPressure)) {
                setBoostPressure(data.boostPressure);
              }
            #endif

            #if ENABLE_OIL_SENSOR              
              if (!isnan(data.oilTemperature)) {
                setOilTemperature(data.oilTemperature);
              }
              if (!isnan(data.oilPressure)) {
                setOilPressure(data.oilPressure);
              }
            #endif

            #if ENABLE_EGT_SENSOR              
              if (!isnan(data.egt)) {
                setEgt(data.egt);
              }
            #endif

            #if ENABLE_INTERCOOLER_SENSOR              
              if (!isnan(data.intercoolerTemperature)) {
                setIntercoolerTemperature(data.intercoolerTemperature);
              }
            #endif
          // Release the semaphore after LVGL operations
          xSemaphoreGive(gui_mutex);
        }
      } else {
        //Serial.println("[Task_Screen_Update] It was unable to receive data from the Queue.");
      }
    }  // Sanity check
    // Don't need to update the screen so often
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}
