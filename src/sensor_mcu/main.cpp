#include <Arduino.h>
#include <esp_task_wdt.h>
#include "sensor_types.h"
#include "sensor.h"
#include "can_tx.h"

// ============================================================
// Shared state
// ============================================================

// HB_FAULT_* bits for the sensors that are currently faulted. Each task owns its
// own bits; the heartbeat task reads the whole byte. Updates go through
// setFaultBit() so two tasks never clobber each other's bits.
static volatile uint8_t faultFlags = 0;
static portMUX_TYPE faultMux = portMUX_INITIALIZER_UNLOCKED;

static void setFaultBit(uint8_t bit, bool faulted) {
  portENTER_CRITICAL(&faultMux);
  if (faulted) faultFlags |= bit; else faultFlags &= (uint8_t)~bit;
  portEXIT_CRITICAL(&faultMux);
}

#define WDT_TIMEOUT_MS 5000

// Something is wrong that the firmware cannot work around. Say so, then reboot
// and try again, rather than sitting silent with the gauges frozen.
static void fatal(const char* msg) {
  Serial.printf("FATAL: %s. Restarting in 3s\n", msg);
  delay(3000);
  ESP.restart();
}

// Print a reading as "12.3" or "fault 0x02" (printing NAN itself is not helpful).
static void printReading(const char* name, const Reading& r, const char* unit, int decimals) {
  if (r.fault == 0 && !isnan(r.value)) {
    Serial.printf("[%s] %.*f %s\n", name, decimals, r.value, unit);
  } else {
    Serial.printf("[%s] fault 0x%02X\n", name, r.fault);
  }
}

// ============================================================
// Task: Read Boost Sensor (high frequency - 100ms)
// ============================================================

void Task_Read_Boost(void *pvParameters) {
  esp_task_wdt_add(NULL);
  uint32_t cycle = 0;
  while (1) {
    #if ENABLE_BOOST_SENSOR
      Reading boost = readBoostPressure();
      sendBoostMessage(boost);
      setFaultBit(HB_FAULT_BOOST, boost.fault != 0);

      // Print every 10th cycle (~1s) to avoid flooding serial
      if (cycle % 10 == 0) printReading("BOOST", boost, "psi", 2);
    #endif

    cycle++;
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================================================
// Task: Read Other Sensors (500ms, intake every 1000ms)
// ============================================================

void Task_Read_Sensors(void *pvParameters) {
  esp_task_wdt_add(NULL);
  uint32_t cycle = 0;

  while (1) {
    #if ENABLE_OIL_SENSOR
      Reading oilPressure = readOilPressure();
      Reading oilTemp = readOilTemperature();
      sendOilMessage(oilTemp, oilPressure);
      setFaultBit(HB_FAULT_OIL_PRESS, oilPressure.fault != 0);
      setFaultBit(HB_FAULT_OIL_TEMP, oilTemp.fault != 0);
      printReading("OIL P", oilPressure, "bar", 2);
      printReading("OIL T", oilTemp, "C", 1);
    #endif

    #if ENABLE_EGT_SENSOR
      Reading egt = readEgt();
      sendEgtMessage(egt);
      setFaultBit(HB_FAULT_EGT, egt.fault != 0);
      printReading("EGT", egt, "C", 0);
    #endif

    // Slower sensors, every other cycle
    if (cycle % 2 == 0) {
      #if ENABLE_INTERCOOLER_SENSOR
        Reading intake = readIntakeTemperature();
        sendIntercoolerMessage(intake);
        setFaultBit(HB_FAULT_INTAKE, intake.fault != 0);
        printReading("INTAKE", intake, "C", 1);
      #endif
    }

    cycle++;
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ============================================================
// Task: Heartbeat + housekeeping (1000ms)
// ============================================================

void Task_Heartbeat(void *pvParameters) {
  esp_task_wdt_add(NULL);
  while (1) {
    maintainSensors();  // Look for the ADS1115 again if it has gone missing
    setFaultBit(HB_FAULT_ADC, !adcOk());

    sendHeartbeat(faultFlags);
    maintainCAN();      // Recover from BUS_OFF / STOPPED

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("MultiGauge Sensor MCU starting...");

  // Task watchdog: if any of the three tasks stops looping for 5s, reset the chip.
  // The Arduino core has already started the watchdog, so this normally reconfigures it.
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = WDT_TIMEOUT_MS;
  wdtConfig.idle_core_mask = 0;
  wdtConfig.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdtConfig) != ESP_OK) {
    esp_task_wdt_init(&wdtConfig);
  }

  if (!initCAN()) fatal("CAN initialization failed");
  Serial.println("CAN initialized at 500 kbit/s");

  // Never fatal: a missing ADS1115 is reported over CAN instead.
  initSensors();

  // ESP32-C3 is single-core, so no core pinning needed.
  if (xTaskCreate(Task_Read_Boost, "Task_Boost", 4096, NULL, 2, NULL) != pdPASS) {
    fatal("failed to create Task_Boost");
  }
  if (xTaskCreate(Task_Read_Sensors, "Task_Sensors", 4096, NULL, 1, NULL) != pdPASS) {
    fatal("failed to create Task_Sensors");
  }
  // Priority 1, not 0: at priority 0 it would share time with the idle task.
  if (xTaskCreate(Task_Heartbeat, "Task_Heartbeat", 4096, NULL, 1, NULL) != pdPASS) {
    fatal("failed to create Task_Heartbeat");
  }

  Serial.println("Tasks started");
}

// ============================================================
// Loop (unused - FreeRTOS handles everything)
// ============================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
