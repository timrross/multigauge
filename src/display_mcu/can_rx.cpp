#include <Arduino.h>
#include <driver/twai.h>
#include "can_rx.h"
#include "can_protocol.h"
#include "sensor_types.h"

// Note: No filtering here - the sensor MCU already filters the data.

// Heartbeat tracking
static uint8_t lastHeartbeatCounter = 0;
static uint32_t lastHeartbeatTime = 0;
static bool heartbeatSeen = false;
static uint32_t lastSensorUptime = 0;
static uint8_t sensorFaultFlags = 0;
static uint32_t missedHeartbeats = 0;
static uint32_t sensorReboots = 0;

// When each value was last received. A value older than its limit is blanked,
// so a frame that stops arriving cannot leave a stale number on the gauge.
static uint32_t lastBoostRx = 0, lastOilRx = 0, lastEgtRx = 0, lastIntercoolerRx = 0;
#define STALE_BOOST_MS  1000   // sent every 100ms
#define STALE_SLOW_MS   3000   // sent every 500-1000ms

bool initCAN() {
  // Configure TWAI (CAN) peripheral
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );
  g_config.rx_queue_len = 32;  // about 16 frames/s arrive; leave room if the UI stalls

  // 500 kbit/s timing
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // Accept all messages
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    return false;
  }

  return true;
}

void maintainCAN() {
  // Same two-step recovery as the sensor node: initiate recovery on BUS_OFF,
  // then start the driver again once it has dropped to STOPPED.
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;
  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

uint8_t getLastHeartbeatCounter() {
  return lastHeartbeatCounter;
}

uint32_t getTimeSinceLastHeartbeat() {
  return millis() - lastHeartbeatTime;
}

uint8_t getSensorFaultFlags() {
  return sensorFaultFlags;
}

bool isSensorConnected() {
  // Connected if a heartbeat has been received and the latest is within the timeout
  if (!heartbeatSeen) {
    return false;
  }
  return getTimeSinceLastHeartbeat() < HEARTBEAT_TIMEOUT_MS;
}

// Copy a frame's payload into a message struct. False if the frame is too short.
template <typename T>
static bool unpack(const twai_message_t& message, T* out) {
  if (message.data_length_code < sizeof(T)) return false;
  memcpy(out, message.data, sizeof(T));
  return true;
}

bool processCANMessages(SensorData* data) {
  twai_message_t message;
  bool received = false;

  // Process all available messages in the RX queue
  while (twai_receive(&message, 0) == ESP_OK) {
    // Only standard data frames belong to this protocol
    if (message.extd || message.rtr) continue;
    received = true;
    uint32_t now = millis();

    // Every decode returns NAN for an invalid value, and that NAN is stored:
    // the UI shows "--" rather than holding the last good number.
    switch (message.identifier) {
      case CAN_ID_BOOST: {
        CanMsgBoost msg;
        if (!unpack(message, &msg)) break;
        data->boostPressure = decodeBoostPsi(&msg);
        data->boostRxMs = now;
        lastBoostRx = now;
        break;
      }

      case CAN_ID_OIL: {
        CanMsgOil msg;
        if (!unpack(message, &msg)) break;
        data->oilTemperature = decodeOilTempC(&msg);
        data->oilPressure = decodeOilPressureBar(&msg);
        lastOilRx = now;
        break;
      }

      case CAN_ID_EGT: {
        CanMsgEgt msg;
        if (!unpack(message, &msg)) break;
        data->egt = decodeEgtC(&msg);
        lastEgtRx = now;
        break;
      }

      case CAN_ID_INTERCOOLER: {
        CanMsgIntercooler msg;
        if (!unpack(message, &msg)) break;
        data->intercoolerTemperature = decodeIntercoolerTempC(&msg);
        lastIntercoolerRx = now;
        break;
      }

      case CAN_ID_ATMOS: {
        CanMsgAtmos msg;
        if (!unpack(message, &msg)) break;
        data->atmosPressure = decodeAtmosPressurePa(&msg);
        data->atmosTemperature = decodeAtmosTempC(&msg);
        break;
      }

      case CAN_ID_HEARTBEAT: {
        CanMsgHeartbeat msg;
        if (!unpack(message, &msg)) break;
        if (heartbeatSeen) {
          if (msg.uptime_ms < lastSensorUptime) {
            sensorReboots++;
            #if DEBUG
              Serial.printf("[CAN] sensor node rebooted (%lu so far)\n", (unsigned long)sensorReboots);
            #endif
          } else {
            uint8_t gap = (uint8_t)(msg.counter - lastHeartbeatCounter - 1);
            if (gap != 0) {
              missedHeartbeats += gap;
              #if DEBUG
                Serial.printf("[CAN] missed %u heartbeat(s) (%lu so far)\n", gap, (unsigned long)missedHeartbeats);
              #endif
            }
          }
        }
        #if DEBUG
          if (!heartbeatSeen || msg.fault_flags != sensorFaultFlags) {
            Serial.printf("[CAN] sensor fault flags 0x%02X\n", msg.fault_flags);
          }
        #endif
        sensorFaultFlags = msg.fault_flags;
        lastSensorUptime = msg.uptime_ms;
        lastHeartbeatCounter = msg.counter;
        lastHeartbeatTime = now;
        heartbeatSeen = true;
        break;
      }
    }
  }

  // Blank anything that has stopped arriving. With no heartbeat at all, blank everything.
  uint32_t now = millis();
  bool connected = isSensorConnected();
  if (!connected || now - lastBoostRx > STALE_BOOST_MS) data->boostPressure = NAN;
  if (!connected || now - lastOilRx > STALE_SLOW_MS) {
    data->oilTemperature = NAN;
    data->oilPressure = NAN;
  }
  if (!connected || now - lastEgtRx > STALE_SLOW_MS) data->egt = NAN;
  if (!connected || now - lastIntercoolerRx > STALE_SLOW_MS) data->intercoolerTemperature = NAN;

  return received;
}
