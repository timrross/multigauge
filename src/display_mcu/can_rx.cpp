#include <Arduino.h>
#include <driver/twai.h>
#include "can_rx.h"
#include "can_protocol.h"
#include "sensor_types.h"

// Note: No EWMA filtering here - sensor MCU already filters data

// Heartbeat tracking
static uint8_t lastHeartbeatCounter = 0;
static uint32_t lastHeartbeatTime = 0;

bool initCAN() {
  // Configure TWAI (CAN) peripheral
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

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

uint8_t getLastHeartbeatCounter() {
  return lastHeartbeatCounter;
}

uint32_t getTimeSinceLastHeartbeat() {
  return millis() - lastHeartbeatTime;
}

bool isSensorConnected() {
  // Consider connected if we've received a heartbeat and it's within timeout
  // Also return false if we've never received a heartbeat (lastHeartbeatTime == 0)
  if (lastHeartbeatTime == 0) {
    return false;
  }
  return getTimeSinceLastHeartbeat() < HEARTBEAT_TIMEOUT_MS;
}

bool processCANMessages(SensorData* data) {
  twai_message_t message;
  bool received = false;

  // Process all available messages in the RX queue
  while (twai_receive(&message, 0) == ESP_OK) {
    received = true;

    switch (message.identifier) {
      case CAN_ID_BOOST: {
        CanMsgBoost* msg = (CanMsgBoost*)message.data;
        double boost_psi = decodeBoostPsi(msg);
        if (!isnan(boost_psi)) {
          data->boostPressure = boost_psi;
        }
        break;
      }

      case CAN_ID_OIL: {
        CanMsgOil* msg = (CanMsgOil*)message.data;
        double temp = decodeOilTempC(msg);
        double pressure = decodeOilPressureBar(msg);
        if (!isnan(temp)) {
          data->oilTemperature = temp;
        }
        if (!isnan(pressure)) {
          data->oilPressure = pressure;
        }
        break;
      }

      case CAN_ID_EGT: {
        CanMsgEgt* msg = (CanMsgEgt*)message.data;
        double temp = decodeEgtC(msg);
        if (!isnan(temp)) {
          data->egt = temp;
        }
        break;
      }

      case CAN_ID_INTERCOOLER: {
        CanMsgIntercooler* msg = (CanMsgIntercooler*)message.data;
        double temp = decodeIntercoolerTempC(msg);
        if (!isnan(temp)) {
          data->intercoolerTemperature = temp;
        }
        break;
      }

      case CAN_ID_ATMOS: {
        CanMsgAtmos* msg = (CanMsgAtmos*)message.data;
        double pressure = decodeAtmosPressurePa(msg);
        double temp = decodeAtmosTempC(msg);
        if (!isnan(pressure)) {
          data->atmosPressure = pressure;
        }
        if (!isnan(temp)) {
          data->atmosTemperature = temp;
        }
        break;
      }

      case CAN_ID_HEARTBEAT: {
        CanMsgHeartbeat* msg = (CanMsgHeartbeat*)message.data;
        lastHeartbeatCounter = msg->counter;
        lastHeartbeatTime = millis();
        break;
      }
    }
  }

  return received;
}
