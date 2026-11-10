#include <Arduino.h>
#include <driver/twai.h>
#include "can_tx.h"
#include "can_protocol.h"
#include "sensor.h"

// Heartbeat counter
static uint8_t heartbeatCounter = 0;

bool initCAN() {
  // Configure TWAI (CAN) peripheral
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  // 500 kbit/s timing
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // No acceptance filter - accept all messages
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

static uint32_t txFailCount = 0;

// Helper to transmit a CAN message
static bool transmitMessage(uint32_t id, const void* data, uint8_t len) {
  // Zero-initialize to ensure ss/self/dlc_non_comp flag bits are not garbage.
  // Uninitialized flags can set self-reception mode (loopback), causing the
  // RX queue to fill with our own frames.
  twai_message_t message = {};
  message.identifier = id;
  message.extd = 0;  // Standard 11-bit ID
  message.rtr = 0;   // Not a remote frame
  message.data_length_code = len;
  memcpy(message.data, data, len);

  // Queue message for transmission (10ms timeout). This fails when the queue
  // is full (nothing on the bus is acknowledging) or the driver is not running.
  if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK) {
    txFailCount++;
    return false;
  }
  return true;
}

uint32_t canTxFailCount() {
  return txFailCount;
}

void maintainCAN() {
  // With no other node on the bus, frames are never acknowledged. That alone only
  // takes the controller to error-passive (it keeps retrying and the TX queue fills).
  // Real bus faults (shorted or swapped wires, wrong bitrate) take it to BUS_OFF.
  //
  // Recovery is two steps: twai_initiate_recovery() runs the 128 x 11 recessive bit
  // sequence and then leaves the driver STOPPED, so it must be started again.
  static twai_state_t lastState = TWAI_STATE_RUNNING;
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (status.state != lastState) {
    static const char* const names[] = { "STOPPED", "RUNNING", "BUS_OFF", "RECOVERING" };
    Serial.printf("[CAN] state %s (tx errors %lu, rx errors %lu, tx failed %lu)\n",
                  status.state <= TWAI_STATE_RECOVERING ? names[status.state] : "?",
                  (unsigned long)status.tx_error_counter, (unsigned long)status.rx_error_counter,
                  (unsigned long)txFailCount);
    lastState = status.state;
  }

  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

void sendBoostMessage(const Reading& boost) {
  CanMsgBoost msg = {};
  encodeBoostMsg(&msg, boost.value, boost.fault);
  transmitMessage(CAN_ID_BOOST, &msg, sizeof(msg));
}

void sendOilMessage(const Reading& temp, const Reading& pressure) {
  CanMsgOil msg = {};
  encodeOilMsg(&msg, temp.value, temp.fault, pressure.value, pressure.fault);
  transmitMessage(CAN_ID_OIL, &msg, sizeof(msg));
}

void sendEgtMessage(const Reading& egt) {
  CanMsgEgt msg = {};
  encodeEgtMsg(&msg, egt.value, egt.fault);
  transmitMessage(CAN_ID_EGT, &msg, sizeof(msg));
}

void sendIntercoolerMessage(const Reading& temp) {
  CanMsgIntercooler msg = {};
  encodeIntercoolerMsg(&msg, temp.value, temp.fault);
  transmitMessage(CAN_ID_INTERCOOLER, &msg, sizeof(msg));
}

void sendHeartbeat(uint8_t faultFlags) {
  CanMsgHeartbeat msg = {};
  encodeHeartbeatMsg(&msg, heartbeatCounter++, millis(), faultFlags);
  transmitMessage(CAN_ID_HEARTBEAT, &msg, sizeof(msg));
}
