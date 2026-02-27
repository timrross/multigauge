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

  // Queue message for transmission (non-blocking, 10ms timeout)
  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
  return (result == ESP_OK);
}

void maintainCAN() {
  // The TWAI peripheral enters BUS_OFF after 32 consecutive TX errors
  // (e.g. no ACK from receiver). In BUS_OFF, all transmission stops.
  // initiate_recovery() triggers the mandatory 128-recessive-bit sequence
  // after which the hardware automatically returns to RUNNING state.
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.println("[CAN] BUS_OFF detected — initiating recovery");
    if (twai_initiate_recovery() == ESP_OK) {
      Serial.println("[CAN] Recovery initiated, waiting for bus...");
    } else {
      Serial.println("[CAN] Recovery failed to initiate");
    }
  }
}

void sendBoostMessage(double boost_psi, bool valid) {
  CanMsgBoost msg = {0};
  encodeBoostMsg(&msg, boost_psi, valid);
  transmitMessage(CAN_ID_BOOST, &msg, sizeof(msg));
}

void sendOilMessage(double temp_c, double pressure_bar, uint8_t oil_status, bool valid) {
  CanMsgOil msg = {0};
  encodeOilMsg(&msg, temp_c, pressure_bar, oil_status, valid);
  transmitMessage(CAN_ID_OIL, &msg, sizeof(msg));
}

void sendEgtMessage(double temp_c, bool valid) {
  CanMsgEgt msg = {0};
  encodeEgtMsg(&msg, temp_c, valid);
  transmitMessage(CAN_ID_EGT, &msg, sizeof(msg));
}

void sendIntercoolerMessage(double temp_c, bool valid) {
  CanMsgIntercooler msg = {0};
  encodeIntercoolerMsg(&msg, temp_c, valid);
  transmitMessage(CAN_ID_INTERCOOLER, &msg, sizeof(msg));
}

void sendAtmosMessage(double pressure_pa, double temp_c, bool valid) {
  CanMsgAtmos msg = {0};
  encodeAtmosMsg(&msg, pressure_pa, temp_c, valid);
  transmitMessage(CAN_ID_ATMOS, &msg, sizeof(msg));
}

void sendHeartbeat() {
  CanMsgHeartbeat msg = {0};
  encodeHeartbeatMsg(&msg, heartbeatCounter++, millis());
  transmitMessage(CAN_ID_HEARTBEAT, &msg, sizeof(msg));
}
