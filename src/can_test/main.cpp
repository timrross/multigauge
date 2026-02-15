// CAN transceiver self-test for ESP32-C3-Zero + SN65HVD230
// Uses TWAI_MODE_NO_ACK (self-test) to loopback messages through the transceiver.
// No second CAN node required.

#include <Arduino.h>
#include <driver/twai.h>

#define CAN_TX_PIN  7
#define CAN_RX_PIN  6

static bool driverRunning = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== CAN Transceiver Self-Test ===");
  Serial.printf("TX pin: GPIO %d\n", CAN_TX_PIN);
  Serial.printf("RX pin: GPIO %d\n\n", CAN_RX_PIN);

  // Self-test mode: transmits without requiring ACK from another node
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NO_ACK
  );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  Serial.print("Installing TWAI driver... ");
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("FAILED (0x%x)\n", err);
    return;
  }
  Serial.println("OK");

  Serial.print("Starting TWAI driver...   ");
  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("FAILED (0x%x)\n", err);
    return;
  }
  Serial.println("OK");
  driverRunning = true;

  Serial.println("\nSending self-test messages...\n");
}

void loop() {
  if (!driverRunning) {
    delay(1000);
    return;
  }

  // Build a test message
  static uint8_t counter = 0;
  twai_message_t txMsg = {};
  txMsg.identifier = 0x100;
  txMsg.data_length_code = 4;
  txMsg.data[0] = 0xDE;
  txMsg.data[1] = 0xAD;
  txMsg.data[2] = 0xBE;
  txMsg.data[3] = counter++;
  txMsg.self = 1;  // Required for self-test: message is received by self

  // Transmit
  esp_err_t txErr = twai_transmit(&txMsg, pdMS_TO_TICKS(100));
  if (txErr != ESP_OK) {
    Serial.printf("TX FAILED (0x%x) - check wiring to transceiver\n", txErr);
    delay(1000);
    return;
  }

  // Try to receive the loopback
  twai_message_t rxMsg;
  esp_err_t rxErr = twai_receive(&rxMsg, pdMS_TO_TICKS(100));
  if (rxErr == ESP_OK) {
    Serial.printf("PASS - TX id=0x%03lX [%02X %02X %02X %02X] -> "
                  "RX id=0x%03lX [%02X %02X %02X %02X]\n",
                  txMsg.identifier,
                  txMsg.data[0], txMsg.data[1], txMsg.data[2], txMsg.data[3],
                  rxMsg.identifier,
                  rxMsg.data[0], rxMsg.data[1], rxMsg.data[2], rxMsg.data[3]);
  } else {
    Serial.printf("RX FAILED (0x%x) - transceiver may not be connected\n", rxErr);
  }

  // Print driver status
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK) {
    Serial.printf("  Bus: %s | TX ok:%lu err:%lu | RX ok:%lu miss:%lu\n",
                  status.state == TWAI_STATE_RUNNING ? "RUNNING" :
                  status.state == TWAI_STATE_BUS_OFF ? "BUS_OFF" : "ERROR",
                  status.msgs_to_tx, status.tx_error_counter,
                  status.msgs_to_rx, status.rx_error_counter);
  }

  Serial.println();
  delay(1000);
}
