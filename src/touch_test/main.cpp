//
// CST826 Touch Controller Test for Qualia ESP32-S3
//
// Scans I2C bus, then continuously polls for touch data.
// All output goes to Serial (115200 baud).
//

#include <Arduino.h>
#include <Wire.h>

// Known CST8xx addresses to try
static const uint8_t CANDIDATE_ADDRS[] = {0x15, 0x5A, 0x1A, 0x38, 0x48};
static const int NUM_CANDIDATES = sizeof(CANDIDATE_ADDRS) / sizeof(CANDIDATE_ADDRS[0]);

// CST826 registers
#define CST826_REG_TOUCHES    0x02
#define CST826_REG_TOUCH_DATA 0x03
#define CST826_REG_CHIPTYPE   0xAA
#define CST826_REG_FIRMVERS   0xA6
#define CST826_REG_PROJID     0xA9

static uint8_t touch_addr = 0;

void i2c_scan() {
  Serial.println("\n=== I2C Bus Scan ===");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  No devices found!");
  } else {
    Serial.printf("  %d device(s) found\n", found);
  }
  Serial.println();
}

uint8_t read_reg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void try_read_chip_info(uint8_t addr) {
  Serial.printf("  Reading chip info from 0x%02X...\n", addr);

  Wire.beginTransmission(addr);
  Wire.write(CST826_REG_CHIPTYPE);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
      uint8_t chip_id = Wire.read();
      Serial.printf("    Chip ID: 0x%02X", chip_id);
      if (chip_id == 0x11) Serial.print(" (CST826)");
      else if (chip_id == 0xB4) Serial.print(" (CST816S)");
      else if (chip_id == 0xB5) Serial.print(" (CST816T)");
      else if (chip_id == 0xB7) Serial.print(" (CST820)");
      Serial.println();
    }
  }

  uint8_t fw = read_reg(addr, CST826_REG_FIRMVERS);
  Serial.printf("    Firmware version: %d\n", fw);

  uint8_t proj = read_reg(addr, CST826_REG_PROJID);
  Serial.printf("    Project ID: %d\n", proj);
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial monitor

  Serial.println("========================================");
  Serial.println("  CST826 Touch Controller Test");
  Serial.println("  Board: Qualia ESP32-S3 RGB666");
  Serial.println("========================================");

  // Init I2C with Qualia default pins
  Wire.begin();
  Wire.setClock(400000);

  Serial.printf("I2C SDA=%d SCL=%d\n", SDA, SCL);

  // Full I2C scan
  i2c_scan();

  // Try candidate addresses
  Serial.println("=== Probing candidate touch addresses ===");
  for (int i = 0; i < NUM_CANDIDATES; i++) {
    uint8_t addr = CANDIDATE_ADDRS[i];
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    Serial.printf("  0x%02X: %s\n", addr, err == 0 ? "ACK (found!)" : "NACK");
    if (err == 0 && touch_addr == 0) {
      touch_addr = addr;
    }
  }

  if (touch_addr == 0) {
    Serial.println("\nNo touch controller found at any candidate address.");
    Serial.println("The CST826 may only respond when being touched.");
    Serial.println("Will poll all candidates during touch...\n");
  } else {
    Serial.printf("\nUsing touch controller at 0x%02X\n", touch_addr);
    try_read_chip_info(touch_addr);
  }

  Serial.println("\n=== Touch the screen now ===");
  Serial.println("(Polling every 50ms, will print touch events)\n");
}

void poll_address(uint8_t addr) {
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) return;

  // Device responded! If we hadn't found it before, report it
  if (touch_addr == 0) {
    touch_addr = addr;
    Serial.printf("\n*** Touch controller woke up at 0x%02X! ***\n", addr);
    try_read_chip_info(addr);
    Serial.println();
  }

  uint8_t touches = read_reg(addr, CST826_REG_TOUCHES);
  if (touches > 0 && touches <= 5) {
    // Read touch data
    Wire.beginTransmission(addr);
    Wire.write(CST826_REG_TOUCH_DATA);
    Wire.endTransmission(false);

    uint8_t buf[6];
    Wire.requestFrom(addr, (uint8_t)6);
    for (int i = 0; i < 6 && Wire.available(); i++) {
      buf[i] = Wire.read();
    }

    uint8_t event = buf[0] >> 6;
    int16_t x = ((buf[0] & 0x0F) << 8) | buf[1];
    int16_t y = ((buf[2] & 0x0F) << 8) | buf[3];
    uint8_t finger_id = buf[2] >> 4;

    const char *event_names[] = {"PRESS", "RELEASE", "TOUCHING", "NONE"};
    Serial.printf("Touch: %-8s  x=%-4d y=%-4d  id=%d  (raw: %02X %02X %02X %02X %02X %02X)\n",
                  event_names[event & 0x03], x, y, finger_id,
                  buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
  }
}

static uint32_t scan_timer = 0;

void loop() {
  if (touch_addr != 0) {
    // Poll known address
    poll_address(touch_addr);
  } else {
    // No address found yet — poll all candidates
    for (int i = 0; i < NUM_CANDIDATES; i++) {
      poll_address(CANDIDATE_ADDRS[i]);
    }

    // Periodic full I2C scan while searching
    if (millis() - scan_timer > 5000) {
      scan_timer = millis();
      Serial.println("(Waiting for touch... rescanning I2C)");
      i2c_scan();
    }
  }

  delay(50);
}
