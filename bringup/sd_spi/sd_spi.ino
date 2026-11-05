/*
  SD-card SPI data-transfer bring-up for Whale Detangler hardware.

  The test creates SPITEST.BIN, writes 16 deterministic 512-byte blocks,
  closes and reopens the file, then reads and verifies every byte. The file
  is removed only after a complete pass; on failure it is intentionally left
  on the card for inspection. Existing files are never overwritten.

  Wiring (Arduino Pro Mini 3.3 V / 8 MHz -> Adafruit microSD breakout):
    3.3V -> 3V             GND -> GND
    D13  -> CLK / SCK      D12 -> DO / MISO
    D11  -> DI / MOSI      D8  -> CS

  Serial Monitor: 38400 baud. Send 'r' to run the test again.

  D11--D13 are shared with the on-board ADXL363 and FRAM. Both are held
  deselected for this standalone bench test.
*/
#include <SPI.h>
#include <SD.h>

static constexpr uint8_t PIN_SD_CS = 8;
static constexpr uint8_t PIN_ACCEL_CS = 10;
static constexpr uint8_t PIN_FRAM_CS = A0;

// Put the rest of the board in its safe, inactive state during bench testing.
static constexpr uint8_t PIN_PFET_EN = 9;  // HIGH = bridge off
static constexpr uint8_t PIN_PYRO_CHG = A1;
static constexpr uint8_t PIN_PYRO_FIRE = A2;

static const char TEST_FILENAME[] = "SPITEST.BIN";  // 8.3 compatible
static constexpr uint16_t BLOCK_SIZE = 512;
static constexpr uint8_t BLOCK_COUNT = 16;

static uint8_t block_buffer[BLOCK_SIZE];

static void run_test();

static void drive_pin(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);  // Set latch first to avoid a low glitch.
  pinMode(pin, OUTPUT);
}

static void park_shared_spi_devices() {
  // D10 must be an output while the ATmega328P is an SPI master. It is also
  // the accelerometer CS, so HIGH keeps that device off the shared MISO line.
  drive_pin(PIN_ACCEL_CS, HIGH);
  drive_pin(PIN_FRAM_CS, HIGH);
  drive_pin(PIN_SD_CS, HIGH);
}

void setup() {
  // Never allow the bench sketch to energize the bridge or pyro circuits.
  drive_pin(PIN_PFET_EN, HIGH);
  drive_pin(PIN_PYRO_CHG, LOW);
  drive_pin(PIN_PYRO_FIRE, LOW);
  park_shared_spi_devices();

  Serial.begin(38400);
  delay(100);
  Serial.println(F("\nWhale Detangler: SD SPI data-transfer test"));
  Serial.println(F("Commands: r = run test"));
  run_test();
}

static uint8_t test_pattern(uint8_t block, uint16_t offset) {
  // Varies both within and between sectors, catching byte order and addressing
  // mistakes as well as a stuck MOSI/MISO line.
  return (uint8_t)(0xA5U ^ block ^ (uint8_t)offset ^ (uint8_t)(offset >> 8));
}

static void fill_test_block(uint8_t block) {
  for (uint16_t i = 0; i < BLOCK_SIZE; ++i) {
    block_buffer[i] = test_pattern(block, i);
  }
}

static bool initialise_card() {
  park_shared_spi_devices();
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println(F("[FAIL] SD.begin(D8)"));
    Serial.println(F("Check 3.3 V/GND and CLK=D13, DO=D12, DI=D11, CS=D8."));
    return false;
  }
  return true;
}

static bool write_test_file() {
  if (SD.exists(TEST_FILENAME)) {
    Serial.println(F("[FAIL] SPITEST.BIN already exists; refusing to overwrite it."));
    return false;
  }

  File file = SD.open(TEST_FILENAME, FILE_WRITE);
  if (!file) {
    Serial.println(F("[FAIL] could not create SPITEST.BIN"));
    return false;
  }

  for (uint8_t block = 0; block < BLOCK_COUNT; ++block) {
    fill_test_block(block);
    if (file.write(block_buffer, BLOCK_SIZE) != BLOCK_SIZE) {
      Serial.print(F("[FAIL] write failed at block "));
      Serial.println(block);
      file.close();
      return false;
    }
  }

  file.close();  // Commits buffered data and FAT/directory updates.
  return true;
}

static bool verify_test_file() {
  File file = SD.open(TEST_FILENAME, FILE_READ);
  if (!file) {
    Serial.println(F("[FAIL] could not reopen SPITEST.BIN"));
    return false;
  }

  for (uint8_t block = 0; block < BLOCK_COUNT; ++block) {
    int received = file.read(block_buffer, BLOCK_SIZE);
    if (received != BLOCK_SIZE) {
      Serial.print(F("[FAIL] short read at block "));
      Serial.println(block);
      file.close();
      return false;
    }

    for (uint16_t i = 0; i < BLOCK_SIZE; ++i) {
      uint8_t expected = test_pattern(block, i);
      if (block_buffer[i] != expected) {
        Serial.print(F("[FAIL] mismatch: block="));
        Serial.print(block);
        Serial.print(F(" offset="));
        Serial.print(i);
        Serial.print(F(" expected=0x"));
        Serial.print(expected, HEX);
        Serial.print(F(" received=0x"));
        Serial.println(block_buffer[i], HEX);
        file.close();
        return false;
      }
    }
  }

  if (file.available()) {
    Serial.println(F("[FAIL] SPITEST.BIN is longer than expected."));
    file.close();
    return false;
  }

  file.close();
  return true;
}

static void run_test() {
  Serial.println(F("\nRunning 8 KiB SD SPI write/read/verify test..."));
  if (!initialise_card()) return;
  if (!write_test_file()) return;
  if (!verify_test_file()) return;

  if (!SD.remove(TEST_FILENAME)) {
    Serial.println(F("[PASS] data verified, but could not remove SPITEST.BIN."));
    return;
  }
  Serial.println(F("[PASS] 8 KiB written, read, verified, and test file removed."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') run_test();
}
