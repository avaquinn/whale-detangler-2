/*
  Minimal SD SPI writer: SD v2 / SDHC cards, single-block writes only.
  The card is raw storage, not a filesystem. Keep it dedicated to this device.
*/
#include <SPI.h>
#include "config.h"
#include "raw_sd.h"

static constexpr uint32_t INIT_HZ = 250000UL;
static constexpr uint32_t DATA_HZ = 1000000UL; //match the conservative, verified bench-test rate
static bool g_ready = false;
#if RAW_SD_DIAGNOSTICS
static uint8_t g_last_error = 0;
static uint8_t g_last_response = 0xFF;

uint8_t raw_sd_last_error() {
  return g_last_error;
}

uint8_t raw_sd_last_response() {
  return g_last_response;
}

static void set_error(uint8_t code) {
  g_last_error = code;
}
#else
static void set_error(uint8_t) {}
#endif

static void select_card(uint32_t hz) {
  SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::SD_CS, LOW);
}

static void deselect_card() {
  digitalWrite(Pins::SD_CS, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();
}

//A card can hold MISO low while completing the preceding operation. Never send
//the next command until it has released the bus; otherwise it can decode the
//command stream as an invalid command (notably CMD24 after initialization).
static bool wait_not_busy(uint16_t timeout_ms) {
  const uint32_t start = millis();
  do {
    if (SPI.transfer(0xFF) == 0xFF) return true;
  } while ((uint32_t)(millis() - start) < timeout_ms);
  return false;
}

//Issues one command. The card remains selected so response bytes can be read.
static uint8_t command(uint8_t cmd, uint32_t arg, uint8_t crc, uint32_t hz) {
  select_card(hz);
  if (!wait_not_busy(250)) return 0xFF;
  SPI.transfer(0x40 | cmd);
  SPI.transfer((uint8_t)(arg >> 24));
  SPI.transfer((uint8_t)(arg >> 16));
  SPI.transfer((uint8_t)(arg >> 8));
  SPI.transfer((uint8_t)arg);
  SPI.transfer(crc);
  for (uint8_t i = 0; i < 10; ++i) {
    uint8_t r = SPI.transfer(0xFF);
    if ((r & 0x80) == 0) return r;
  }
  return 0xFF;
}

static uint8_t simple_command(uint8_t cmd, uint32_t arg, uint8_t crc, uint32_t hz) {
  uint8_t r = command(cmd, arg, crc, hz);
  deselect_card();
  return r;
}

bool raw_sd_begin() {
  if (g_ready) return true;
  set_error(0);
#if RAW_SD_DIAGNOSTICS
  g_last_response = 0xFF;
#endif

  digitalWrite(Pins::SD_CS, HIGH);
  pinMode(Pins::SD_CS, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(INIT_HZ, MSBFIRST, SPI_MODE0));
  for (uint8_t i = 0; i < 10; ++i) SPI.transfer(0xFF); //>= 74 clocks with CS high
  SPI.endTransaction();

  bool idle = false;
  for (uint8_t i = 0; i < 20; ++i) {
    if (simple_command(0, 0, 0x95, INIT_HZ) == 0x01) {
      idle = true;
      break;
    }
    delay(2);
  }
  if (!idle) {
    set_error(1);
    return false;
  }

  //Require an SD v2 card. The tested 32 GB SDHC card is in this class.
  uint8_t r = command(8, 0x1AA, 0x87, INIT_HZ);
  uint8_t r7[4];
  for (uint8_t i = 0; i < sizeof(r7); ++i) r7[i] = SPI.transfer(0xFF);
  deselect_card();
  if (r != 0x01 || r7[2] != 0x01 || r7[3] != 0xAA) {
    set_error(2);
    return false;
  }

  bool ready = false;
  for (uint16_t i = 0; i < 500; ++i) {
    if (simple_command(55, 0, 0x01, INIT_HZ) > 0x01) {
      set_error(3);
      return false;
    }
    if (simple_command(41, 0x40000000UL, 0x01, INIT_HZ) == 0x00) {
      ready = true;
      break;
    }
    delay(2);
  }
  if (!ready) {
    set_error(3);
    return false;
  }

  r = command(58, 0, 0x01, INIT_HZ);
  uint8_t ocr0 = SPI.transfer(0xFF);
  for (uint8_t i = 0; i < 3; ++i) SPI.transfer(0xFF);
  deselect_card();
  if (r != 0x00 || (ocr0 & 0x40) == 0) {
    set_error(4);
    return false; //only block-addressed SDHC/SDXC
  }

  g_ready = true;
  return true;
}

bool raw_sd_write_sector(uint32_t sector, const uint8_t *data, uint16_t len) {
  if (!g_ready || !data || len == 0 || len > 512) return false;
  const uint8_t r1 = command(24, sector, 0x01, DATA_HZ);
  if (r1 != 0x00) {
    deselect_card();
#if RAW_SD_DIAGNOSTICS
    g_last_response = r1;
#endif
    set_error(5);
    return false;
  }
  SPI.transfer(0xFF);
  SPI.transfer(0xFE); //single-block data token
  for (uint16_t i = 0; i < len; ++i) SPI.transfer(data[i]);
  for (uint16_t i = len; i < 512; ++i) SPI.transfer(0xFF);
  SPI.transfer(0xFF); //CRC ignored in SPI mode
  SPI.transfer(0xFF);
  const uint8_t response = SPI.transfer(0xFF);
  if ((response & 0x1F) != 0x05) {
    deselect_card();
    set_error(6);
    return false;
  }
  if (!wait_not_busy(250)) {
    deselect_card();
    set_error(7);
    return false;
  }
  deselect_card();
  return true;
}
