/*
  Hardware SPI abstraction
  - Wraps Arduino SPI usage with per-device transactions (clock/mode) and chip-select discipline
  - Used by SPI peripherals (accelerometer and FRAM)
  - Every chip select must be configured idle-high (spi_config_cs) BEFORE any device is addressed,
    otherwise an unconfigured, floating CS can let a second device drive MISO
*/
#include "spi_bus.h"

static constexpr uint32_t SPI_CLOCK_HZ = 4000000UL; //4 MHz = F_CPU/2, the max at 8 MHz

static uint8_t active_cs = 0xFF; //currently selected CS pin, 0xFF when idle

void spi_init() {
  SPI.begin();
}

//configure a CS pin as an idle-high output without a low glitch:
//writing HIGH first enables the pull-up, then switching to OUTPUT keeps it high
void spi_config_cs(uint8_t cs_pin) {
  digitalWrite(cs_pin, HIGH);
  pinMode(cs_pin, OUTPUT);
}

//begin a transaction and select a device
void spi_select(uint8_t cs_pin, uint8_t spi_mode) {
  if (active_cs != 0xFF) { //nested select is a bug; end the previous one so the bus is never shared
    spi_deselect();
  }
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, spi_mode));
  digitalWrite(cs_pin, LOW);
  active_cs = cs_pin;
}

//deselect the active device and end the transaction
void spi_deselect() {
  if (active_cs == 0xFF) {
    return;
  }
  digitalWrite(active_cs, HIGH);
  SPI.endTransaction();
  active_cs = 0xFF;
}

//exchange a single byte on the SPI bus
uint8_t spi_txrx(uint8_t data) {
  return SPI.transfer(data);
}

//exchange a buffer of bytes on the SPI bus
/*
Inputs:
  tx = bytes to transmit; tx = NULL sends 0x00 (read-only)
  rx = buffer for received bytes; rx = NULL discards them (write-only)
  len = number of bytes to exchange
*/
void spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t in = SPI.transfer(tx ? tx[i] : 0x00);
    if (rx) {
      rx[i] = in;
    }
  }
}
