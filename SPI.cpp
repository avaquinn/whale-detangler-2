/*
  Hardware SPI abstraction
  - Wraps Arduino SPI usage with consistent begin/transfer helpers
  - Provides safe per-device transactions (clock/mode) and chip-select discipline
  - Used by SPI peripherals (accelerometer and FRAM)
*/
#include "SPI.h"
#include <SPI.h>

static constexpr uint32_t SPI_CLOCK_HZ = 4000000UL; //4 MHz

//guards to prevent accidental nested transactions
static bool in_transaction = false; //true if there is an active transaction
static uint8_t active_cs = 0xFF; //current active CS pin

//initialize SCL/MOSI/MISO hardware SPI
void spi_init() {
  SPI.begin();
}

//SPI CS of peripheral inactive
void spi_config_cs(uint8_t cs_pin) {
  //select correct device
  pinMode(cs_pin, OUTPUT);
  //set CS inactive (active-low)
  digitalWrite(cs_pin, HIGH);
}

//begin transaction for a given device in the specified mode
void spi_begin(uint8_t cs_pin, uint8_t spi_mode) {
  //if transaction is already active, end it defensively
  if (in_transaction) {
    spi_end(active_cs);
  }

  //start a transaction
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, spi_mode));
  //select a device
  digitalWrite(cs_pin, LOW);

  //update guards
  in_transaction = true;
  active_cs = cs_pin;
}

//end a transaction
void spi_end(uint8_t cs_pin) {
  //ignore if we aren't in a transaction
  if (!in_transaction) {
    return;
  }

  //deselect device
  digitalWrite(cs_pin, HIGH);
  //end transaction
  SPI.endTransaction();

  //update guards
  in_transaction = false;
  active_cs = 0xFF;
}

//exchange a single byte on the SPI bus
/*
Input:
  data = the byte to transmit
Output:
  the byte received simultaneously
*/
uint8_t spi_txrx(uint8_t data) {
  //send one byte and return the byte received
  return SPI.transfer(data);
}

//exchange a buffer of bytes on the SPI bus
/*
Inputs:
  tx = point to bytes to transmit; tx = NULL for read-only
  rx = pointer to buffer where received bytes will be stored; rx = NULL for write-only
  len = number of bytes to exchange
*/
void spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
  for (size_t i = 0; i < len; i++) {
    //if caller didn't provide tx buffer, send 0xFF
    uint8_t out = tx ? tx[i] : 0xFF;
    //exchange one byte
    uint8_t in = SPI.transfer(out);

    //store into rx buffer if provided; otherwise discard
    if (rx) {
      rx[i] = in;
    }
  }
}