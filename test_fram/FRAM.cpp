/*
  FRAM driver (SPI)
  - Implements low-level FRAM read/write operations and required command sequencing
  - Provides reliable nonvolatile storage primitives for the logger
  - Used for ring-buffer recording
*/
#include <Arduino.h>
#include "SPI.h"
#include "FRAM.h"

//total amount of bytes in FRAM
static constexpr uint32_t FRAM_TOTAL_BYTES = 262144UL;

//opcodes
static constexpr uint8_t CMD_WREN = 0x06; //write enable
static constexpr uint8_t CMD_WRDI = 0x04; //write disable
static constexpr uint8_t CMD_RDSR = 0x05; //read status register
static constexpr uint8_t CMD_WRSR = 0x01; //write status register
static constexpr uint8_t CMD_WRITE = 0x02; //write memory
static constexpr uint8_t CMD_READ = 0x03; //read memory

//argument and bounds checking
bool check_range(uint32_t addr, size_t len) {
  if (len == 0) return true; //no-op is allowed
  if (addr >= FRAM_TOTAL_BYTES) return false;
  if (addr + (uint32_t)len > FRAM_TOTAL_BYTES) return false;
}

//initialize FRAM interface; init SPI bus and config CS pin idle-high
void fram_init(){
    //initialize the SPI 
    spi_init();
    //confgiure FRAM CS pin as output, idle high (active-low)
    spi_config_cs(Pins::FRAM_CS);
}

//read a byte from the status register
uint8_t fram_read_status() {
  spi_begin(Pins::FRAM_CS, 0);
  spi_txrx(CMD_RDSR);
  uint8_t status = spi_txrx(0xFF); //clock out one status byte
  spi_end(Pins::FRAM_CS);
  return status;
}

//guard from writing
void fram_write_disable() {
  spi_begin(Pins::FRAM_CS, 0);
  spi_txrx(CMD_WRDI);
  spi_end(Pins::FRAM_CS);
}

//write 'len' bytes starting at 'addr' from 'src'
//returns false if out-of-range or invalid args.
bool fram_write(uint32_t addr, const uint8_t *src, size_t len) {
  //validate pointers and bounds
  if (!src && len != 0) return false;
  if (!check_range(addr, len)) return false;

  if (len == 0) return true; //no-op write is valid

  //enable writing
  spi_begin(Pins::FRAM_CS, 0);
  spi_txrx(CMD_WREN);
  spi_end(Pins::FRAM_CS);
  
  spi_begin(Pins::FRAM_CS, 0);
  spi_txrx(CMD_WRITE); //send WRITE command

  //send 24-bit address (MSB first)
  spi_txrx((uint8_t)((addr >> 16) & 0xFF)); // Address bits 23-16
  spi_txrx((uint8_t)((addr >> 8) & 0xFF)); // Address bits 15-8
  spi_txrx((uint8_t)(addr & 0xFF)); // Address bits 7-0

  //write 'len' bytes, transmit src bytes, ignore received bytes
  spi_transfer(src, NULL, len);

  spi_end(Pins::FRAM_CS);
  fram_write_disable(); //guard to disable write after operation
  return true;
}

//read 'len' bytes starting at 'addr' into 'dst'
//returns false if out-of-range or invalid args
bool fram_read(uint32_t addr, uint8_t *dst, size_t len) {
  //validate pointers and bounds
  if (!dst && len != 0) return false;
  if (!check_range(addr, len)) return false;

  if (len == 0) return true; //no-op read is valid

  spi_begin(Pins::FRAM_CS, 0);
  spi_txrx(CMD_READ); //send READ command to tell FRAM we're reading data

  //send 24-bit address (MSB first)
  spi_txrx((uint8_t)((addr >> 16) & 0xFF)); // Address bits 23-16
  spi_txrx((uint8_t)((addr >> 8) & 0xFF)); // Address bits 15-8
  spi_txrx((uint8_t)(addr & 0xFF)); // Address bits 7-0 

  //read 'len' bytes, send dummy data, store in dst
  spi_transfer(NULL, dst, len);

  spi_end(Pins::FRAM_CS);
  return true;
}