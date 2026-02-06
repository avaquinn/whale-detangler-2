/*
  FRAM driver (SPI)
  - Implements low-level FRAM read/write operations and required command sequencing
  - Provides reliable nonvolatile storage primitives for the logger
  - Used for ring-buffer recording
*/
#include <SPI.h>
#include "SPI.h"
#include "FRAM.h"

using Pins::FRAM_CS;


  // SPI mode
  constexpr uint8_t FRAM_SPI_MODE = SPI_MODE0;

// From FRAM Datasheet Opcodes (page 5)
constexpr uint8_t FRAM_CMD_WREN  = 0x06;
constexpr uint8_t FRAM_CMD_WRITE = 0x02;
constexpr uint8_t FRAM_CMD_READ  = 0x03;



// initialize the FRAM driver 
void fram_driver(void){
    // initialize the SPI 
    spi_init();

    // set the CS pin for FRAM --> A0 or D14 from config.h
    spi_config_cs(FRAM_CS);
}



// write teh data from MCU -> FRAM
void fram_write(uint32_t addr, const uint8_t *data, size_t length){

  // now that spi is ready with chip select as well get the transmission ready for delivery
  spi_begin(FRAM_CS, FRAM_SPI_MODE);


// send Write enable opcode
  spi_txrx(FRAM_CMD_WREN);
  spi_end(FRAM_CS);


// Begin actual write transaction
  spi_begin(FRAM_CS, FRAM_SPI_MODE);

// Send WRITE command opcode to tell FRAM we're writing data
  spi_txrx(FRAM_CMD_WRITE);


// Send 24-bit address (MSB first)
  spi_txrx((addr >> 16) & 0xFF);  // Address bits 23-16
  spi_txrx((addr >> 8) & 0xFF);   // Address bits 15-8
  spi_txrx(addr & 0xFF);          // Address bits 7-0


// Send the actual data bytes from MCU RAM
  spi_transfer(data, NULL, length);


 // End SPI transaction, releasing CS pin high
  spi_end(FRAM_CS);

}





// reading first from external device to put in FRAM
void fram_read(uint32_t addr, uint8_t *data, size_t length){

// Begin SPI transaction
  spi_begin(FRAM_CS, FRAM_SPI_MODE);

// Send READ command opcode to tell FRAM we're reading data
  spi_txrx(FRAM_CMD_READ);


// Send 24-bit address (MSB first)
  spi_txrx((addr >> 16) & 0xFF);  // Address bits 23-16
  spi_txrx((addr >> 8) & 0xFF);   // Address bits 15-8
  spi_txrx(addr & 0xFF);          // Address bits 7-0 


// Read data bytes from FRAM into buffer
  spi_transfer(NULL, data, length);

  // End SPI transaction
  spi_end(FRAM_CS);
}