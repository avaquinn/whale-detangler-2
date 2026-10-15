/*
  FRAM driver (SPI)
  - Implements low-level MB85RS2MTA read/write operations and command sequencing
  - Verifies the part with RDID so a missing/dead chip is reported instead of silently "working"
  - Provides reliable nonvolatile storage primitives for the logger
*/
#include <avr/wdt.h>
#include "spi_bus.h"
#include "FRAM.h"

//opcodes
static constexpr uint8_t CMD_WREN = 0x06; //write enable (the chip clears it after every WRITE)
static constexpr uint8_t CMD_WRITE = 0x02; //write memory
static constexpr uint8_t CMD_READ = 0x03; //read memory
static constexpr uint8_t CMD_RDID = 0x9F; //read device ID

//Fujitsu manufacturer ID + continuation code, then the MB85RS2MTA product ID
static const uint8_t EXPECTED_ID[4] = {0x04, 0x7F, 0x48, 0x03};

//fill is split into chunks so a long erase never holds CS low for the whole array
static constexpr uint32_t FILL_CHUNK_BYTES = 4096;

//argument and bounds checking
static bool check_range(uint32_t addr, uint32_t len) {
  if (addr >= FRAM_TOTAL_BYTES) return false;
  if (len > FRAM_TOTAL_BYTES - addr) return false;
  return true;
}

static void send_cmd_addr(uint8_t cmd, uint32_t addr) {
  spi_txrx(cmd);
  spi_txrx((uint8_t)(addr >> 16)); //address bits 23-16
  spi_txrx((uint8_t)(addr >> 8)); //address bits 15-8
  spi_txrx((uint8_t)addr); //address bits 7-0
}

static void write_enable() {
  spi_select(Pins::FRAM_CS, SPI_MODE0);
  spi_txrx(CMD_WREN);
  spi_deselect();
}

void fram_read_id(uint8_t id[4]) {
  spi_select(Pins::FRAM_CS, SPI_MODE0);
  spi_txrx(CMD_RDID);
  spi_transfer(NULL, id, 4);
  spi_deselect();
}

//configure CS idle-high and confirm the part answers with the expected ID
bool fram_init() {
  spi_config_cs(Pins::FRAM_CS);

  uint8_t id[4];
  fram_read_id(id);
  return memcmp(id, EXPECTED_ID, sizeof(id)) == 0;
}

//write 'len' bytes starting at 'addr' from 'src'
//returns false if out-of-range or invalid args
bool fram_write(uint32_t addr, const uint8_t *src, size_t len) {
  if (len == 0) return true; //no-op write is valid
  if (!src || !check_range(addr, len)) return false;

  write_enable();
  spi_select(Pins::FRAM_CS, SPI_MODE0);
  send_cmd_addr(CMD_WRITE, addr);
  spi_transfer(src, NULL, len);
  spi_deselect();
  return true;
}

//read 'len' bytes starting at 'addr' into 'dst'
//returns false if out-of-range or invalid args
bool fram_read(uint32_t addr, uint8_t *dst, size_t len) {
  if (len == 0) return true; //no-op read is valid
  if (!dst || !check_range(addr, len)) return false;

  spi_select(Pins::FRAM_CS, SPI_MODE0);
  send_cmd_addr(CMD_READ, addr);
  spi_transfer(NULL, dst, len);
  spi_deselect();
  return true;
}

//set 'len' bytes starting at 'addr' to 'value' (used to erase log regions)
bool fram_fill(uint32_t addr, uint8_t value, uint32_t len) {
  if (!check_range(addr, len)) return false;

  while (len > 0) {
    uint32_t chunk = (len > FILL_CHUNK_BYTES) ? FILL_CHUNK_BYTES : len;
    write_enable();
    spi_select(Pins::FRAM_CS, SPI_MODE0);
    send_cmd_addr(CMD_WRITE, addr);
    for (uint32_t i = 0; i < chunk; i++) {
      spi_txrx(value);
    }
    spi_deselect();
    addr += chunk;
    len -= chunk;
    wdt_reset(); //a full erase takes about a second
  }
  return true;
}
