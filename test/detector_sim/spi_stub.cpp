#include "spi_bus.h"
void spi_init() {}
void spi_config_cs(uint8_t) {}
void spi_select(uint8_t, uint8_t) {}
void spi_deselect() {}
uint8_t spi_txrx(uint8_t) { return 0; }
void spi_transfer(const uint8_t *, uint8_t *rx, size_t len) { if (rx) memset(rx, 0, len); }
