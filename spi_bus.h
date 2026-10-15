#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <Arduino.h>
#include <SPI.h> //SPI_MODE0..3

void spi_init();
void spi_config_cs(uint8_t cs_pin);
void spi_select(uint8_t cs_pin, uint8_t spi_mode);
void spi_deselect();
uint8_t spi_txrx(uint8_t data);
void spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len);

#endif
