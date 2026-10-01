#ifndef FRAM_H
#define FRAM_H

#include "config.h"

//MB85RS2MTA: 2 Mbit = 262,144 bytes, 24-bit addressing
constexpr uint32_t FRAM_TOTAL_BYTES = 262144UL;

bool fram_init(); //configure CS and verify the device ID; false if the chip does not answer
void fram_read_id(uint8_t id[4]); //RDID: manufacturer, continuation, product ID x2
bool fram_write(uint32_t addr, const uint8_t *src, size_t len);
bool fram_read(uint32_t addr, uint8_t *dst, size_t len);
bool fram_fill(uint32_t addr, uint8_t value, uint32_t len);

#endif
