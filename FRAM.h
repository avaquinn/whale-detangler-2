#ifndef FRAM_H
#define FRAM_H

#include "config.h"

void fram_init();
bool fram_write(uint32_t addr, const uint8_t *src, size_t len);
bool fram_read(uint32_t addr, uint8_t *dst, size_t len);
uint8_t fram_read_status(); //RDSR
void fram_write_disable(); //WRDI (guard)

#endif