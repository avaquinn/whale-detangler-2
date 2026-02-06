#ifndef FRAM_H
#define FRAM_H

#include "config.h"

void fram_driver(void);
void fram_write(uint32_t addr, const uint8_t *data, size_t length);
void fram_read(uint32_t addr, uint8_t *data, size_t length);

#endif