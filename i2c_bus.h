#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>

void i2c_init();
bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len, uint8_t retries = 1);
bool i2c_write_bytes(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len, uint8_t retries = 1);

bool i2c_read_u8(uint8_t addr, uint8_t reg, uint8_t &out, uint8_t retries = 1);
bool i2c_write_u8(uint8_t addr, uint8_t reg, uint8_t data, uint8_t retries = 1);
bool i2c_read_u16be(uint8_t addr, uint8_t reg, uint16_t &out, uint8_t retries = 1);
bool i2c_write_u16be(uint8_t addr, uint8_t reg, uint16_t data, uint8_t retries = 1);

#endif
