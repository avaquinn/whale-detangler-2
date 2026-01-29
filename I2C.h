#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>

void i2c_init();
bool i2c_write(uint8_t addr, uint8_t reg, uint16_t data, uint8_t retries = 1);
bool i2c_read(uint8_t addr, uint8_t reg, uint16_t &out, uint8_t retries = 1);

#endif