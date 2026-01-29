#ifndef I2C_H
#define I2C_H

#include <wire.h>
#include "I2C.cpp"

void i2c_init();
bool i2c_write8();
bool i2c_write16();
#endif