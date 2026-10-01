#ifndef ADXL_H
#define ADXL_H

#include <Arduino.h>

bool adxl_init();
bool adxl_read_xyz(int16_t &x, int16_t &y, int16_t &z);
bool adxl_read_temp(int16_t &raw);
bool adxl_read_adc(int16_t &raw); //signed 12-bit aux ADC: 10%..90% of VS maps to -2048..+2047
bool adxl_poll_activity(); //true if an activity event latched since the last call (acknowledges it)
uint16_t adxl_dynamic_mg(int16_t x, int16_t y, int16_t z); //| |a| - 1 g | in mg

#endif
