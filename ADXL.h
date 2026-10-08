#ifndef ADXL_H
#define ADXL_H

#include <Arduino.h>

bool adxl_init();
bool adxl_read_xyz(int16_t &x, int16_t &y, int16_t &z);
bool adxl_read_temp(int16_t &raw);
bool adxl_read_adc(int16_t &raw); //signed 12-bit aux ADC: 10%..90% of VS maps to -2048..+2047
bool adxl_poll_activity(); //true if an activity event latched since the last call (acknowledges it)
uint16_t adxl_magnitude_mg(int16_t x, int16_t y, int16_t z); //|a| in mg
//| |a| - learned gravity baseline | in mg; call ONCE per sample (it updates the baseline)
uint16_t adxl_motion_mg(int16_t x, int16_t y, int16_t z, uint32_t t_ms);

#endif
