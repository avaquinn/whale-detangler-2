#ifndef ADXL_H
#define ADXL_H

bool adxl_init();
bool adxl_read_xyz(int16_t &x, int16_t &y, int16_t &z);
bool adxl_read_adc(uint16_t &adc_raw);
bool adxl_read_status(uint8_t &status);

#endif