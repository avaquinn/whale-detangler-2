#ifndef PRESSURE_H
#define PRESSURE_H

void pressure_init();
bool pressure_read_raw_adc(uint16_t &adc_raw);

#endif