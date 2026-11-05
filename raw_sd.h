#ifndef RAW_SD_H
#define RAW_SD_H

#include <Arduino.h>

//Minimal SPI-mode SD writer for a dedicated card. It deliberately does not
//interpret or preserve FAT/exFAT filesystems.
bool raw_sd_begin();
bool raw_sd_write_sector(uint32_t sector, const uint8_t *data, uint16_t len);

#if RAW_SD_DIAGNOSTICS
//1=CMD0, 2=CMD8, 3=ACMD41, 4=CMD58/OCR, 5=CMD24, 6=write response, 7=write busy timeout
uint8_t raw_sd_last_error();
uint8_t raw_sd_last_response(); //the CMD24 R1 response when error 5 is reported
#endif

#endif
