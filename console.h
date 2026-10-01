#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>

//poll Serial for a command line; 'safe' = device is not underwater/charging, so blocking,
//calibration, and erase commands are allowed
void console_poll(bool safe);

#endif
