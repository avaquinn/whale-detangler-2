#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>

//poll Serial for a command line; 'safe' = device is not underwater/charging, so blocking,
//calibration, and erase commands are allowed
void console_poll(bool safe);

bool console_liveMode(); //"live on": stream S lines at Ui::LIVE_PERIOD_MS for the PC viewer
bool console_logSurface(); //"logsurface on": also log samples to FRAM while at the surface

#endif
