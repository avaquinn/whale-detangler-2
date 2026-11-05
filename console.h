#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>

//poll Serial for a command line; 'safe' = device is not underwater/charging, so blocking,
//calibration, and erase commands are allowed
void console_poll(bool safe);

//Bench-only manual safety override. Always false in a deployment build.
bool console_takeDisarmRequest();

bool console_logSurface(); //"logsurface on": also log samples to FRAM while at the surface

#endif
