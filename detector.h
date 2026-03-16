#ifndef DETECTOR_H
#define DETECTOR_H

#include "types.h"

void detector_init();
DetectorOutput detector_evaluate(const SensorSnapshot &snap, bool in_charge_window);

#endif