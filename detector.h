#ifndef DETECTOR_H
#define DETECTOR_H

#include "types.h"

void detector_init();
DetectorOutput detector_evaluate(const SensorSnapshot &snap, bool in_charge_window);
DeployPhase detector_phase();
int16_t detector_rate_cm_s(); //latest depth rate, positive = getting deeper

#endif
