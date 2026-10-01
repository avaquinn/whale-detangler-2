#ifndef CYCLE_H
#define CYCLE_H

#include "types.h"

void cycle_init();
//feed every sample with the detector's phase; returns true (and fills 'done') when a cycle ends
bool cycle_update(const SensorSnapshot &snap, DeployPhase phase, CycleSummary &done);

#endif
