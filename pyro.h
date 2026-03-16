#ifndef PYRO_H
#define PYRO_H

#include <Arduino.h>

void pyro_init();
bool pyro_startCharge(uint32_t now_ms);
void pyro_stopCharge();
bool pyro_isCharging();
bool pyro_isReadyToFire(uint32_t now_ms);
bool pyro_chargeTimedOut(uint32_t now_ms);
bool pyro_fire(uint32_t now_ms);
bool pyro_hasFired();

#endif