/*
  Pyrotechnic interface control
  - Controls PYRO_CHG and PYRO_FIRE outputs
  - Enforces minimum charge time, a one-shot fire latch, and a bounded fire pulse
  - The latch is restored from FRAM at boot so a brownout/reset after firing cannot re-arm the unit
  - With BENCH_NO_PYRO the outputs are never driven high, but all bookkeeping still runs
*/
#include "pyro.h"
#include "config.h"

static bool g_charging = false;
static bool g_fired = false;
static uint32_t g_charge_start_ms = 0;

static inline void drive(uint8_t pin, bool en) {
#if BENCH_NO_PYRO
  (void)en;
  digitalWrite(pin, LOW);
#else
  digitalWrite(pin, en ? HIGH : LOW);
#endif
}

void pyro_init(bool already_fired) {
  digitalWrite(Pins::PYRO_CHG, LOW);
  pinMode(Pins::PYRO_CHG, OUTPUT);
  digitalWrite(Pins::PYRO_FIRE, LOW);
  pinMode(Pins::PYRO_FIRE, OUTPUT);

  g_charging = false;
  g_fired = already_fired;
  g_charge_start_ms = 0;
}

bool pyro_startCharge(uint32_t now_ms) {
  if (g_fired) {
    return false;
  }
  if (!g_charging) {
    g_charging = true;
    g_charge_start_ms = now_ms;
    drive(Pins::PYRO_CHG, true);
  }
  return true;
}

void pyro_stopCharge() {
  g_charging = false;
  drive(Pins::PYRO_CHG, false);
}

bool pyro_isCharging() {
  return g_charging;
}

bool pyro_isReadyToFire(uint32_t now_ms) {
  return g_charging && !g_fired && (uint32_t)(now_ms - g_charge_start_ms) >= Pyro::MIN_CHARGE_MS;
}

bool pyro_chargeTimedOut(uint32_t now_ms) {
  return g_charging && (uint32_t)(now_ms - g_charge_start_ms) >= Pyro::MAX_CHARGE_MS;
}

bool pyro_fire(uint32_t now_ms) {
  if (!pyro_isReadyToFire(now_ms)) {
    return false;
  }

  //stop charging before the fire command so both switches are never on together
  pyro_stopCharge();

  drive(Pins::PYRO_FIRE, true);
  delay(Pyro::FIRE_PULSE_MS);
  drive(Pins::PYRO_FIRE, false);

  g_fired = true; //one-shot latch; the caller persists it to FRAM
  return true;
}

bool pyro_hasFired() {
  return g_fired;
}
