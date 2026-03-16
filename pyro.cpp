/*
  Pyrotechnic interface control
  - Controls PYRO_CHG and PYRO_FIRE outputs
  - Provides controlled charge and fire commands with bounded pulse timing
*/
#include <Arduino.h>
#include "pyro.h"
#include "config.h"

// Keep fire pulse short and bounded.
// Hardware-side line cutter circuitry may further shape pulse behavior.
static constexpr uint16_t FIRE_PULSE_MS = 10;

static bool g_charging = false;
static bool g_fired = false;
static uint32_t g_charge_start_ms = 0;
static uint32_t g_last_fire_ms = 0;
static bool g_has_fire_time = false;

static inline void set_charge(bool en) {
  digitalWrite(Pins::PYRO_CHG, en ? HIGH : LOW);
}

static inline void set_fire(bool en) {
  digitalWrite(Pins::PYRO_FIRE, en ? HIGH : LOW);
}

void pyro_init() {
  pinMode(Pins::PYRO_CHG, OUTPUT);
  pinMode(Pins::PYRO_FIRE, OUTPUT);

  // Default to safe/idle outputs.
  set_charge(false);
  set_fire(false);

  g_charging = false;
  g_fired = false;
  g_charge_start_ms = 0;
  g_last_fire_ms = 0;
  g_has_fire_time = false;
}

bool pyro_startCharge(uint32_t now_ms) {
  if (g_fired) {
    return false;
  }

  // Guard against accidental rapid repeat attempts.
  if (g_has_fire_time && (uint32_t)(now_ms - g_last_fire_ms) < Pyro::MIN_REFIRE_LOCKOUT_MS) {
    return false;
  }

  if (!g_charging) {
    g_charging = true;
    g_charge_start_ms = now_ms;
    set_charge(true);
  }
  return true;
}

void pyro_stopCharge() {
  g_charging = false;
  set_charge(false);
}

bool pyro_isCharging() {
  return g_charging;
}

bool pyro_isReadyToFire(uint32_t now_ms) {
  if (!g_charging || g_fired) {
    return false;
  }
  return (uint32_t)(now_ms - g_charge_start_ms) >= Pyro::MIN_CHARGE_MS;
}

bool pyro_chargeTimedOut(uint32_t now_ms) {
  if (!g_charging) {
    return false;
  }
  return (uint32_t)(now_ms - g_charge_start_ms) >= Pyro::MAX_CHARGE_MS;
}

bool pyro_fire(uint32_t now_ms) {
  if (g_fired) {
    return false;
  }
  if (g_has_fire_time && (uint32_t)(now_ms - g_last_fire_ms) < Pyro::MIN_REFIRE_LOCKOUT_MS) {
    return false;
  }
  if (!pyro_isReadyToFire(now_ms)) {
    return false;
  }

  // Stop charge before command to avoid undefined overlap states.
  pyro_stopCharge();

  // One bounded fire pulse.
  set_fire(true);
  delay(FIRE_PULSE_MS);
  set_fire(false);

  g_fired = true;          // One-shot latch for single-use cutter workflow.
  g_last_fire_ms = now_ms; // Keep lockout bookkeeping coherent.
  g_has_fire_time = true;
  return true;
}

bool pyro_hasFired() {
  return g_fired;
}
