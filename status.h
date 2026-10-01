#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>

//battery display states
enum class BatteryDisplay : uint8_t {
  MORE_THAN_30_DAYS = 0, //green solid
  LESS_THAN_30_DAYS, //yellow solid
  LESS_THAN_7_DAYS, //yellow blink
  LOW_BATTERY_NONOP, //red solid (non-operating)
  UNKNOWN //fallback pattern
};

//persistent status overrides (higher priority than user display)
enum class PersistentStatus : uint8_t {
  NONE = 0,
  SERVICE_REQUIRED, //red blink
  FIRED, //yellow/red blink continuously
  LOW_BATTERY_NONOP //red solid
};

void status_init();
void status_showBatteryDisplay(BatteryDisplay state, uint32_t now_ms);
void status_setOverride(PersistentStatus status, uint32_t now_ms);
void status_tick(uint32_t now_ms);

#endif