/*
  Status LED control and patterns
  - Provides simple APIs for setting device indication states (OK / warning / fault / activity)
  - Implements non-blocking blink patterns using timestamps (no delay()-heavy logic)
*/
#include <Arduino.h>
#include "status.h"
#include "config.h"

//LED helper functions
static inline void led_write(uint8_t pin, bool on) {
  //common anode -> ON means pulling cathode LOW
  digitalWrite(pin, on ? LOW : HIGH);
}

static inline void set_leds(bool red_on, bool green_on, bool yellow_on) {
  led_write(Pins::LED_RED, red_on);
  led_write(Pins::LED_GREEN, green_on);
  led_write(Pins::LED_YELLOW, yellow_on);
}

static inline void all_off() {
  set_leds(false, false, false);
}

//Internal state
static PersistentStatus persistent_status = PersistentStatus::NONE; //persistent override state

static bool window_active = false; //whether user display window is active
static uint32_t window_end_ms = 0; //when to turn it off
static BatteryDisplay batt_state = BatteryDisplay::UNKNOWN; //battery state

static uint32_t last_tick_ms = 0; //rate-limit LED updates
static uint32_t phase_start_ms = 0; //start time of current blink phase
static bool phase_on = false; //current on/off phase state

//Timing constants
static constexpr uint16_t BLINK_SLOW_MS = 500;
static constexpr uint16_t BLINK_FAST_MS = 250;
static constexpr uint16_t STATUS_TICK_MS = 50; //debounce/minimum time between status LED updates (prevents flicker)

//reset pattern timing whenever we switch “modes”
static inline void reset_phase(uint32_t now_ms) {
  phase_start_ms = now_ms;
  phase_on = false;
}

//decide if we should toggle the phase based on an on/off duration pair.
//returns current phase state after any toggles.
static bool step_blink(uint32_t now_ms, uint16_t half_period_ms) {
  uint32_t elapsed = now_ms - phase_start_ms;

  if (elapsed >= half_period_ms) {
    phase_on = !phase_on;
    phase_start_ms = now_ms;
  }
  return phase_on;
}

//initialize LED pins and turn everything OFF
void status_init() {
  //configure LED pins as outputs
  pinMode(Pins::LED_RED, OUTPUT);
  pinMode(Pins::LED_GREEN, OUTPUT);
  pinMode(Pins::LED_YELLOW, OUTPUT);

  all_off(); //default all LEDs OFF

  //reset internal state
  persistent_status = PersistentStatus::NONE;
  window_active = false;
  window_end_ms = 0;
  batt_state = BatteryDisplay::UNKNOWN;
  last_tick_ms = 0;
  phase_start_ms = 0;
  phase_on = false;
}

//call when triple-tap is detected.
//shows the battery state for a fixed window, then turns LEDs off again unless a persistent override is active
void status_showBatteryDisplay(BatteryDisplay state, uint32_t now_ms) {
  batt_state = state; //store what we want to show

  //start/extend the user display window
  window_active = true;
  window_end_ms = now_ms + 5000; //5s

  reset_phase(now_ms); //reset timing so blink patterns start cleanly
}

//set/clear persistent override
//if set to NONE, module falls back to either user window (if active) or OFF.
void status_setOverride(PersistentStatus status, uint32_t now_ms) {
  persistent_status = status; //update override
  reset_phase(now_ms); //reset phase so new pattern starts cleanly
}

//must be called frequently from loop() (non-blocking)
void status_tick(uint32_t now_ms) {
  //rate-limit to reduce jitter and CPU work
  if ((uint32_t)(now_ms - last_tick_ms) < STATUS_TICK_MS) {
    return;
  }
  last_tick_ms = now_ms;

  //auto-expire the user window
  if (window_active && (int32_t)(now_ms - window_end_ms) >= 0) {
    window_active = false;
  }

  //persistent overrides (highest priority)
  switch (persistent_status) {
    case PersistentStatus::LOW_BATTERY_NONOP: {
      set_leds(true, false, false); //red solid - non-operating, low battery
      return; }
    case PersistentStatus::SERVICE_REQUIRED: {
      bool on = step_blink(now_ms, BLINK_FAST_MS); //red blink - service required, not operating
      set_leds(on, false, false);
      return; }
    case PersistentStatus::FIRED: {
      bool phase = step_blink(now_ms, BLINK_FAST_MS); //yellow/red blink constantly after firing until battery discharged
      set_leds(!phase, false, phase); //when phase is true => yellow on, red off; else red on, yellow off
      return; }
    case PersistentStatus::NONE:
    default:
      break; //fall through to next priority level
  }

  //user display window (triple-tap battery display)
  if (window_active) {
    switch (batt_state) {
      case BatteryDisplay::MORE_THAN_30_DAYS: {
        set_leds(false, true, false); //green solid
        return; }
      case BatteryDisplay::LESS_THAN_30_DAYS: {
        set_leds(false, false, true); //yellow solid
        return; }
      case BatteryDisplay::LESS_THAN_7_DAYS: {
        bool on = step_blink(now_ms, BLINK_SLOW_MS); //yellow blink
        set_leds(false, false, on);
        return; }
      case BatteryDisplay::LOW_BATTERY_NONOP: {
        set_leds(true, false, false); //red solid - non-operating, low battery
        return; }
      case BatteryDisplay::UNKNOWN:
      default: {
        //fallback: slow alternating green/red so user knows “status mode” works
        bool phase = step_blink(now_ms, BLINK_SLOW_MS);
        set_leds(!phase, phase, false);
        return; }
    }
  }

  all_off(); //everything off otherwise
}