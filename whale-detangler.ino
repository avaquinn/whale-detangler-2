#include <Arduino.h>
#include "SPI.h"
#include "ADXL.h"
#include "status.h"
#include "config.h"

enum class DemoState : uint8_t {
  NORMAL = 0,
  ANOMALY_CHARGING,
  FIRED
};

// Demo tuning constants (adjust on hardware as needed).
static constexpr uint16_t SAMPLE_PERIOD_MS = 100;           // 10 Hz print/sample
static constexpr uint16_t JERK_SPIKE_THRESHOLD = 220;       // raw counts
static constexpr uint8_t RECOVERY_SAMPLES_REQUIRED = 8;     // calm samples to return green
static constexpr uint32_t FIRE_AFTER_ANOMALY_MS = 5000UL;   // yellow -> red delay

static DemoState demo_state = DemoState::NORMAL;
static uint32_t last_sample_ms = 0;
static uint32_t anomaly_start_ms = 0;
static uint8_t calm_streak = 0;

static bool have_prev = false;
static int16_t prev_x = 0;
static int16_t prev_y = 0;
static int16_t prev_z = 0;

static const char* state_name(DemoState s) {
  switch (s) {
    case DemoState::NORMAL: return "NORMAL";
    case DemoState::ANOMALY_CHARGING: return "ANOMALY_CHARGING";
    case DemoState::FIRED: return "FIRED";
    default: return "UNKNOWN";
  }
}

static uint8_t state_code(DemoState s) {
  return (uint8_t)s;
}

static bool sample_is_erratic(int16_t x, int16_t y, int16_t z) {
  if (!have_prev) {
    return false;
  }

  // Jerk-like score: large sample-to-sample jumps indicate erratic motion.
  uint32_t dx = (uint32_t)abs((int32_t)x - (int32_t)prev_x);
  uint32_t dy = (uint32_t)abs((int32_t)y - (int32_t)prev_y);
  uint32_t dz = (uint32_t)abs((int32_t)z - (int32_t)prev_z);
  uint32_t jerk = dx + dy + dz;
  return jerk >= JERK_SPIKE_THRESHOLD;
}

static void update_leds_for_state(uint32_t now_ms) {
  if (demo_state == DemoState::FIRED) {
    // Latch red forever after fire.
    status_setOverride(PersistentStatus::LOW_BATTERY_NONOP, now_ms);
    return;
  }

  // Clear any persistent override while in active demo states.
  status_setOverride(PersistentStatus::NONE, now_ms);

  if (demo_state == DemoState::NORMAL) {
    // Reuse battery-display green pattern as a stable "normal" indicator.
    status_showBatteryDisplay(BatteryDisplay::MORE_THAN_30_DAYS, now_ms);
  } else {
    // Reuse battery-display yellow pattern as "charging/anomaly" indicator.
    status_showBatteryDisplay(BatteryDisplay::LESS_THAN_30_DAYS, now_ms);
  }
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(200);
#endif

  status_init();
  spi_init();

  bool ok = adxl_init();
  if (!ok) {
#if DEBUG_SERIAL
    Serial.println("ADXL init failed. Latching RED.");
#endif
    demo_state = DemoState::FIRED;
  }

  update_leds_for_state(millis());

#if DEBUG_SERIAL
  Serial.println("Whale-detangler alpha demo started.");
  Serial.println("State machine: GREEN(normal) -> YELLOW(anomaly) -> RED(fired latch).");
#endif
}

void loop() {
  uint32_t now_ms = millis();

  update_leds_for_state(now_ms);
  status_tick(now_ms);

  if (demo_state == DemoState::FIRED) {
    return;
  }

  if ((uint32_t)(now_ms - last_sample_ms) < SAMPLE_PERIOD_MS) {
    return;
  }
  last_sample_ms = now_ms;

  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  if (!adxl_read_xyz(x, y, z)) {
#if DEBUG_SERIAL
    Serial.println("ADXL read failed; keeping previous state.");
#endif
    return;
  }

  bool erratic = sample_is_erratic(x, y, z);

  DemoState prev_state = demo_state;
  if (demo_state == DemoState::NORMAL) {
    if (erratic) {
      demo_state = DemoState::ANOMALY_CHARGING;
      anomaly_start_ms = now_ms;
      calm_streak = 0;
    }
  } else if (demo_state == DemoState::ANOMALY_CHARGING) {
    if (erratic) {
      calm_streak = 0;
    } else {
      calm_streak++;
      if (calm_streak >= RECOVERY_SAMPLES_REQUIRED) {
        demo_state = DemoState::NORMAL;
      }
    }

    if ((uint32_t)(now_ms - anomaly_start_ms) >= FIRE_AFTER_ANOMALY_MS) {
      demo_state = DemoState::FIRED;
    }
  }

  if (demo_state != prev_state) {
#if DEBUG_SERIAL
    Serial.print("STATE -> ");
    Serial.println(state_name(demo_state));
#endif
  }

#if DEBUG_SERIAL
  Serial.print("t=");
  Serial.print(now_ms);
  Serial.print("ms, xyz=(");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print("), erratic=");
  Serial.print(erratic ? "1" : "0");
  Serial.print(", state=");
  Serial.println(state_name(demo_state));

  // Machine-readable stream for realtime plotting tools.
  // Format: PLOT,<t_ms>,<x>,<y>,<z>,<state_code>,<erratic_flag>
  Serial.print("PLOT,");
  Serial.print(now_ms);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");
  Serial.print(state_code(demo_state));
  Serial.print(",");
  Serial.println(erratic ? 1 : 0);
#endif

  prev_x = x;
  prev_y = y;
  prev_z = z;
  have_prev = true;
}
