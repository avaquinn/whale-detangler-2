/*
  Whale/entanglement detection logic
  - Consumes SensorSnapshot data (and short history/features) to classify behavior
  - Outputs a Decision/Action (e.g., NO_ACTION, START_CHARGE, ABORT_CHARGE, FIRE_CUT) plus reason codes
*/
#include <Arduino.h>
#include <stdlib.h>
#include "detector.h"
#include "config.h"

// Placeholder tuning values for early bring-up.
// These can be replaced with calibrated pressure/depth and accel heuristics later.
static constexpr uint16_t PRESSURE_DELTA_TRIGGER = 24;
static constexpr uint16_t PRESSURE_DELTA_CLEAR = 8;
static constexpr uint16_t ACCEL_JERK_TRIGGER = 300;
static constexpr uint16_t ACCEL_JERK_CLEAR = 120;

// Previous-sample storage for delta-based features.
static bool g_have_prev = false;
static SensorSnapshot g_prev = {};

// Streak counters implement basic debounce/confirmation.
// - anomaly streak: avoid starting charge from a one-sample spike.
// - calm streak: avoid aborting charge from a one-sample calm blip.
static uint8_t g_anomaly_streak = 0;
static uint8_t g_calm_streak = 0;

// Charge-window tracking is local to the detector so timeout checks
// stay consistent even if caller loop timing jitters.
static bool g_charge_window_seen = false;
static uint32_t g_charge_window_start_ms = 0;

// Unsigned absolute difference helper for pressure deltas.
static inline uint16_t abs_u16_diff_u16(uint16_t a, uint16_t b) {
  return (a > b) ? (a - b) : (b - a);
}

// "Jerk" proxy = per-axis sample-to-sample change summed across X/Y/Z.
// This is intentionally simple for Phase A and can be replaced later
// with richer motion features (RMS windows, directionality, DTW, etc.).
static uint16_t accel_jerk(const SensorSnapshot &a, const SensorSnapshot &b) {
  uint32_t dx = (uint32_t)abs((int32_t)a.ax - (int32_t)b.ax);
  uint32_t dy = (uint32_t)abs((int32_t)a.ay - (int32_t)b.ay);
  uint32_t dz = (uint32_t)abs((int32_t)a.az - (int32_t)b.az);
  uint32_t jerk = dx + dy + dz;
  if (jerk > 0xFFFFUL) jerk = 0xFFFFUL;
  return (uint16_t)jerk;
}

// Reset detector internal history/counters.
void detector_init() {
  // Clear all state so a new deployment/test starts from known conditions.
  g_have_prev = false;
  g_prev = {};
  g_anomaly_streak = 0;
  g_calm_streak = 0;
  g_charge_window_seen = false;
  g_charge_window_start_ms = 0;
}

// Evaluate one snapshot and return the next requested action.
//
// `in_charge_window` tells the detector which phase the state machine is in:
// - false: normal monitoring phase (detector may request START_CHARGE)
// - true:  charging-confirmation phase (detector may ABORT_CHARGE or FIRE_CUT)
//
// This keeps detector logic phase-aware without depending on global state.
DetectorOutput detector_evaluate(const SensorSnapshot &snap, bool in_charge_window) {
  // Default return is explicitly NO_ACTION.
  // Branches below overwrite this when detector has a decision.
  DetectorOutput out = {DetectorAction::NO_ACTION, REASON_NONE};

  // Need at least one prior sample to compute pressure delta and jerk.
  if (!g_have_prev) {
    g_prev = snap;
    g_have_prev = true;
    return out;
  }

  // Feature extraction (placeholder):
  // - pressure delta: change in raw pressure ADC since previous sample
  // - jerk: change in accel since previous sample
  uint16_t p_delta = abs_u16_diff_u16(snap.pressure_adc_raw, g_prev.pressure_adc_raw);
  uint16_t jerk = accel_jerk(snap, g_prev);

  bool pressure_anomaly = p_delta >= PRESSURE_DELTA_TRIGGER;
  bool accel_anomaly = jerk >= ACCEL_JERK_TRIGGER;
  bool any_anomaly = pressure_anomaly || accel_anomaly;

  if (in_charge_window) {
    // Stage 2 (already charging):
    // confirm either recovery (ABORT) or persistent anomaly (FIRE on timeout).
    if (!g_charge_window_seen) {
      g_charge_window_seen = true;
      g_charge_window_start_ms = snap.t_ms;
      g_calm_streak = 0;
    }

    // Any anomaly resets calm streak.
    if (any_anomaly) {
      g_calm_streak = 0;
    } else {
      // Clear thresholds are lower than trigger thresholds to add hysteresis
      // and reduce rapid flip-flopping near boundaries.
      bool pressure_clear = p_delta <= PRESSURE_DELTA_CLEAR;
      bool accel_clear = jerk <= ACCEL_JERK_CLEAR;
      if (pressure_clear && accel_clear) {
        if (g_calm_streak < 255) g_calm_streak++;
      } else {
        g_calm_streak = 0;
      }
    }

    // ABORT has priority if we have enough calm confirmation.
    if (g_calm_streak >= Detector::ABORT_CONFIRM_COUNT) {
      out.action = DetectorAction::ABORT_CHARGE;
      out.reasons = REASON_NONE;
      g_charge_window_seen = false;
      g_anomaly_streak = 0;
    // If we never recovered inside charge window, request FIRE.
    } else if ((uint32_t)(snap.t_ms - g_charge_window_start_ms) >= Detector::CHARGE_CONFIRM_TIMEOUT_MS) {
      out.action = DetectorAction::FIRE_CUT;
      out.reasons = REASON_TIMEOUT;
      if (pressure_anomaly) out.reasons |= REASON_DEPTH_ANOMALY;
      if (accel_anomaly) out.reasons |= REASON_ACCEL_ANOMALY;
      g_charge_window_seen = false;
      g_anomaly_streak = 0;
    } else {
      // Continue charging while collecting more evidence.
      out.action = DetectorAction::KEEP_CHARGING;
      out.reasons = REASON_NONE;
      if (pressure_anomaly) out.reasons |= REASON_DEPTH_ANOMALY;
      if (accel_anomaly) out.reasons |= REASON_ACCEL_ANOMALY;
    }
  } else {
    // Stage 1 (monitoring):
    // look for confirmed anomaly before asking pyro to charge.
    g_charge_window_seen = false;
    g_calm_streak = 0;

    if (any_anomaly) {
      if (g_anomaly_streak < 255) g_anomaly_streak++;
    } else {
      g_anomaly_streak = 0;
    }

    // Request charge only after consecutive anomaly confirmations.
    if (g_anomaly_streak >= Detector::PRECHARGE_CONFIRM_COUNT) {
      out.action = DetectorAction::START_CHARGE;
      out.reasons = REASON_NONE;
      if (pressure_anomaly) out.reasons |= REASON_DEPTH_ANOMALY;
      if (accel_anomaly) out.reasons |= REASON_ACCEL_ANOMALY;
      g_anomaly_streak = 0;
    }
  }

  // Advance history for next evaluate() call.
  g_prev = snap;
  return out;
}
