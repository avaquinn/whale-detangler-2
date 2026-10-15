/*
  Entanglement detection (anomalous pot motion)
  - A normal pot has a predictable profile: fast descent, long static soak (small tide/swell ripple),
    then a fast, monotonic haul to the surface. An entangled whale drags the gear in ways that break
    that profile. The detector tracks the phase and flags departures from it:
      crush depth (deeper than any legal set), soak excursion (left the resting depth without a clean
      haul), reversal (wrong direction during descent/ascent), towed (sustained motion without upward
      travel), ghost-gear timer (optional)
  - Depth is decimated to 1 Hz and smoothed over Detector::SMOOTH_S seconds so surface-wave pressure
    (5-20 s periods) is not mistaken for vertical motion; rate-based checks wait for a full window
  - Anomalies are STATES, not one-shot events, so the two-stage charge -> confirm logic works:
      stage 1: anomaly for PRECHARGE_CONFIRM_COUNT samples (and deep enough) -> START_CHARGE
      stage 2: ABORT after ABORT_CONFIRM_COUNT normal samples; FIRE only if the anomaly is still
               present FIRE_CONFIRM_MS after charging started and the pot is deeper than MIN_FIRE_DEPTH
  - Samples without a valid calibrated depth are never used as evidence
*/
#include "detector.h"

static constexpr uint8_t HIST_LEN = Detector::SMOOTH_S + Detector::RATE_LAG_S;
static constexpr uint16_t HIST_STEP_MS = 1000;
static constexpr uint8_t SHORT_MEAN_S = 4; //for surface/disarm decisions

//1 Hz depth history (ring, newest at g_head)
static int16_t g_hist[HIST_LEN];
static uint8_t g_count = 0;
static uint8_t g_head = 0;
static uint32_t g_next_push_ms = 0;

static DeployPhase g_phase = DeployPhase::SURFACE;
static uint32_t g_phase_start_ms = 0;
static uint32_t g_deploy_start_ms = 0;

static bool g_rate_valid = false;
static int16_t g_rate_cm_s = 0; //positive = getting deeper
static int16_t g_smooth_cm = 0;

static bool g_still = false;
static uint32_t g_still_since_ms = 0;
static int16_t g_soak_baseline_cm = 0;

static uint32_t g_motion_ms = 0; //accumulated motion without upward travel
static uint32_t g_last_motion_ms = 0;
static uint32_t g_last_eval_ms = 0;

static uint8_t g_anomaly_streak = 0;
static uint8_t g_calm_streak = 0;
static bool g_charge_window_seen = false;
static uint32_t g_charge_window_start_ms = 0;

static void hist_reset() {
  g_count = 0;
  g_head = 0;
  g_rate_valid = false;
  g_rate_cm_s = 0;
}

//push one point per second; after a gap, repeat the current depth so the ring stays time-aligned
static void hist_update(uint32_t t_ms, int16_t depth_cm) {
  if (g_count == 0) {
    g_next_push_ms = t_ms;
  }
  uint8_t pushes = 0;
  while ((int32_t)(t_ms - g_next_push_ms) >= 0 && pushes < HIST_LEN) {
    g_head = (uint8_t)((g_head + 1) % HIST_LEN);
    g_hist[g_head] = depth_cm;
    if (g_count < HIST_LEN) g_count++;
    g_next_push_ms += HIST_STEP_MS;
    pushes++;
  }
  if (pushes == HIST_LEN) {
    g_next_push_ms = t_ms + HIST_STEP_MS; //long gap: resynchronise
  }
}

//mean of 'n' points, the newest of which is 'skip' points before the newest entry
static int16_t hist_mean(uint8_t n, uint8_t skip) {
  int32_t sum = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx = (uint8_t)((g_head + HIST_LEN - ((skip + i) % HIST_LEN)) % HIST_LEN);
    sum += g_hist[idx];
  }
  return (int16_t)(sum / n);
}

static void update_rate() {
  if (g_count == 0) return;
  uint8_t n = g_count < Detector::SMOOTH_S ? g_count : Detector::SMOOTH_S;
  g_smooth_cm = hist_mean(n, 0);

  g_rate_valid = (g_count >= HIST_LEN);
  if (g_rate_valid) {
    int16_t older = hist_mean(Detector::SMOOTH_S, Detector::RATE_LAG_S);
    g_rate_cm_s = (int16_t)(((int32_t)g_smooth_cm - older) / Detector::RATE_LAG_S);
  } else {
    g_rate_cm_s = 0;
  }
}

static void set_phase(DeployPhase p, uint32_t t_ms) {
  g_phase = p;
  g_phase_start_ms = t_ms;
  g_still = false;
}

static void reset_confirmation() {
  g_anomaly_streak = 0;
  g_calm_streak = 0;
  g_charge_window_seen = false;
}

static void start_soak(uint32_t t_ms) {
  set_phase(DeployPhase::SOAK, t_ms);
  g_soak_baseline_cm = g_smooth_cm;
}

void detector_init() {
  hist_reset();
  set_phase(DeployPhase::SURFACE, 0);
  g_deploy_start_ms = 0;
  g_smooth_cm = 0;
  g_soak_baseline_cm = 0;
  g_motion_ms = 0;
  g_last_motion_ms = 0;
  g_last_eval_ms = 0;
  reset_confirmation();
}

DeployPhase detector_phase() {
  return g_phase;
}

int16_t detector_rate_cm_s() {
  return g_rate_cm_s;
}

//phase transitions for a normal cycle
static void update_phase(uint32_t t, int16_t depth) {
  if (g_phase == DeployPhase::SURFACE) {
    if (depth >= Depth::ARM_DEPTH_CM) { //arm on the raw reading: descents are fast
      set_phase(DeployPhase::DESCENT, t);
      g_deploy_start_ms = t;
      g_motion_ms = 0;
      reset_confirmation();
    }
    return;
  }

  //disarm on a short mean so a wave trough near the arm depth does not toggle the state
  if (g_count >= SHORT_MEAN_S && hist_mean(SHORT_MEAN_S, 0) < Depth::ARM_DEPTH_CM - Depth::ARM_HYST_CM) {
    set_phase(DeployPhase::SURFACE, t);
    hist_reset();
    reset_confirmation();
    return;
  }

  const int16_t rate = g_rate_cm_s;
  const bool still_now = g_rate_valid && rate <= Detector::STILL_RATE_CM_S && rate >= -Detector::STILL_RATE_CM_S;
  if (still_now && !g_still) g_still_since_ms = t;
  g_still = still_now;
  const uint32_t still_ms = g_still ? (uint32_t)(t - g_still_since_ms) : 0;

  switch (g_phase) {
    case DeployPhase::DESCENT:
      if (still_ms >= Detector::SOAK_SETTLE_MS || (uint32_t)(t - g_phase_start_ms) >= Detector::MAX_DESCENT_MS) {
        start_soak(t);
      }
      break;

    case DeployPhase::SOAK:
      if (g_rate_valid && rate <= -Detector::HAUL_RATE_CM_S) {
        set_phase(DeployPhase::ASCENT, t);
      } else if (g_still) {
        //follow the tide slowly while still; frozen during motion so an excursion is not absorbed
        g_soak_baseline_cm += (int16_t)((g_smooth_cm - g_soak_baseline_cm) / 16);
      }
      break;

    case DeployPhase::ASCENT:
      //hauler paused, or the pot settled back down: resting again at a new depth
      if (still_ms >= Detector::SOAK_SETTLE_MS) {
        start_soak(t);
      }
      break;

    case DeployPhase::SURFACE:
      break;
  }
}

//accumulate motion time while the pot is not being hauled up
static void update_motion(const SensorSnapshot &snap, uint32_t dt) {
  const bool rising = g_rate_valid && g_rate_cm_s <= -Detector::HAUL_RATE_CM_S;
  if (!(snap.flags & SNAP_VALID_ACCEL) || g_phase == DeployPhase::DESCENT || rising) {
    return;
  }
  if (snap.motion_mg >= Detector::MOTION_MG) {
    g_motion_ms += dt;
    g_last_motion_ms = snap.t_ms;
  } else if ((uint32_t)(snap.t_ms - g_last_motion_ms) >= Detector::CALM_RESET_MS) {
    g_motion_ms = 0;
  }
}

//which abnormal conditions hold right now
static uint16_t anomaly_reasons(uint32_t t) {
  uint16_t r = REASON_NONE;
  const int16_t rate = g_rate_cm_s;

  if (g_smooth_cm >= Depth::CRUSH_DEPTH_CM) r |= REASON_CRUSH_DEPTH;

  if (Detector::MAX_DEPLOY_HOURS > 0 &&
      (uint32_t)(t - g_deploy_start_ms) >= (uint32_t)Detector::MAX_DEPLOY_HOURS * 3600000UL) {
    r |= REASON_TIME_UNDERWATER;
  }

  if (g_motion_ms >= Detector::TOWED_MOTION_MS) r |= REASON_TOWED;

  if (!g_rate_valid) return r;

  switch (g_phase) {
    case DeployPhase::DESCENT:
      if (rate <= -Detector::REVERSAL_RATE_CM_S) r |= REASON_REVERSAL; //pulled up before reaching bottom
      break;

    case DeployPhase::SOAK: {
      int16_t dev = g_smooth_cm - g_soak_baseline_cm;
      if (dev < 0) dev = -dev;
      if (dev >= Detector::SOAK_EXCURSION_CM) r |= REASON_SOAK_EXCURSION;
      break;
    }

    case DeployPhase::ASCENT:
      if (rate >= Detector::REVERSAL_RATE_CM_S) r |= REASON_REVERSAL; //dragged back down mid-haul
      break;

    case DeployPhase::SURFACE:
      break;
  }
  return r;
}

DetectorOutput detector_evaluate(const SensorSnapshot &snap, bool in_charge_window) {
  DetectorOutput out = {DetectorAction::NO_ACTION, REASON_NONE};
  const uint32_t t = snap.t_ms;
  const uint32_t dt = g_last_eval_ms ? (uint32_t)(t - g_last_eval_ms) : 0;
  g_last_eval_ms = t;

  //no calibrated depth: never count it as evidence; fail safe if charging
  if (!(snap.flags & SNAP_VALID_DEPTH)) {
    if (in_charge_window) {
      out.action = DetectorAction::ABORT_CHARGE;
      out.reasons = REASON_SENSOR_FAULT;
      reset_confirmation();
    } else {
      g_anomaly_streak = 0;
    }
    return out;
  }

  const int16_t depth = snap.depth_cm;
  if (g_phase != DeployPhase::SURFACE || depth >= Depth::ARM_DEPTH_CM) {
    hist_update(t, depth);
    update_rate();
  }
  update_phase(t, depth);

  if (g_phase == DeployPhase::SURFACE) {
    if (in_charge_window) {
      out.action = DetectorAction::ABORT_CHARGE;
      out.reasons = REASON_SHALLOW;
    }
    return out;
  }

  update_motion(snap, dt);

  const uint16_t reasons = anomaly_reasons(t);
  const bool anomaly = reasons != REASON_NONE;
  const bool deep_enough = depth >= Depth::MIN_FIRE_DEPTH_CM;

  if (!in_charge_window) {
    //stage 1: confirm the anomaly before asking pyro to charge
    g_charge_window_seen = false;
    g_calm_streak = 0;
    g_anomaly_streak = anomaly ? (uint8_t)min(g_anomaly_streak + 1, 255) : 0;

    if (g_anomaly_streak >= Detector::PRECHARGE_CONFIRM_COUNT && deep_enough) {
      out.action = DetectorAction::START_CHARGE;
      out.reasons = reasons;
      g_anomaly_streak = 0;
    }
    return out;
  }

  //stage 2: charging; confirm or abort
  if (!g_charge_window_seen) {
    g_charge_window_seen = true;
    g_charge_window_start_ms = t;
    g_calm_streak = 0;
  }
  g_calm_streak = anomaly ? 0 : (uint8_t)min(g_calm_streak + 1, 255);

  if (g_calm_streak >= Detector::ABORT_CONFIRM_COUNT) {
    out.action = DetectorAction::ABORT_CHARGE;
    reset_confirmation();
  } else if ((uint32_t)(t - g_charge_window_start_ms) >= Detector::FIRE_CONFIRM_MS) {
    if (anomaly && deep_enough) {
      out.action = DetectorAction::FIRE_CUT;
      out.reasons = reasons;
    } else {
      //never fire on missing evidence: the anomaly faded or the pot is too shallow
      out.action = DetectorAction::ABORT_CHARGE;
      out.reasons = deep_enough ? reasons : (uint16_t)(reasons | REASON_SHALLOW);
    }
    reset_confirmation();
  } else {
    out.action = DetectorAction::KEEP_CHARGING;
    out.reasons = reasons;
  }
  return out;
}
