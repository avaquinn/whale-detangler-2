/*
  Per-cycle summary (requirement: log one record per set -> soak -> haul cycle)
  - Tracks bottom depth, soak depth range and duration, and descent/ascent acceleration profiles
  - The logger assigns cycle_idx when the summary is written
*/
#include "cycle.h"
#include "ADXL.h"

static bool g_active = false;
static CycleSummary g_sum;
static float g_drop_sq = 0; //sum of squared dynamic mg (float: a long descent overflows uint32)
static float g_ret_sq = 0;
static uint32_t g_last_t = 0;

void cycle_init() {
  g_active = false;
}

static void profile_add(AccelProfile &p, float &sum_sq, const SensorSnapshot &snap, uint32_t dt) {
  p.duration_ms += dt;
  if (!(snap.flags & SNAP_VALID_ACCEL)) return;
  uint16_t dyn = adxl_dynamic_mg(snap.ax, snap.ay, snap.az);
  if (dyn > p.peak_mg) p.peak_mg = dyn;
  sum_sq += (float)dyn * dyn;
  if (p.sample_count < 0xFFFF) p.sample_count++;
}

static uint16_t rms(float sum_sq, uint16_t n) {
  return n ? (uint16_t)(sqrtf(sum_sq / n) + 0.5f) : 0;
}

bool cycle_update(const SensorSnapshot &snap, DeployPhase phase, CycleSummary &done) {
  const bool have_depth = (snap.flags & SNAP_VALID_DEPTH) != 0;

  if (!g_active) {
    if (phase == DeployPhase::SURFACE) return false;
    g_active = true;
    g_sum = {};
    g_sum.start_t_ms = snap.t_ms;
    g_sum.soak_min_depth_cm = INT16_MAX;
    g_sum.soak_max_depth_cm = INT16_MIN;
    g_drop_sq = 0;
    g_ret_sq = 0;
    g_last_t = snap.t_ms;
  }

  uint32_t dt = snap.t_ms - g_last_t;
  g_last_t = snap.t_ms;

  if (have_depth && snap.depth_cm > g_sum.bottom_depth_cm) {
    g_sum.bottom_depth_cm = snap.depth_cm;
  }

  switch (phase) {
    case DeployPhase::DESCENT:
      profile_add(g_sum.drop, g_drop_sq, snap, dt);
      break;
    case DeployPhase::SOAK:
      g_sum.soak_duration_ms += dt;
      if (have_depth) {
        if (snap.depth_cm < g_sum.soak_min_depth_cm) g_sum.soak_min_depth_cm = snap.depth_cm;
        if (snap.depth_cm > g_sum.soak_max_depth_cm) g_sum.soak_max_depth_cm = snap.depth_cm;
      }
      break;
    case DeployPhase::ASCENT:
      profile_add(g_sum.retrieval, g_ret_sq, snap, dt);
      break;
    case DeployPhase::SURFACE:
      g_sum.end_t_ms = snap.t_ms;
      g_sum.drop.rms_mg = rms(g_drop_sq, g_sum.drop.sample_count);
      g_sum.retrieval.rms_mg = rms(g_ret_sq, g_sum.retrieval.sample_count);
      if (g_sum.soak_min_depth_cm > g_sum.soak_max_depth_cm) { //never soaked
        g_sum.soak_min_depth_cm = 0;
        g_sum.soak_max_depth_cm = 0;
      }
      done = g_sum;
      g_active = false;
      return true;
  }
  return false;
}
