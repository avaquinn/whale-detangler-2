//host simulation of the real detector.cpp / cycle.cpp against synthetic pot profiles
//every normal-operation scenario must end NO FIRE; rerun after changing any Detector:: constant (./run.sh)
#include <cstdio>
#include <vector>
#include <functional>
#include "types.h"
#include "detector.h"
#include "cycle.h"

struct Key { double t_s; double depth_cm; };

struct Scenario {
  const char *name;
  const char *expect; //"NO FIRE" or "FIRE"
  std::vector<Key> keys;
  std::function<double(double)> overlay; //added to depth when underwater
  std::function<double(double)> motion_mg; //dynamic accel
  bool valid_depth = true;
};

static double interp(const std::vector<Key> &k, double t) {
  if (t <= k.front().t_s) return k.front().depth_cm;
  for (size_t i = 1; i < k.size(); i++) {
    if (t <= k[i].t_s) {
      double f = (t - k[i - 1].t_s) / (k[i].t_s - k[i - 1].t_s);
      return k[i - 1].depth_cm + f * (k[i].depth_cm - k[i - 1].depth_cm);
    }
  }
  return k.back().depth_cm;
}

static uint32_t rng = 12345;
static double noise(double amp) { //uniform +-amp
  rng = rng * 1103515245u + 12345u;
  return ((double)((rng >> 8) & 0xFFFF) / 65535.0 * 2 - 1) * amp;
}

static const double PI2 = 6.283185307;

static void run(const Scenario &sc) {
  detector_init();
  cycle_init();
  bool charging = false, fired = false;
  uint32_t charge_start = 0;
  int charges = 0, aborts = 0, timeouts = 0, cycles = 0;
  uint16_t charge_reasons = 0;
  double fire_t = 0, fire_depth = 0; uint16_t fire_reasons = 0;
  const double end_t = sc.keys.back().t_s;
  DeployPhase last_phase = DeployPhase::SURFACE;
  char phases[256] = {0};
  CycleSummary cs = {};

  uint32_t t_ms = 0;
  while (t_ms / 1000.0 <= end_t && !fired) {
    double t = t_ms / 1000.0;
    double base = interp(sc.keys, t);
    double d = base + (base > 100 ? sc.overlay(t) : 0) + noise(5);

    SensorSnapshot s = {};
    s.t_ms = t_ms;
    s.depth_cm = (int16_t)d;
    s.flags = SNAP_VALID_ACCEL | SNAP_VALID_PRESSURE | (sc.valid_depth ? SNAP_VALID_DEPTH : 0);
    double m = sc.motion_mg(t);
    s.ax = (int16_t)(m / 4 * 0.7); s.ay = 0; s.az = (int16_t)(250 + m / 4 * 0.7);
    s.motion_mg = (uint16_t)m; //firmware computes this against a learned gravity baseline

    DetectorOutput o = detector_evaluate(s, charging);
    DeployPhase ph = detector_phase();
    if (ph != last_phase) {
      char buf[32];
      snprintf(buf, sizeof buf, "%c@%.0fs ", "SDKA"[(int)ph], t);
      if (strlen(phases) + strlen(buf) < sizeof phases - 1) strcat(phases, buf);
      last_phase = ph;
    }

    //mirror of run_state_machine()
    if (ph == DeployPhase::SURFACE) {
      if (charging) { charging = false; aborts++; }
    } else if (charging && t_ms - charge_start >= Pyro::MAX_CHARGE_MS) {
      charging = false; timeouts++;
    } else if (!charging && o.action == DetectorAction::START_CHARGE) {
      charging = true; charge_start = t_ms; charges++; charge_reasons |= o.reasons;
    } else if (charging && o.action == DetectorAction::ABORT_CHARGE) {
      charging = false; aborts++;
    } else if (charging && o.action == DetectorAction::FIRE_CUT) {
      if (t_ms - charge_start >= Pyro::MIN_CHARGE_MS) {
        fired = true; fire_t = t; fire_depth = d; fire_reasons = o.reasons;
      } else {
        printf("  !! FIRE_CUT before MIN_CHARGE_MS\n");
      }
    }

    s.phase = (uint8_t)ph;
    if (cycle_update(s, ph, cs)) cycles++;

    uint16_t period = charging ? Timing::CHARGE_SAMPLE_PERIOD_MS
                    : (ph == DeployPhase::DESCENT || ph == DeployPhase::ASCENT) ? Timing::TRANSIT_SAMPLE_PERIOD_MS
                    : (ph == DeployPhase::SOAK) ? Timing::SOAK_SAMPLE_PERIOD_MS
                    : Timing::SURFACE_SAMPLE_PERIOD_MS;
    t_ms += period;
  }

  bool pass = (strcmp(sc.expect, "FIRE") == 0) == fired;
  printf("[%s] %s (expect %s)\n", pass ? "PASS" : "FAIL", sc.name, sc.expect);
  printf("  phases: %s\n", phases);
  printf("  charges=%d aborts=%d timeouts=%d charge_reasons=0x%03X cycles_logged=%d\n",
         charges, aborts, timeouts, charge_reasons, cycles);
  if (fired) printf("  FIRED at t=%.1fs depth=%.0fcm reasons=0x%03X\n", fire_t, fire_depth, fire_reasons);
  if (cycles) printf("  last cycle: bottom=%dcm soak=[%d,%d]cm soak_s=%u drop_ms=%u ret_ms=%u\n",
                     cs.bottom_depth_cm, cs.soak_min_depth_cm, cs.soak_max_depth_cm,
                     cs.soak_duration_ms / 1000, cs.drop.duration_ms, cs.retrieval.duration_ms);
}

int main() {
  auto calm = [](double) { return 0.0; };
  auto deep_sea = [](double t) { return 100 * sin(PI2 * t / 44640) + 25 * sin(PI2 * t / 11); };
  constexpr double H = 3600;

  std::vector<Scenario> s;

  s.push_back({"normal 300 ft set, 6 h soak, tide + attenuated swell", "NO FIRE",
    {{0,0},{30,0},{91,9144},{91+6*H,9144},{182+6*H,0},{242+6*H,0}}, deep_sea, calm});

  s.push_back({"normal shallow 20 m set, 300 cm/s drop, slow 76 cm/s haul, +-100 cm 12 s swell", "NO FIRE",
    {{0,0},{30,0},{37,2000},{37+2*H,2000},{63+2*H,0},{120+2*H,0}},
    [](double t) { return 100 * sin(PI2 * t / 12); }, calm});

  s.push_back({"haul pauses 60 s mid-water", "NO FIRE",
    {{0,0},{30,0},{91,9144},{91+H,9144},{136+H,4644},{196+H,4644},{242+H,0},{300+H,0}}, deep_sea, calm});

  s.push_back({"haul pauses 4 min mid-water", "NO FIRE",
    {{0,0},{30,0},{91,9144},{91+H,9144},{136+H,4644},{376+H,4644},{422+H,0},{480+H,0}}, deep_sea, calm});

  s.push_back({"bottom rocking: 20 s motion bursts every 60 s", "NO FIRE",
    {{0,0},{30,0},{91,9144},{91+2*H,9144},{182+2*H,0},{240+2*H,0}}, deep_sea,
    [](double t) { return fmod(t, 60) < 20 ? 250.0 : 0.0; }});

  s.push_back({"whale lifts pot off bottom then dives with it", "FIRE",
    {{0,0},{30,0},{91,9144},{91+2*H,9144},{151+2*H,5544},{211+2*H,9000},{271+2*H,6000},{400+2*H,8500}}, deep_sea, calm});

  s.push_back({"whale drags pot slowly (8 cm/s) off its soak depth with motion", "FIRE",
    {{0,0},{30,0},{91,9144},{91+2*H,9144},{191+2*H,8344},{400+2*H,8000}}, deep_sea,
    [=](double t) { return t > 91 + 2 * H ? 300.0 : 0.0; }});

  s.push_back({"whale tows pot horizontally at constant depth (motion, no depth change)", "FIRE",
    {{0,0},{30,0},{91,9144},{91+2*H,9144},{400+2*H,9144}}, deep_sea,
    [=](double t) { return t > 91 + 2 * H ? 300.0 : 0.0; }});

  s.push_back({"4 min hauler pause with gentle boat heave (40 mg)", "NO FIRE",
    {{0,0},{30,0},{91,9144},{91+H,9144},{136+H,4644},{376+H,4644},{422+H,0},{480+H,0}}, deep_sea,
    [=](double t) { return (t > 136 + H && t < 376 + H) ? 40.0 : 0.0; }});

  s.push_back({"whale lifts pot 6 m slowly (10 cm/s), no accel motion: excursion only", "FIRE",
    {{0,0},{30,0},{91,9144},{91+2*H,9144},{151+2*H,8544},{400+2*H,8544}}, deep_sea, calm});

  s.push_back({"whale at shallow 20 m set with +-100 cm swell: lift and dive", "FIRE",
    {{0,0},{30,0},{37,2000},{37+H,2000},{77+H,1000},{117+H,1900},{157+H,1000},{300+H,1800}},
    [](double t) { return 100 * sin(PI2 * t / 12); }, calm});

  s.push_back({"whale drags pot below crush depth", "FIRE",
    {{0,0},{30,0},{91,9144},{91+H,9144},{151+H,12500},{300+H,12500}}, deep_sea, calm});

  s.push_back({"erratic motion but only 26 ft deep (below MIN_FIRE_DEPTH)", "NO FIRE",
    {{0,0},{30,0},{35,800},{35+H,800},{65+H,500},{95+H,800},{125+H,500},{155+H,800},{200+H,800}}, calm,
    [=](double t) { return t > 35 + H ? 400.0 : 0.0; }});

  Scenario nocal = s[5];
  nocal.name = "whale lift/dive but depth uncalibrated";
  nocal.expect = "NO FIRE";
  nocal.valid_depth = false;
  s.push_back(nocal);

  for (auto &sc : s) run(sc);
  return 0;
}
