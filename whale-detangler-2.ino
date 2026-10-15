/*
  Whale Detangler data logger firmware (Arduino Pro Mini 3.3V / 8 MHz, ATmega328P)
  - Each sample: accelerometer + temperature + pressure/depth (+ cached battery)
  - Detector tracks the pot's deployment phase and requests charge / abort / fire
  - Pyro enforces charge timing and a one-shot latch that is persisted in FRAM
  - FRAM journal keeps boot/event/cycle records for years; a separate ring keeps recent samples
  - Status LEDs, triple-tap battery display (at the surface), and a serial console for bench work
  - Safety: the device never arms without a valid depth calibration, and a charge timeout always
    aborts; only a detector FIRE_CUT (anomaly confirmed and deep enough) can fire
*/
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "config.h"
#include "types.h"
#include "spi_bus.h"
#include "i2c_bus.h"
#include "ADXL.h"
#include "battery.h"
#include "status.h"
#include "pressure.h"
#include "logger.h"
#include "pyro.h"
#include "detector.h"
#include "cycle.h"
#include "console.h"

//reset cause captured before the Arduino core runs; also stops a watchdog left running by a reset
uint8_t g_reset_cause __attribute__((section(".noinit")));
void capture_reset_cause() __attribute__((naked, used, section(".init3")));
void capture_reset_cause() {
  g_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

static DeviceState g_state = DeviceState::BOOT;
static DeployPhase g_last_phase = DeployPhase::SURFACE;

static BatterySnapshot g_last_batt = {};
static bool g_have_battery = false;
static uint32_t g_last_batt_read_ms = 0;
static bool g_gauge_alert_pending = false;

static uint32_t g_last_sample_ms = 0;
static uint8_t g_faulted_sensors = 0; //SensorId bits currently failing (edge-logged)
static bool g_activity_pending = false;

//triple-tap tracking: activity events are grouped into bursts; a short burst followed by quiet is a tap
static bool g_burst_active = false;
static uint32_t g_burst_start_ms = 0;
static uint32_t g_last_activity_ms = 0;
static uint8_t g_tap_count = 0;
static uint32_t g_first_tap_ms = 0;

//latest completed sample, reused by the "live" stream for the fields it does not re-read
static SensorSnapshot g_last_snap = {};
static uint32_t g_last_live_ms = 0;

//standalone recording: acceleration samples collected into a block, written when full
static RecBlock g_rec_block = {};
static uint32_t g_next_rec_ms = 0;

#if DEBUG_SERIAL
static const __FlashStringHelper *state_name(DeviceState s) {
  switch (s) {
    case DeviceState::BOOT: return F("BOOT");
    case DeviceState::SAFE_IDLE: return F("SAFE_IDLE");
    case DeviceState::MONITORING: return F("MONITORING");
    case DeviceState::CHARGING: return F("CHARGING");
    case DeviceState::FIRED: return F("FIRED");
    case DeviceState::FAULT: return F("FAULT");
    default: return F("UNKNOWN");
  }
}
#endif

static BatteryDisplay battery_display_from_soc(uint16_t soc_x100) {
  if (soc_x100 >= Ui::SOC_X100_GREEN_MIN) return BatteryDisplay::MORE_THAN_30_DAYS;
  if (soc_x100 >= Ui::SOC_X100_YELLOW_MIN) return BatteryDisplay::LESS_THAN_30_DAYS;
  if (soc_x100 >= Ui::SOC_X100_RED_MIN) return BatteryDisplay::LESS_THAN_7_DAYS;
  return BatteryDisplay::LOW_BATTERY_NONOP;
}

static void log_event(uint32_t t_ms, EventCode code, uint8_t data0, uint16_t data1) {
  (void)logger_logEvent(t_ms, code, data0, data1); //no-op if the logger is not ready
}

static void transition_state(DeviceState next, uint16_t reasons, uint32_t now_ms) {
  if (next == g_state) {
    return;
  }
  g_state = next;
  log_event(now_ms, EventCode::STATE_CHANGE, (uint8_t)next, reasons);
#if DEBUG_SERIAL
  Serial.print(F("STATE -> "));
  Serial.println(state_name(next));
#endif
}

//log sensor faults only when they start and when they clear, not on every sample
static void track_sensor(uint8_t id, bool ok, uint32_t now_ms) {
  bool was_faulted = (g_faulted_sensors & id) != 0;
  if (!ok && !was_faulted) {
    g_faulted_sensors |= id;
    log_event(now_ms, EventCode::SENSOR_FAULT, id, 0);
  } else if (ok && was_faulted) {
    g_faulted_sensors &= (uint8_t)~id;
    log_event(now_ms, EventCode::SENSOR_RECOVERED, id, 0);
  }
}

//drive everything with a physical effect to a safe level before anything else runs
static void init_safe_outputs() {
  digitalWrite(Pins::PYRO_CHG, LOW);
  pinMode(Pins::PYRO_CHG, OUTPUT);
  digitalWrite(Pins::PYRO_FIRE, LOW);
  pinMode(Pins::PYRO_FIRE, OUTPUT);
  digitalWrite(Pins::PFET_EN, Pins::PFET_OFF);
  pinMode(Pins::PFET_EN, OUTPUT);
  //every SPI chip select idle-high before the bus is clocked
  spi_config_cs(Pins::ACCEL_CS);
  spi_config_cs(Pins::FRAM_CS);
}

static void update_persistent_status_led(uint32_t now_ms) {
  if (g_state == DeviceState::FAULT) {
    status_setOverride(PersistentStatus::SERVICE_REQUIRED, now_ms);
  } else if (g_state == DeviceState::FIRED) {
    status_setOverride(PersistentStatus::FIRED, now_ms);
  } else if (logger_recMode() == RecMode::RECORDING) {
    status_setOverride(PersistentStatus::RECORDING, now_ms);
  } else if (logger_recMode() == RecMode::HOLDING) {
    status_setOverride(PersistentStatus::RECORDING_HELD, now_ms);
  } else if (!pressure_getCal().valid) {
    status_setOverride(PersistentStatus::SERVICE_REQUIRED, now_ms); //uncalibrated: cannot arm
  } else if (g_have_battery && g_last_batt.soc_x100 < Ui::SOC_X100_RED_MIN) {
    status_setOverride(PersistentStatus::LOW_BATTERY_NONOP, now_ms);
  } else {
    status_setOverride(PersistentStatus::NONE, now_ms);
  }
}

//three taps inside the window at the surface show the battery LEDs
static void register_tap(uint32_t now_ms) {
  if (g_tap_count == 0 || (uint32_t)(now_ms - g_first_tap_ms) > Ui::TAP_WINDOW_MS) {
    g_tap_count = 0;
    g_first_tap_ms = now_ms;
  }
  if (++g_tap_count >= Ui::TAP_COUNT) {
    g_tap_count = 0;
    if (g_state == DeviceState::SAFE_IDLE || g_state == DeviceState::FIRED) {
      status_showBatteryDisplay(g_have_battery ? battery_display_from_soc(g_last_batt.soc_x100)
                                               : BatteryDisplay::UNKNOWN, now_ms);
    }
  }
}

//group activity events into bursts. Picking the board up or moving it produces a long run of
//events and never counts; a tap is a burst no longer than TAP_MAX_MS followed by TAP_QUIET_MS of calm.
static void update_tap_filter(bool activity, uint32_t now_ms) {
  if (activity) {
    if (!g_burst_active) {
      g_burst_active = true;
      g_burst_start_ms = now_ms;
    }
    g_last_activity_ms = now_ms;
    return;
  }
  if (g_burst_active && (uint32_t)(now_ms - g_last_activity_ms) >= Ui::TAP_QUIET_MS) {
    g_burst_active = false;
    if ((uint32_t)(g_last_activity_ms - g_burst_start_ms) <= Ui::TAP_MAX_MS) {
      register_tap(now_ms);
    }
  }
}

static uint16_t sample_period_ms() {
  if (g_state == DeviceState::CHARGING) return Timing::CHARGE_SAMPLE_PERIOD_MS;
  switch (detector_phase()) {
    case DeployPhase::DESCENT:
    case DeployPhase::ASCENT: return Timing::TRANSIT_SAMPLE_PERIOD_MS;
    case DeployPhase::SOAK: return Timing::SOAK_SAMPLE_PERIOD_MS;
    default: return Timing::SURFACE_SAMPLE_PERIOD_MS;
  }
}

static void read_battery(uint32_t now_ms, bool force) {
  if (!force && g_have_battery && (uint32_t)(now_ms - g_last_batt_read_ms) < Timing::BATTERY_READ_PERIOD_MS) {
    return;
  }
  g_last_batt_read_ms = now_ms;
  BatterySnapshot b;
  bool ok = battery_readSnapshot(b);
  if (ok) {
    g_last_batt = b;
    g_have_battery = true;
  }
  track_sensor(SENSOR_BATTERY, ok, now_ms);
}

static void build_snapshot(SensorSnapshot &snap, uint32_t now_ms) {
  snap = {};
  snap.t_ms = now_ms;

  if (adxl_read_xyz(snap.ax, snap.ay, snap.az)) {
    snap.flags |= SNAP_VALID_ACCEL;
    snap.motion_mg = adxl_motion_mg(snap.ax, snap.ay, snap.az, now_ms);
  }
  if (adxl_read_temp(snap.temp_raw)) snap.flags |= SNAP_VALID_TEMP;
  track_sensor(SENSOR_ACCEL, snap.flags & SNAP_VALID_ACCEL, now_ms);

  logger_markStage(STAGE_BRIDGE); //breadcrumb: if a reset hits now, the next boot reports it
  PressureResult pr = pressure_read_raw(snap.pressure_raw);
  logger_markStage(STAGE_NONE);
  if (pr == PressureResult::OK) {
    snap.flags |= SNAP_VALID_PRESSURE;
    if (pressure_toDepthCm(snap.pressure_raw, snap.depth_cm)) snap.flags |= SNAP_VALID_DEPTH;
  } else if (pr == PressureResult::CLIPPED) {
    snap.flags |= SNAP_PRESSURE_CLIP;
  }
  if (pr != PressureResult::DISABLED) {
    track_sensor(SENSOR_PRESSURE, pr == PressureResult::OK, now_ms);
  }

  logger_markStage(STAGE_GAUGE);
  read_battery(now_ms, false);
  logger_markStage(STAGE_NONE);
  if (g_have_battery) {
    snap.batt = g_last_batt;
    snap.flags |= SNAP_VALID_BATTERY;
  }

  if (g_activity_pending) snap.flags |= SNAP_ACCEL_ACTIVITY;
  if (g_gauge_alert_pending) snap.flags |= SNAP_GAUGE_ALERT;
  g_activity_pending = false;
  g_gauge_alert_pending = false;
}

static void fire_sequence(uint16_t reasons, uint32_t now_ms) {
  if (pyro_fire(now_ms)) {
    (void)logger_setFired(true); //persist before anything else can reset us
    log_event(now_ms, EventCode::FIRED, 0, reasons);
    transition_state(DeviceState::FIRED, reasons, now_ms);
  } else {
    pyro_stopCharge();
    log_event(now_ms, EventCode::PYRO_FAULT, 2, reasons);
    transition_state(DeviceState::FAULT, reasons, now_ms);
  }
}

//state machine for one sample; detector decides, this module acts
static void run_state_machine(const DetectorOutput &d, bool armed, uint32_t now_ms) {
  if (!armed) {
    if (pyro_isCharging()) {
      pyro_stopCharge();
      log_event(now_ms, EventCode::CHARGE_ABORTED, 0, d.reasons | REASON_SHALLOW);
    }
    transition_state(DeviceState::SAFE_IDLE, REASON_NONE, now_ms);
    return;
  }

  if (g_state == DeviceState::SAFE_IDLE) {
    transition_state(DeviceState::MONITORING, REASON_NONE, now_ms);
  }

  //hard fail-safe: a charge that runs too long is ABORTED, never fired
  if (g_state == DeviceState::CHARGING && pyro_chargeTimedOut(now_ms)) {
    pyro_stopCharge();
    log_event(now_ms, EventCode::CHARGE_TIMEOUT, 0, REASON_TIMEOUT);
    transition_state(DeviceState::MONITORING, REASON_TIMEOUT, now_ms);
    return;
  }

  if (g_state == DeviceState::MONITORING && d.action == DetectorAction::START_CHARGE) {
    log_event(now_ms, EventCode::CHARGE_REQUESTED, 0, d.reasons);
    if (pyro_startCharge(now_ms)) {
      log_event(now_ms, EventCode::CHARGE_STARTED, 0, d.reasons);
      transition_state(DeviceState::CHARGING, d.reasons, now_ms);
    } else {
      log_event(now_ms, EventCode::PYRO_FAULT, 1, d.reasons);
    }
  } else if (g_state == DeviceState::CHARGING) {
    if (d.action == DetectorAction::ABORT_CHARGE) {
      pyro_stopCharge();
      log_event(now_ms, EventCode::CHARGE_ABORTED, 0, d.reasons);
      transition_state(DeviceState::MONITORING, d.reasons, now_ms);
    } else if (d.action == DetectorAction::FIRE_CUT) {
      fire_sequence(d.reasons, now_ms);
    }
  }
}

//machine-readable sample line for tools/viewer.py (same columns as the dump's SMP rows, minus boot/seq):
//S,t_ms,state,phase,ax_mg,ay_mg,az_mg,motion_mg,temp_raw,p_raw,depth_cm,mv,soc_x100,flags
//depth_cm is empty when there is no calibrated depth
static void print_stream_line(const SensorSnapshot &s) {
  Serial.print(F("S,"));
  Serial.print(s.t_ms);
  Serial.print(',');
  Serial.print(s.state);
  Serial.print(',');
  Serial.print(s.phase);
  Serial.print(',');
  Serial.print((int32_t)s.ax * Accel::MG_PER_LSB);
  Serial.print(',');
  Serial.print((int32_t)s.ay * Accel::MG_PER_LSB);
  Serial.print(',');
  Serial.print((int32_t)s.az * Accel::MG_PER_LSB);
  Serial.print(',');
  Serial.print(s.motion_mg);
  Serial.print(',');
  Serial.print(s.temp_raw);
  Serial.print(',');
  Serial.print(s.pressure_raw);
  Serial.print(',');
  if (s.flags & SNAP_VALID_DEPTH) Serial.print(s.depth_cm);
  Serial.print(',');
  Serial.print(s.batt.voltage_mv);
  Serial.print(',');
  Serial.print(s.batt.soc_x100);
  Serial.print(',');
  Serial.println(s.flags);
}

//standalone recording: one acceleration sample every 1/rec_hz s, packed into blocks of
//REC_BLOCK_SAMPLES. Each sample carries its own time offset, so the short gaps while a full
//sample reads the pressure sensor (~70 ms) are recorded honestly rather than smeared.
//also called while a pressure reading waits (pressure_setIdleHook), so it reads the clock itself
static void record_tick() {
  const uint32_t now_ms = millis();
  if (logger_recMode() != RecMode::RECORDING) {
    g_rec_block.n = 0; //stopped or full: drop the partial block
    g_next_rec_ms = now_ms;
    return;
  }
  if ((int32_t)(now_ms - g_next_rec_ms) < 0) {
    return;
  }
  const uint16_t period = 1000 / logger_recHz();
  g_next_rec_ms += period;
  if ((int32_t)(now_ms - g_next_rec_ms) >= 0) {
    g_next_rec_ms = now_ms + period; //fell behind (blocking read): resume the schedule, no burst
  }

  if (g_rec_block.n == 0) {
    g_rec_block.t0_ms = now_ms;
  }
  RecSample &s = g_rec_block.s[g_rec_block.n];
  s.dt_ms = (uint16_t)(now_ms - g_rec_block.t0_ms);
  if (!adxl_read_xyz(s.ax, s.ay, s.az)) {
    return;
  }
  if (++g_rec_block.n >= Logging::REC_BLOCK_SAMPLES) {
    (void)logger_appendRecBlock(g_rec_block);
    g_rec_block.n = 0;
  }
}

//"live on": fresh accelerometer reading at 25 Hz, other fields from the latest full sample
static void stream_live(uint32_t now_ms) {
  if (!console_liveMode() || (uint32_t)(now_ms - g_last_live_ms) < Ui::LIVE_PERIOD_MS) {
    return;
  }
  g_last_live_ms = now_ms;
  SensorSnapshot s = g_last_snap;
  s.t_ms = now_ms;
  s.state = (uint8_t)g_state;
  s.phase = (uint8_t)detector_phase();
  if (adxl_read_xyz(s.ax, s.ay, s.az)) {
    s.motion_mg = adxl_motion_mg(s.ax, s.ay, s.az, now_ms);
  }
  print_stream_line(s);
}

#if DEBUG_SERIAL
//one machine-readable line per sample; tools/viewer.py decodes it into plain words
//(a second human-readable line used to duplicate it, and flash is nearly full)
static void debug_print(const SensorSnapshot &snap) {
  print_stream_line(snap);
}
#endif

void setup() {
  init_safe_outputs(); //first: pyro low, bridge off, all chip selects high

  Serial.begin(SERIAL_BAUD);
  Serial.print(F("\nwhale-detangler fw "));
  Serial.print(DeviceInfo::FW_VERSION_STR);
#if BENCH_NO_PYRO
  Serial.print(F(" [BENCH_NO_PYRO]"));
#endif
  Serial.println(F(" - type 'help'"));

  spi_init();
  i2c_init();
  status_init();
  detector_init();
  cycle_init();

  uint8_t failed = 0;
  if (!adxl_init()) failed |= SENSOR_ACCEL;
  if (!pressure_init()) failed |= SENSOR_PRESSURE;
  if (!battery_init()) failed |= SENSOR_BATTERY; //non-critical: LEDs lose battery info only
  if (!logger_init(g_reset_cause)) failed |= SENSOR_STORAGE;

  PressureCal cal;
  if (logger_getCal(cal)) pressure_setCal(cal);
  pressure_setIdleHook(record_tick); //keep recording while the bridge settles
  pressure_setEnabled(logger_bridgeEnabled());
  pyro_init(logger_isFired());

  const uint32_t now = millis();
  read_battery(now, true);

  const uint8_t critical = failed & (SENSOR_ACCEL | SENSOR_PRESSURE | SENSOR_STORAGE);
  if (failed) {
    g_faulted_sensors = failed;
    log_event(now, EventCode::SENSOR_FAULT, failed, 0);
    Serial.print(F("init failures (SensorId bits): 0x"));
    Serial.println(failed, HEX);
  }

  if (critical) {
    transition_state(DeviceState::FAULT, REASON_SENSOR_FAULT, now);
  } else if (pyro_hasFired()) {
    transition_state(DeviceState::FIRED, REASON_NONE, now);
  } else {
    transition_state(DeviceState::SAFE_IDLE, REASON_NONE, now);
  }
  if (!pressure_getCal().valid) {
    Serial.println(F("WARNING: no depth calibration; the device will not arm (see 'cal')"));
  }
  if (logger_resetStage() != STAGE_NONE) {
    static const char stage_names[] PROGMEM = "?\0bridge on (pressure read)\0writing FRAM\0reading fuel gauge";
    const char *name = stage_names;
    for (uint8_t i = 0; i < logger_resetStage(); i++) name += strlen_P(name) + 1;
    Serial.print(F("NOTE: the previous boot was reset while: "));
    Serial.println((const __FlashStringHelper *)name);
    log_event(now, EventCode::RESET_DURING, logger_resetStage(), g_reset_cause);
  }
  if (!logger_bridgeEnabled()) {
    Serial.println(F("NOTE: bridge is OFF (no pressure/depth); 'bridge on' to restore"));
  }
  if (logger_recMode() == RecMode::RECORDING) {
    Serial.print(F("recording continues: "));
    logger_printRecStatus(Serial);
  } else if (logger_recMode() == RecMode::HOLDING) {
    Serial.println(F("a recording is held in FRAM (logging paused): 'dump' it, then 'rec clear YES'"));
  }

#if ENABLE_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
}

void loop() {
  wdt_reset();
  const uint32_t now_ms = millis();

  const bool safe = g_state != DeviceState::MONITORING && g_state != DeviceState::CHARGING;
  console_poll(safe);

  update_persistent_status_led(now_ms);
  status_tick(now_ms);

  BatteryAlerts alerts;
  if (battery_pollAlerts(now_ms, alerts)) {
    g_gauge_alert_pending = true;
    read_battery(now_ms, true);
    log_event(now_ms, EventCode::BATTERY_ALERT, battery_alertBits(alerts), g_last_batt.voltage_mv);
  }

  const bool activity = adxl_poll_activity();
  if (activity) g_activity_pending = true;
  update_tap_filter(activity, now_ms);

  stream_live(now_ms);
  record_tick();

  //latched states: keep LEDs, console, and alerts alive; no sensing or control
  if (g_state == DeviceState::FIRED || g_state == DeviceState::FAULT ||
      (uint32_t)(now_ms - g_last_sample_ms) < sample_period_ms()) {
    sleep_mode(); //idle (default mode): CPU stops until the next interrupt (1 ms timer tick, UART)
    return;
  }
  g_last_sample_ms = now_ms;

  SensorSnapshot snap;
  build_snapshot(snap, now_ms);

  const DetectorOutput decision = detector_evaluate(snap, g_state == DeviceState::CHARGING);
  const DeployPhase phase = detector_phase();
  if (phase != g_last_phase) {
    log_event(now_ms, EventCode::PHASE_CHANGE, (uint8_t)phase, (uint16_t)snap.depth_cm);
    g_last_phase = phase;
  }

  run_state_machine(decision, phase != DeployPhase::SURFACE, now_ms);

  snap.state = (uint8_t)g_state;
  snap.phase = (uint8_t)phase;
  const bool log_sample = phase != DeployPhase::SURFACE || console_logSurface() ||
                          logger_recMode() == RecMode::RECORDING; //recordings keep depth/battery/state too
  logger_markStage(STAGE_FRAM);
  const bool stored = !log_sample || logger_appendSample(snap);
  logger_markStage(STAGE_NONE);
  if (!stored) {
    track_sensor(SENSOR_STORAGE, false, now_ms);
  }
  g_last_snap = snap;

  CycleSummary summary;
  if (cycle_update(snap, phase, summary)) {
    (void)logger_appendCycle(summary);
  }

#if DEBUG_SERIAL
  debug_print(snap);
#endif
}
