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

//triple-tap tracking
static uint8_t g_tap_count = 0;
static uint32_t g_first_tap_ms = 0;
static uint32_t g_last_tap_ms = 0;

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
  } else if (!pressure_getCal().valid) {
    status_setOverride(PersistentStatus::SERVICE_REQUIRED, now_ms); //uncalibrated: cannot arm
  } else if (g_have_battery && g_last_batt.soc_x100 < Ui::SOC_X100_RED_MIN) {
    status_setOverride(PersistentStatus::LOW_BATTERY_NONOP, now_ms);
  } else {
    status_setOverride(PersistentStatus::NONE, now_ms);
  }
}

//count activity events; three inside the window at the surface shows the battery LEDs
static void handle_tap(uint32_t now_ms) {
  if ((uint32_t)(now_ms - g_last_tap_ms) < Ui::TAP_DEBOUNCE_MS) {
    return;
  }
  g_last_tap_ms = now_ms;
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

  if (adxl_read_xyz(snap.ax, snap.ay, snap.az)) snap.flags |= SNAP_VALID_ACCEL;
  if (adxl_read_temp(snap.temp_raw)) snap.flags |= SNAP_VALID_TEMP;
  track_sensor(SENSOR_ACCEL, snap.flags & SNAP_VALID_ACCEL, now_ms);

  PressureResult pr = pressure_read_raw(snap.pressure_raw);
  if (pr == PressureResult::OK) {
    snap.flags |= SNAP_VALID_PRESSURE;
    if (pressure_toDepthCm(snap.pressure_raw, snap.depth_cm)) snap.flags |= SNAP_VALID_DEPTH;
  } else if (pr == PressureResult::CLIPPED) {
    snap.flags |= SNAP_PRESSURE_CLIP;
  }
  track_sensor(SENSOR_PRESSURE, pr == PressureResult::OK, now_ms);

  read_battery(now_ms, false);
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

#if DEBUG_SERIAL
static void debug_print(const SensorSnapshot &snap) {
  Serial.print(F("t="));
  Serial.print(snap.t_ms);
  Serial.print(F(" state="));
  Serial.print(state_name(g_state));
  Serial.print(F(" phase="));
  Serial.print(snap.phase);
  Serial.print(F(" xyz=("));
  Serial.print(snap.ax);
  Serial.print(',');
  Serial.print(snap.ay);
  Serial.print(',');
  Serial.print(snap.az);
  Serial.print(F(") p_raw="));
  Serial.print(snap.pressure_raw);
  Serial.print(F(" depth_cm="));
  if (snap.flags & SNAP_VALID_DEPTH) Serial.print(snap.depth_cm);
  else Serial.print(F("--"));
  Serial.print(F(" rate="));
  Serial.print(detector_rate_cm_s());
  Serial.print(F(" soc="));
  Serial.print(g_have_battery ? g_last_batt.soc_x100 : 0);
  Serial.print(F(" flags=0x"));
  Serial.println(snap.flags, HEX);

  //PLOT,<t_ms>,<x>,<y>,<z>,<DeviceState>,<activity> for realtime_3d_plot.py
  Serial.print(F("PLOT,"));
  Serial.print(snap.t_ms);
  Serial.print(',');
  Serial.print(snap.ax);
  Serial.print(',');
  Serial.print(snap.ay);
  Serial.print(',');
  Serial.print(snap.az);
  Serial.print(',');
  Serial.print((uint8_t)g_state);
  Serial.print(',');
  Serial.println((snap.flags & SNAP_ACCEL_ACTIVITY) ? 1 : 0);
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

  if (adxl_poll_activity()) {
    g_activity_pending = true;
    handle_tap(now_ms);
  }

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
  if ((phase != DeployPhase::SURFACE || Logging::LOG_SURFACE_SAMPLES) && !logger_appendSample(snap)) {
    track_sensor(SENSOR_STORAGE, false, now_ms);
  }

  CycleSummary summary;
  if (cycle_update(snap, phase, summary)) {
    (void)logger_appendCycle(summary);
  }

#if DEBUG_SERIAL
  debug_print(snap);
#endif
}
