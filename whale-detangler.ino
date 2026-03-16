#include <Arduino.h>
#include <avr/interrupt.h>
#include "config.h"
#include "types.h"
#include "SPI.h"
#include "I2C.h"
#include "ADXL.h"
#include "battery.h"
#include "status.h"
#include "pressure.h"
#include "logger.h"
#include "pyro.h"
#include "detector.h"

// ---------------------------------------------------------------------------
// Integrated firmware orchestrator:
// - sensor sampling (accel + pressure + battery)
// - detector decisions (start charge / abort / fire)
// - pyro control (charge + single-shot fire)
// - FRAM logging (boot/sample/events)
// - user/status behavior (triple-tap + persistent state LEDs)
// - interrupt handling (gauge alert + accel INT1/INT2)
// ---------------------------------------------------------------------------

// Placeholder raw-pressure threshold for "armed" operation gate.
// Requirement target is ~7 psi / ~15 ft; this raw ADC value must be calibrated.
static constexpr uint16_t ARM_PRESSURE_ADC_THRESHOLD = 120;

// Battery-display placeholders; convert SOC% to a coarse "days left" class.
static constexpr uint16_t SOC_X100_GREEN_MIN = 7000;   // >= 70%
static constexpr uint16_t SOC_X100_YELLOW_MIN = 3000;  // >= 30%
static constexpr uint16_t SOC_X100_RED_MIN = 1200;     // >= 12%

// Accel interrupt flags set in ISRs and consumed in loop().
static volatile bool g_accel_int1_irq = false; // motion interrupt
static volatile bool g_accel_int2_irq = false; // triple-tap interrupt
static volatile bool g_int2_prev_level = false;

// Core runtime state.
static DeviceState g_state = DeviceState::BOOT;
static bool g_logger_ok = false;
static bool g_system_fault = false;

// Latest battery snapshot is cached for LED display and logging.
static BatterySnapshot g_last_batt = {};
static bool g_have_battery = false;

// Sample scheduling.
static uint32_t g_last_sample_ms = 0;

static const char* state_name(DeviceState s) {
  switch (s) {
    case DeviceState::BOOT: return "BOOT";
    case DeviceState::SAFE_IDLE: return "SAFE_IDLE";
    case DeviceState::MONITORING: return "MONITORING";
    case DeviceState::CHARGING: return "CHARGING";
    case DeviceState::FIRED: return "FIRED";
    case DeviceState::FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

static inline uint8_t state_code(DeviceState s) {
  return (uint8_t)s;
}

// Convert battery estimate to LED display class (placeholder mapping).
static BatteryDisplay battery_display_from_soc(uint16_t soc_x100) {
  if (soc_x100 >= SOC_X100_GREEN_MIN) return BatteryDisplay::MORE_THAN_30_DAYS;
  if (soc_x100 >= SOC_X100_YELLOW_MIN) return BatteryDisplay::LESS_THAN_30_DAYS;
  if (soc_x100 >= SOC_X100_RED_MIN) return BatteryDisplay::LESS_THAN_7_DAYS;
  return BatteryDisplay::LOW_BATTERY_NONOP;
}

// Small helper so logging calls stay guarded when storage init failed.
static inline void log_event_if_ready(uint32_t t_ms, EventCode code, uint8_t data0, uint16_t data1) {
  if (g_logger_ok) {
    (void)logger_logEvent(t_ms, code, data0, data1);
  }
}

// Transition helper centralizes state changes + event logging.
static void transition_state(DeviceState next, uint16_t reason_bits, uint32_t now_ms) {
  if (next == g_state) {
    return;
  }
  g_state = next;
  log_event_if_ready(now_ms, EventCode::STATE_CHANGE, (uint8_t)next, reason_bits);
#if DEBUG_SERIAL
  Serial.print("STATE -> ");
  Serial.println(state_name(next));
#endif
}

// Put high-risk outputs into safe defaults immediately.
static void init_safe_outputs() {
  pinMode(Pins::PYRO_CHG, OUTPUT);
  digitalWrite(Pins::PYRO_CHG, LOW);
  pinMode(Pins::PYRO_FIRE, OUTPUT);
  digitalWrite(Pins::PYRO_FIRE, LOW);
  pinMode(Pins::PFET_EN, OUTPUT);
  digitalWrite(Pins::PFET_EN, LOW);
}

// Apply persistent LED behavior based on core state/fault.
static void update_persistent_status_led(uint32_t now_ms) {
  if (g_state == DeviceState::FAULT || g_system_fault) {
    status_setOverride(PersistentStatus::SERVICE_REQUIRED, now_ms);
    return;
  }
  if (g_state == DeviceState::FIRED || pyro_hasFired()) {
    status_setOverride(PersistentStatus::FIRED, now_ms);
    return;
  }

  // Optional low-battery non-op indicator path.
  if (g_have_battery && g_last_batt.SOC < SOC_X100_RED_MIN) {
    status_setOverride(PersistentStatus::LOW_BATTERY_NONOP, now_ms);
    return;
  }

  status_setOverride(PersistentStatus::NONE, now_ms);
}

// INT1 (D3, external interrupt) is used for motion alert.
void accel_int1_isr() {
  g_accel_int1_irq = true;
}

// INT2 (D4) is handled with pin-change interrupt to support triple-tap on ATmega328P.
ISR(PCINT2_vect) {
  bool level = (PIND & _BV(PD4)) != 0;
  if (level && !g_int2_prev_level) {
    g_accel_int2_irq = true; // rising-edge detect
  }
  g_int2_prev_level = level;
}

// Configure accel interrupt plumbing:
// - INT1: external interrupt
// - INT2: pin-change interrupt
static void setup_accel_interrupts() {
  pinMode(Pins::ACCEL_INT1, INPUT);
  pinMode(Pins::ACCEL_INT2, INPUT);

  attachInterrupt(digitalPinToInterrupt(Pins::ACCEL_INT1), accel_int1_isr, RISING);

  // Enable PCINT for D4 (PD4, PCINT20).
  g_int2_prev_level = (PIND & _BV(PD4)) != 0;
  noInterrupts();
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT20);
  interrupts();
}

// Atomically fetch and clear accel interrupt flags.
static void consume_accel_irq_flags(bool &int1, bool &int2) {
  noInterrupts();
  int1 = g_accel_int1_irq;
  int2 = g_accel_int2_irq;
  g_accel_int1_irq = false;
  g_accel_int2_irq = false;
  interrupts();
}

// Initialize all subsystems; return true if critical modules are ready.
static bool init_subsystems() {
  init_safe_outputs();

  // Global bus ownership:
  // - SPI initialized once here and shared by ADXL + FRAM.
  // - I2C initialized once here and used by battery gauge.
  spi_init();
  i2c_init();

  status_init();
  pressure_init();
  pyro_init();
  detector_init();

  setup_accel_interrupts();

  bool ok = true;
  if (!adxl_init()) ok = false;
  if (!battery_init()) ok = false;

  g_logger_ok = logger_init();
  if (g_logger_ok) {
    if (!logger_writeBootRecordOnce(millis())) {
      g_logger_ok = false;
      ok = false;
    }
  } else {
    ok = false;
  }

  return ok;
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(200);
  Serial.println("Starting integrated whale-detangler firmware.");
#endif

  bool init_ok = init_subsystems();
  if (!init_ok) {
    g_system_fault = true;
    transition_state(DeviceState::FAULT, REASON_SENSOR_FAULT | REASON_STORAGE_FAULT, millis());
    log_event_if_ready(millis(), EventCode::SENSOR_FAULT, 0, 0);
  } else {
    transition_state(DeviceState::SAFE_IDLE, REASON_NONE, millis());
  }
}

void loop() {
  const uint32_t now_ms = millis();

  // Always service status LED timing, even in fault/fired modes.
  update_persistent_status_led(now_ms);
  status_tick(now_ms);

  // Handle fuel-gauge alert interrupt source (ISR lives in battery module).
  BatteryAlerts batt_alerts = {};
  bool gauge_alert = battery_pollAlerts(batt_alerts);
  if (gauge_alert) {
    log_event_if_ready(now_ms, EventCode::STATE_CHANGE, 0xB0, 0); // marker-style event for now
  }

  // Consume accel interrupt flags.
  bool accel_int1 = false;
  bool accel_int2 = false;
  consume_accel_irq_flags(accel_int1, accel_int2);

  // INT2 triple-tap: show battery status for ~5s.
  if (accel_int2) {
    BatteryDisplay disp = BatteryDisplay::UNKNOWN;
    if (g_have_battery) {
      disp = battery_display_from_soc(g_last_batt.SOC);
    }
    status_showBatteryDisplay(disp, now_ms);
  }

  // Once fired/faulted, keep system latched and skip normal control path.
  if (g_state == DeviceState::FIRED || g_state == DeviceState::FAULT) {
    return;
  }

  // Use faster sample cadence while charging; slower cadence otherwise.
  const uint16_t period_ms =
      (g_state == DeviceState::CHARGING) ? Timing::CHARGE_WINDOW_SAMPLE_PERIOD_MS
                                         : Timing::SAMPLE_PERIOD_MS;
  if ((uint32_t)(now_ms - g_last_sample_ms) < period_ms) {
    return;
  }
  g_last_sample_ms = now_ms;

  // Build one full sensor snapshot.
  SensorSnapshot snap = {};
  snap.t_ms = now_ms;
  snap.flags = 0;

  if (accel_int1) snap.flags |= SNAP_ACCEL_INT1;
  if (accel_int2) snap.flags |= SNAP_ACCEL_INT2;
  if (gauge_alert) snap.flags |= SNAP_GAUGE_ALERT;

  if (adxl_read_xyz(snap.ax, snap.ay, snap.az)) {
    snap.flags |= SNAP_VALID_ACCEL;
  } else {
    snap.ax = 0;
    snap.ay = 0;
    snap.az = 0;
    log_event_if_ready(now_ms, EventCode::SENSOR_FAULT, 1, 0);
  }

  if (pressure_read_raw_adc(snap.pressure_adc_raw)) {
    snap.flags |= SNAP_VALID_PRESSURE;
  } else {
    snap.pressure_adc_raw = 0;
    log_event_if_ready(now_ms, EventCode::SENSOR_FAULT, 2, 0);
  }

  if (battery_readSnapshot(snap.batt)) {
    snap.flags |= SNAP_VALID_BATTERY;
    g_last_batt = snap.batt;
    g_have_battery = true;
  } else {
    snap.batt = {};
    log_event_if_ready(now_ms, EventCode::SENSOR_FAULT, 3, 0);
  }

  // Persist sample to FRAM.
  if (g_logger_ok) {
    if (!logger_appendSample(snap)) {
      g_system_fault = true;
      transition_state(DeviceState::FAULT, REASON_STORAGE_FAULT, now_ms);
      log_event_if_ready(now_ms, EventCode::STORAGE_FAULT, 0, 0);
      return;
    }
  }

  // Safety gate: no normal operation until pressure indicates underwater deployment.
  const bool armed = (snap.flags & SNAP_VALID_PRESSURE) &&
                     (snap.pressure_adc_raw >= ARM_PRESSURE_ADC_THRESHOLD);
  if (!armed) {
    // If we were charging, force safe abort while returning to idle.
    if (pyro_isCharging()) {
      pyro_stopCharge();
      log_event_if_ready(now_ms, EventCode::CHARGE_ABORTED, 0, REASON_NONE);
    }
    transition_state(DeviceState::SAFE_IDLE, REASON_NONE, now_ms);

#if DEBUG_SERIAL
    Serial.print("SAFE_IDLE t=");
    Serial.print(now_ms);
    Serial.print(" p_raw=");
    Serial.println(snap.pressure_adc_raw);
#endif
    return;
  }

  // Enter monitoring once armed and not already charging/fired/fault.
  if (g_state == DeviceState::SAFE_IDLE || g_state == DeviceState::BOOT) {
    transition_state(DeviceState::MONITORING, REASON_NONE, now_ms);
  }

  // Run phase-aware detector.
  const bool in_charge_window = (g_state == DeviceState::CHARGING);
  const DetectorOutput decision = detector_evaluate(snap, in_charge_window);

  // Extra safety timeout from pyro module (independent of detector timeout path).
  if (g_state == DeviceState::CHARGING && pyro_chargeTimedOut(now_ms)) {
    log_event_if_ready(now_ms, EventCode::CHARGE_TIMEOUT, 0, REASON_TIMEOUT);
    if (pyro_isReadyToFire(now_ms) && pyro_fire(now_ms)) {
      transition_state(DeviceState::FIRED, REASON_TIMEOUT, now_ms);
      log_event_if_ready(now_ms, EventCode::FIRE_COMMANDED, 0, REASON_TIMEOUT);
      log_event_if_ready(now_ms, EventCode::FIRED, 0, REASON_TIMEOUT);
      return;
    }
    pyro_stopCharge();
    transition_state(DeviceState::MONITORING, REASON_TIMEOUT, now_ms);
    return;
  }

  // Main state-action handling.
  if (g_state == DeviceState::MONITORING) {
    if (decision.action == DetectorAction::START_CHARGE) {
      log_event_if_ready(now_ms, EventCode::CHARGE_REQUESTED, 0, decision.reasons);
      if (pyro_startCharge(now_ms)) {
        transition_state(DeviceState::CHARGING, decision.reasons, now_ms);
        log_event_if_ready(now_ms, EventCode::CHARGE_STARTED, 0, decision.reasons);
      } else {
        // Could not enter charge state: flag pyro fault and remain monitoring.
        log_event_if_ready(now_ms, EventCode::PYRO_FAULT, 1, decision.reasons);
      }
    }
  } else if (g_state == DeviceState::CHARGING) {
    if (decision.action == DetectorAction::ABORT_CHARGE) {
      pyro_stopCharge();
      transition_state(DeviceState::MONITORING, decision.reasons, now_ms);
      log_event_if_ready(now_ms, EventCode::CHARGE_ABORTED, 0, decision.reasons);
    } else if (decision.action == DetectorAction::FIRE_CUT) {
      log_event_if_ready(now_ms, EventCode::CUT_CONFIRMED, 0, decision.reasons);
      if (pyro_fire(now_ms)) {
        transition_state(DeviceState::FIRED, decision.reasons, now_ms);
        log_event_if_ready(now_ms, EventCode::FIRE_COMMANDED, 0, decision.reasons);
        log_event_if_ready(now_ms, EventCode::FIRED, 0, decision.reasons);
      } else {
        // If fire request occurs but command fails, treat as critical pyro fault.
        g_system_fault = true;
        transition_state(DeviceState::FAULT, REASON_SENSOR_FAULT, now_ms);
        log_event_if_ready(now_ms, EventCode::PYRO_FAULT, 2, decision.reasons);
      }
    }
  }

#if DEBUG_SERIAL
  // Human-readable sample line.
  Serial.print("t=");
  Serial.print(now_ms);
  Serial.print(" state=");
  Serial.print(state_name(g_state));
  Serial.print(" xyz=(");
  Serial.print(snap.ax);
  Serial.print(",");
  Serial.print(snap.ay);
  Serial.print(",");
  Serial.print(snap.az);
  Serial.print(") p_raw=");
  Serial.print(snap.pressure_adc_raw);
  Serial.print(" soc=");
  Serial.print(g_have_battery ? g_last_batt.SOC : 0);
  Serial.print(" flags=0x");
  Serial.println(snap.flags, HEX);

  // Machine-readable stream for live plotting:
  // PLOT,<t_ms>,<x>,<y>,<z>,<state_code>,<int1_flag>
  Serial.print("PLOT,");
  Serial.print(now_ms);
  Serial.print(",");
  Serial.print(snap.ax);
  Serial.print(",");
  Serial.print(snap.ay);
  Serial.print(",");
  Serial.print(snap.az);
  Serial.print(",");
  Serial.print(state_code(g_state));
  Serial.print(",");
  Serial.println(accel_int1 ? 1 : 0);
#endif
}
