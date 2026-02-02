#ifndef TYPES_H
#define TYPES_H
/*
  Shared data types used across modules
  - Defines structs/enums for a sensor snapshot, device states, event codes, and detector outputs
*/
#include <Arduino.h>
#include <stdint.h>

enum class DeviceState : uint8_t { //high-level device state
  BOOT = 0,
  SAFE_IDLE, //idle state
  MONITORING, //normal sensing/logic
  CHARGING, //capacitor charging in progress
  FIRED, //cut fired
  FAULT //error state
};

enum class DetectorAction : uint8_t { //detector output
  NO_ACTION = 0,
  START_CHARGE, //begin charging capacitor
  KEEP_CHARGING, //during charge window, keep charging and evaluating
  ABORT_CHARGE, //during charge window, abort and return to monitoring
  FIRE_CUT //confirm cut and request fire
};

enum DetectorReasonFlags : uint16_t { //reason flags, use for tuning
  REASON_NONE                = 0,
  REASON_DEPTH_ANOMALY       = (1 << 0),
  REASON_ACCEL_ANOMALY       = (1 << 1),
  REASON_TIME_UNDERWATER     = (1 << 2),
  REASON_BATTERY_EOL         = (1 << 3),
  REASON_SENSOR_FAULT        = (1 << 4),
  REASON_STORAGE_FAULT       = (1 << 5),
  REASON_TIMEOUT             = (1 << 6),
  REASON_MANUAL_OVERRIDE     = (1 << 7)
};

struct __attribute__((packed)) DetectorOutput { //detector output returned each time we evaluate a snapshot
  DetectorAction action; //what the state machine should do next
  uint16_t reasons; //bitmask of reason flags
};

struct __attribute__((packed)) BatterySnapshot {
  uint16_t voltage_mv; //millivolts stored as uint16_t to avoid floats
  uint16_t SOC; //SOC in hundreths of a percent (e.g., 7567 = 75.67%)
};

struct __attribute__((packed)) SensorSnapshot {
  uint32_t t_ms; //timestamp (ms since boot)
  //accelerometer raw readings
  int16_t ax;
  int16_t ay;
  int16_t az;

  uint16_t pressure_adc_raw; //accelerometer raw readings for pressure/strain

  BatterySnapshot batt; //battery state

  uint16_t flags; //context flags
}

enum SnapshotFlags : uint16_t { //sensor snapshot flags
  SNAP_VALID_ACCEL    = (1 << 0),
  SNAP_VALID_PRESSURE = (1 << 1),
  SNAP_VALID_BATTERY  = (1 << 2),

  SNAP_ACCEL_INT1     = (1 << 3),
  SNAP_ACCEL_INT2     = (1 << 4),
  SNAP_GAUGE_ALERT    = (1 << 5)
};

//summary of acceleration profile during drop and retrieval
struct __attribute__((packed)) AccelProfile {
  uint16_t duration_ms; //how long the profile window lasted
  uint16_t sample_count; //number of samples used for summary

  //peak magnitude proxy (can be |a| max computed in detector/state machine)
  //stored as raw “magnitude units”
  uint16_t peak_mag;

  //RMS magnitude proxy (fixed-point), store RMS * 100 for extra precision without floats.
  uint16_t rms_mag_x100;
}

struct __attribute__((packed)) CycleSummary {
  uint32_t cycle_idx; //monotonically increasing cycle count
  uint32_t start_t_ms; //ms since boot
  uint32_t end_t_ms;

  //bottom depth poxy: max pressure observed during the cycle
  uint16_t bottom_pressure_adc;

  //pressure changes during soak: min/max during the soak window
  uint16_t soak_min_pressure_adc;
  uint16_t soak_max_pressure_adc;

  //accelerometer profiles
  AccelProfile drop;
  AccelProfile retrieval;
}

enum class LogRecordType : uint8_t {
  BOOT = 0,
  SAMPLE = 1,
  EVENT = 2,
  CYCLE_SUMMARY = 3
};

enum class EventCode : uint8_t { //TODO, adjust these as necessary
  NONE = 0,

  //state transitions
  STATE_CHANGE,

  //two-step detection timeline
  CHARGE_REQUESTED,
  CHARGE_STARTED,
  CHARGE_ABORTED,
  CHARGE_TIMEOUT,
  CUT_CONFIRMED,
  FIRE_COMMANDED,
  FIRED,

  //faults
  SENSOR_FAULT,
  STORAGE_FAULT,
  PYRO_FAULT
};

struct __attribute__((packed)) EventRecord { //TODO, is this necessary?
  uint32_t t_ms;
  EventCode code;
  uint8_t data0; //small parameter (e.g., new state or error ID)
  uint16_t data1; //small parameter (e.g., reason bits LSB)
};

struct __attribute__((packed)) BootRecord { //set information at boot
  uint32_t t_ms;
  char board_serial[DeviceInfo::BOARD_SERIAL_MAX_LEN];
  char fw_version[12];
}

#endif