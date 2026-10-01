#ifndef TYPES_H
#define TYPES_H
/*
  Shared data types used across modules
  - Device state, deployment phase, detector outputs, sensor snapshot, and FRAM log payloads
  - Log payloads are packed and written raw; bump the logger layout version if any of them change
*/
#include "config.h"

enum class DeviceState : uint8_t { //high-level device state
  BOOT = 0,
  SAFE_IDLE, //at the surface / not armed
  MONITORING, //armed underwater, running the detector
  CHARGING, //capacitor charging, confirming the anomaly
  FIRED, //cut fired (latched in FRAM)
  FAULT //hardware init failure, service required
};

enum class DeployPhase : uint8_t { //where the pot is in a normal set/soak/haul cycle
  SURFACE = 0,
  DESCENT,
  SOAK,
  ASCENT
};

enum class DetectorAction : uint8_t { //detector output
  NO_ACTION = 0,
  START_CHARGE, //begin charging capacitor
  KEEP_CHARGING, //during charge window, keep charging and evaluating
  ABORT_CHARGE, //during charge window, abort and return to monitoring
  FIRE_CUT //anomaly persisted through the confirm window, request fire
};

enum DetectorReasonFlags : uint16_t { //why the detector acted; logged with every decision
  REASON_NONE             = 0,
  REASON_CRUSH_DEPTH      = (1 << 0), //deeper than any legal set
  REASON_SOAK_EXCURSION   = (1 << 1), //left the soak depth without a clean haul
  REASON_REVERSAL         = (1 << 2), //went the wrong way during descent/ascent
  REASON_TOWED            = (1 << 3), //sustained motion without upward travel
  //bit 4 unused (reserved)
  REASON_TIME_UNDERWATER  = (1 << 5), //ghost-gear timer expired
  REASON_SENSOR_FAULT     = (1 << 6),
  REASON_STORAGE_FAULT    = (1 << 7),
  REASON_TIMEOUT          = (1 << 8), //pyro charge timeout (always aborts)
  REASON_SHALLOW          = (1 << 9) //anomaly present but too shallow to fire
};

struct DetectorOutput {
  DetectorAction action; //what the state machine should do next
  uint16_t reasons; //bitmask of DetectorReasonFlags
};

struct __attribute__((packed)) BatterySnapshot {
  uint16_t voltage_mv;
  uint16_t soc_x100; //SOC in hundredths of a percent (7567 = 75.67%)
};

//linear depth calibration: depth_cm = (raw - zero_raw) * cm_per_count
struct __attribute__((packed)) PressureCal {
  int32_t zero_raw;
  float cm_per_count;
  uint8_t valid;
};

enum SnapshotFlags : uint16_t {
  SNAP_VALID_ACCEL    = (1 << 0),
  SNAP_VALID_PRESSURE = (1 << 1), //raw reading ok (powered, not clipped)
  SNAP_VALID_DEPTH    = (1 << 2), //raw ok AND calibration valid
  SNAP_VALID_BATTERY  = (1 << 3), //batt holds a reading (may be up to BATTERY_READ_PERIOD_MS old)
  SNAP_VALID_TEMP     = (1 << 4),
  SNAP_ACCEL_ACTIVITY = (1 << 5), //ADXL activity interrupt since the last sample
  SNAP_GAUGE_ALERT    = (1 << 6), //fuel gauge ALRT serviced since the last sample
  SNAP_PRESSURE_CLIP  = (1 << 7) //front end saturated
};

struct __attribute__((packed)) SensorSnapshot {
  uint32_t t_ms; //ms since boot
  int16_t ax; //raw accelerometer counts (Accel::MG_PER_LSB)
  int16_t ay;
  int16_t az;
  int16_t temp_raw; //ADXL363 temperature, raw counts (for drift correlation)
  int32_t pressure_raw; //front-end counts (see config.h for per-backend meaning)
  int16_t depth_cm; //calibrated depth, valid only with SNAP_VALID_DEPTH
  BatterySnapshot batt;
  uint16_t flags; //SnapshotFlags
  uint8_t state; //DeviceState at the time of logging
  uint8_t phase; //DeployPhase at the time of logging
};

struct __attribute__((packed)) AccelProfile { //summary of acceleration during descent or ascent
  uint32_t duration_ms;
  uint16_t sample_count;
  uint16_t peak_mg; //max dynamic acceleration | |a| - 1 g |
  uint16_t rms_mg; //RMS dynamic acceleration
};

struct __attribute__((packed)) CycleSummary { //one set -> soak -> haul cycle
  uint32_t cycle_idx; //monotonically increasing over the device's life
  uint32_t start_t_ms; //ms since boot at immersion
  uint32_t end_t_ms; //ms since boot at return to the surface
  int16_t bottom_depth_cm; //max depth seen
  int16_t soak_min_depth_cm; //depth range while soaking (tide, sea state)
  int16_t soak_max_depth_cm;
  uint32_t soak_duration_ms;
  AccelProfile drop;
  AccelProfile retrieval;
};

enum class LogRecordType : uint8_t {
  BOOT = 0,
  SAMPLE = 1,
  EVENT = 2,
  CYCLE_SUMMARY = 3
};

enum class EventCode : uint8_t {
  NONE = 0,
  STATE_CHANGE, //data0 = new DeviceState, data1 = reasons
  PHASE_CHANGE, //data0 = new DeployPhase, data1 = depth_cm
  CHARGE_REQUESTED, //data1 = reasons
  CHARGE_STARTED,
  CHARGE_ABORTED,
  CHARGE_TIMEOUT,
  FIRED,
  SENSOR_FAULT, //data0 = SensorId bitmask
  SENSOR_RECOVERED, //data0 = SensorId bitmask
  STORAGE_FAULT,
  PYRO_FAULT, //data0 = 1 start failed, 2 fire failed
  BATTERY_ALERT, //data0 = BatteryAlerts bits, data1 = voltage_mv
  CAL_CHANGED, //data0 = valid
  REARMED //fired latch cleared from the console
};

enum SensorId : uint8_t { //bitmask used in SENSOR_FAULT / SENSOR_RECOVERED events
  SENSOR_ACCEL = (1 << 0),
  SENSOR_PRESSURE = (1 << 1),
  SENSOR_BATTERY = (1 << 2),
  SENSOR_STORAGE = (1 << 3)
};

struct __attribute__((packed)) EventRecord {
  uint32_t t_ms;
  EventCode code;
  uint8_t data0;
  uint16_t data1;
};

struct __attribute__((packed)) BootRecord { //one per boot; identity lives in the FRAM header
  uint8_t reset_cause; //MCUSR at boot (may be 0 if the bootloader cleared it)
  char fw_version[12];
};

#endif
