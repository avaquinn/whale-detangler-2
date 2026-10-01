#ifndef CONFIG_H
#define CONFIG_H
/*
  Central configuration header
  - Defines pin mappings, compile-time feature flags, and every tunable constant
  - Depths are in centimetres of seawater (1 ft = 30.48 cm, 1 psi ~= 68.6 cm)
*/
#include <Arduino.h>

//---------------------------------------------------------------------------------------------
//feature flags
//---------------------------------------------------------------------------------------------
#define DEBUG_SERIAL 1 //per-sample human + PLOT lines on Serial (the command console is always on)
#define SERIAL_BAUD 115200

//1 = PYRO_CHG / PYRO_FIRE are never driven high. The full state machine still runs and logs
//(including the persistent FIRED latch), so detection can be tested on the bench.
//Leave at 1 until the pyro board works and a simulated initiator load is attached.
#define BENCH_NO_PYRO 1

//2 s hardware watchdog. The stock Pro Mini 8 MHz bootloader (ATmegaBOOT) may not survive a
//watchdog reset and can boot-loop until reflashed over ISP. Burn Optiboot (e.g. MiniCore) first.
#define ENABLE_WATCHDOG 0

//pressure front end
#define PRESSURE_FE_ADXL_AUX 0 //Rev 1.0 board: bridge -> INA333 -> ADXL363 aux ADC (12-bit)
#define PRESSURE_FE_NAU7802 1  //Rev 1.1 proposal: bridge -> NAU7802 24-bit ratiometric ADC (I2C)
#define PRESSURE_FRONTEND PRESSURE_FE_ADXL_AUX

namespace Pins {
  //hardware SPI (D11/D12/D13) and I2C (A4/A5) pins are fixed by the ATmega328P

  //fuel gauge ALRT (open-drain, active-low, latched until serviced)
  constexpr uint8_t ALERT = 2; //D2

  //SPI chip selects (active-low)
  constexpr uint8_t ACCEL_CS = 10; //D10
  constexpr uint8_t FRAM_CS = A0; //A0 / D14

  //ADXL363 interrupts (active-high, latched until STATUS is read)
  constexpr uint8_t ACCEL_INT1 = 3; //D3
  constexpr uint8_t ACCEL_INT2 = 4; //D4

  //analog front-end power gate: Q1 is a P-FET with R7 pulling its gate to VCC,
  //so the bridge is powered when this pin is LOW
  constexpr uint8_t PFET_EN = 9; //D9
  constexpr uint8_t PFET_ON = LOW;
  constexpr uint8_t PFET_OFF = HIGH;

  //tri-colour LED, common anode (LOW = on)
  constexpr uint8_t LED_RED = 5; //D5
  constexpr uint8_t LED_GREEN = 6; //D6
  constexpr uint8_t LED_YELLOW = 7; //D7

  //pyro control (100k pull-downs on the board keep these low while the MCU is in reset)
  constexpr uint8_t PYRO_CHG = A1; //A1 / D15
  constexpr uint8_t PYRO_FIRE = A2; //A2 / D16
}

namespace Timing {
  //sample period per deployment phase; slow when nothing is expected to change
  constexpr uint16_t SURFACE_SAMPLE_PERIOD_MS = 1000; //waiting for immersion
  constexpr uint16_t TRANSIT_SAMPLE_PERIOD_MS = 200; //descent / ascent
  constexpr uint16_t SOAK_SAMPLE_PERIOD_MS = 1000; //static on the bottom
  constexpr uint16_t CHARGE_SAMPLE_PERIOD_MS = 100; //charge/confirm window

  //fuel gauge SOC changes slowly; no need to poll it every sample
  constexpr uint32_t BATTERY_READ_PERIOD_MS = 60000UL;
  //minimum spacing between attempts to service a stuck ALRT line
  constexpr uint16_t BATTERY_ALERT_RETRY_MS = 1000;
}

namespace Pressure {
  //ADXL aux-ADC front end: the ADC converts once per accelerometer ODR period (10 ms at 100 Hz),
  //so wait several periods after powering the bridge, then average fresh conversions
  constexpr uint16_t ADXL_SETTLE_MS = 30;
  constexpr uint8_t ADXL_ODR_PERIOD_MS = 10;
  constexpr uint8_t ADXL_AVG_SAMPLES = 4; //raw value is the SUM of these; recalibrate if changed
  constexpr int16_t ADXL_CLIP_COUNTS = 2040; //|code| at or above this means the INA333/ADC is clipped

  //NAU7802 front end
  constexpr uint8_t NAU_ADDR = 0x2A;
  constexpr uint8_t NAU_LDO_CODE = 0b101; //AVDD = 3.0 V (keeps ~0.3 V headroom below the 3.3 V rail)
  constexpr uint8_t NAU_GAIN_CODE = 0b111; //PGA x128
  constexpr uint8_t NAU_RATE_CODE = 0b011; //80 SPS (12.5 ms per conversion)
  constexpr uint16_t NAU_SETTLE_MS = 20; //after power-up, before the first conversion is trusted (tune with strain_bridge test 2)
  constexpr uint16_t NAU_CAL_SETTLE_MS = 250; //LDO + bridge settle before the internal offset calibration
  constexpr uint8_t NAU_DISCARD = 2; //conversions thrown away after power-up
  constexpr uint8_t NAU_AVG_SAMPLES = 4; //raw value is the AVERAGE of these
  constexpr int32_t NAU_CLIP_COUNTS = 8388000L; //|code| near full scale (2^23)

  //console calibration: readings averaged per "cal zero" / "cal span" point
  constexpr uint8_t CAL_READINGS = 16;
  //reject a span point closer than this to zero (counts)
  constexpr int32_t CAL_MIN_SPAN_COUNTS = 20;
}

namespace Depth { //TODO: reconcile arm depth, Gen 2 requirement says 7 psi / 15 ft, client docs say 30 ft
  constexpr int16_t ARM_DEPTH_CM = 457; //15 ft: below this the device is SAFE_IDLE
  constexpr int16_t ARM_HYST_CM = 60; //must rise above ARM_DEPTH_CM - ARM_HYST_CM to disarm
  constexpr int16_t MIN_FIRE_DEPTH_CM = 914; //30 ft: never charge or fire shallower than this
  constexpr int16_t CRUSH_DEPTH_CM = 12000; //~394 ft: deeper than any legal set, treat as dragged down
}

namespace Detector { //placeholders: every value here must be tuned with pier / tow data
  //depth is decimated to 1 Hz; "smoothed depth" is the mean of the last SMOOTH_S points, which
  //averages out surface-wave pressure (5-20 s periods). Rate = change of smoothed depth over RATE_LAG_S.
  //Rate-based checks are disabled until SMOOTH_S + RATE_LAG_S seconds of history exist.
  constexpr uint8_t SMOOTH_S = 20;
  constexpr uint8_t RATE_LAG_S = 10;

  //|rate| at or below this counts as "still" (cm/s)
  constexpr int16_t STILL_RATE_CM_S = 5;
  //rising faster than this counts as a haul (pots come up at ~75-300 cm/s)
  constexpr int16_t HAUL_RATE_CM_S = 30;
  //moving the "wrong way" faster than this is a reversal (deeper during ascent, shallower during descent)
  constexpr int16_t REVERSAL_RATE_CM_S = 30;

  //still for this long means the pot is resting (on the bottom, or hanging during a hauler pause)
  constexpr uint32_t SOAK_SETTLE_MS = 20000UL;
  //descents take 30-120 s; after this long, treat the pot as soaking even if never perfectly still
  constexpr uint32_t MAX_DESCENT_MS = 600000UL;
  //smoothed-depth excursion from the soak baseline that is not a clean haul
  constexpr int16_t SOAK_EXCURSION_CM = 300;

  //accelerometer: dynamic acceleration above this counts as motion
  constexpr uint16_t MOTION_MG = 150;
  //"towed": accumulated motion without upward travel (soak, or a stopped ascent)
  constexpr uint32_t TOWED_MOTION_MS = 30000UL;
  //calm for this long resets the motion accumulator
  constexpr uint32_t CALM_RESET_MS = 10000UL;

  //optional ghost-gear timer: anomaly after this long underwater (0 = disabled, max 1190 h)
  constexpr uint16_t MAX_DEPLOY_HOURS = 0;

  //consecutive anomalous samples before requesting a charge
  constexpr uint8_t PRECHARGE_CONFIRM_COUNT = 5;
  //consecutive normal samples inside the charge window that abort
  constexpr uint8_t ABORT_CONFIRM_COUNT = 5;
  //the anomaly must still be present this long after charging starts before FIRE is requested
  constexpr uint16_t FIRE_CONFIRM_MS = 3000;
}

namespace Pyro {
  //minimum time the capacitor charges before FIRE is permitted
  constexpr uint16_t MIN_CHARGE_MS = 2000;
  //charging longer than this always ABORTS (hard fail-safe, independent of the detector)
  constexpr uint16_t MAX_CHARGE_MS = 6000;
  //fire pulse width (the initiator functions in <= 10 ms)
  constexpr uint16_t FIRE_PULSE_MS = 10;
}

static_assert(Detector::FIRE_CONFIRM_MS >= Pyro::MIN_CHARGE_MS, "fire confirm must allow a full charge");
static_assert(Detector::FIRE_CONFIRM_MS < Pyro::MAX_CHARGE_MS, "fire confirm must finish before the charge timeout");
static_assert(Depth::MIN_FIRE_DEPTH_CM >= Depth::ARM_DEPTH_CM, "never fire shallower than the arm depth");

namespace Accel {
  //±8 g range => 4 mg per LSB
  constexpr uint8_t MG_PER_LSB = 4;
  //activity threshold for tap/motion interrupts (LSB of the ±8 g range, referenced mode)
  constexpr uint16_t ACTIVITY_THRESHOLD = 140; //~560 mg above the reference
  constexpr uint8_t ACTIVITY_TIME = 1; //samples above threshold; taps are short
}

namespace Ui {
  //triple-tap (three activity events inside the window) shows battery status, only at the surface
  constexpr uint8_t TAP_COUNT = 3;
  constexpr uint16_t TAP_WINDOW_MS = 1500;
  constexpr uint16_t TAP_DEBOUNCE_MS = 120;
  constexpr uint16_t BATTERY_DISPLAY_MS = 5000;

  //SOC (x100) to LED class; placeholder until SOC -> days-left is characterised
  constexpr uint16_t SOC_X100_GREEN_MIN = 7000;
  constexpr uint16_t SOC_X100_YELLOW_MIN = 3000;
  constexpr uint16_t SOC_X100_RED_MIN = 1200;
}

namespace Logging {
  //FRAM layout (MB85RS2MTA, 262,144 bytes)
  //0x00000-0x000FF : two header slots (A/B, alternating, CRC-protected)
  //0x00100-0x2FFFF : journal ring (boot, event, cycle-summary records), sized for 12+ years
  //0x30000-0x3FFFF : sample ring (raw snapshots while underwater), ~2,000 most recent samples
  constexpr uint32_t HEADER_SLOT_BYTES = 128;
  constexpr uint32_t JOURNAL_START = 0x00100UL;
  constexpr uint32_t JOURNAL_END = 0x30000UL;
  constexpr uint32_t SAMPLE_START = 0x30000UL;
  constexpr uint32_t SAMPLE_END = 0x40000UL;

  //1 = also log samples while at the surface (useful on the bench, fills the ring quickly)
  constexpr bool LOG_SURFACE_SAMPLES = false;
}

namespace DeviceInfo {
  //serial is a human-readable ID like "18.09.10001" (YY.MM.1000X); set once with the console "serial" command
  constexpr uint8_t BOARD_SERIAL_MAX_LEN = 16;
  constexpr char BOARD_SERIAL_DEFAULT[] = "UNSET";
  constexpr char FW_VERSION_STR[] = "0.2.0";
}

#endif
