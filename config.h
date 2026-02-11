#ifndef CONFIG_H
#define CONFIG_H
/*
  Central configuration header
  - Defines pin mappings, device addresses, and compile-time feature flags
  - Contains tunable constants: sample rates, timing delays (settle/charge), thresholds,
    logging settings (record version/size), and debug options
*/

#define DEBUG_SERIAL 1 //Serial prints for debugging
#define DEBUG_BAUD 115200

namespace Pins {
  //I2C
  constexpr uint8_t SDA = A4; //A4 or D18
  constexpr uint8_t SCL = A5; //A5 or D19

  //Fuel gauge interrupt
  constexpr uint8_t ALERT = 2; //D2 --> interrupt for SOC alert

  //SPI
  constexpr uint8_t MOSI = 11; //D11
  constexpr uint8_t MISO = 12; //D12
  constexpr uint8_t SCK = 13; //D13

  //SPI chip selects
  constexpr uint8_t ACCEL_CS = 10; //D10
  constexpr uint8_t FRAM_CS = A0; //A0 or D14

  //ADXL interrupts
  constexpr uint8_t ACCEL_INT1 = 3; //D3 --> primary interrupt for detection
  constexpr uint8_t ACCEL_INT2 = 4; //D4 --> secondary interrupt for triple-tap LED status

  //Analog front-end power gating
  constexpr uint8_t PFET_EN = 9; //D9

  //LEDs
  constexpr uint8_t LED_RED = 5; //D5
  constexpr uint8_t LED_GREEN = 6; //D6
  constexpr uint8_t LED_YELLOW = 7; //D7

  //Pyro control
  constexpr uint8_t PYRO_CHG = A1; //A1 or D15
  constexpr uint8_t PYRO_FIRE = A2; //A2 or D16
}

namespace Timing { //TODO: adjust later for power and responsiveness
  //main sensor sample period during normal monitoring
  constexpr uint16_t SAMPLE_PERIOD_MS = 200;//5Hz

  //may want to sample faster during charge window to confirm/abort quickly
  constexpr uint16_t CHARGE_WINDOW_SAMPLE_PERIOD_MS = 50; //20Hz

  //after enabling PFET, allow signal chain to settle (amp + filters + bridge supply)
  constexpr uint16_t PRESSURE_SETTLE_MS = 10;
}

namespace Pyro {
  //minimum time to allow capacitor to charge before we permit a FIRE command
  constexpr uint16_t MIN_CHARGE_MS = 2000; //>= 2s charge window

  //maximum time we will keep charging before aborting (prevents endless charge state)
  constexpr uint16_t MAX_CHARGE_MS = 6000; //TODO

  //maximum fire pulse width; TODO: might not need this, I think it's handled on pyro board
  // constexpr uint16_t FIRE_PULSE_MS = 10; //<= 10ms

  //minimum time between any two fire attempts (should normally be never, but good for guard); potentially TODO
  constexpr uint32_t MIN_REFIRE_LOCKOUT_MS = 60000UL; //60s
}

namespace Logging {
  //reserve small header region for metadata
  constexpr uint16_t HEADER_BYTES = 256;

  //where records begin
  constexpr uint32_t DATA_START_ADDR = HEADER_BYTES;

  // constexpr uint16_t MAX_RECORD_BYTES = 64; TODO: max record size to simplify buffering?
}

namespace DeviceInfo {
  //serial is a human-readable ID like "18.09.10001" (YY.MM.1000X format)
  //should be stored once in FRAM header (or set during programming), not logged every sample
  constexpr uint8_t BOARD_SERIAL_MAX_LEN = 16; //plenty for "YY.MM.1000X" + null terminator

  //compile-time placeholder, set this during programming and write it to FRAM header
  constexpr char BOARD_SERIAL_DEFAULT[] = "UNSET";
  
  //firmware version
  constexpr char FW_VERSION_STR[] = "0.1.0";
}

namespace Detector {
  //how many consecutive suspicious samples before we request START_CHARGE
  constexpr uint8_t PRECHARGE_CONFIRM_COUNT = 5;

  //how many consecutive normal/safe samples during charge window to ABORT
  constexpr uint8_t ABORT_CONFIRM_COUNT = 5;

  //maximum time to wait in charge-confirm stage before forcing an abort
  constexpr uint16_t CHARGE_CONFIRM_TIMEOUT_MS = Pyro::MAX_CHARGE_MS;
}

namespace CycleCfg { //placeholders until we decide pressure units (raw ADC vs converted)
  //we can start using ADC thresholds, then switch to PSI/ft once calibrated.
  constexpr uint16_t START_PRESSURE_ADC = 0; // TODO: set after calibration
  constexpr uint16_t END_PRESSURE_ADC = 0; // TODO: set after calibration

  //debounce times so noise doesn’t create fake cycles
  constexpr uint16_t START_DEBOUNCE_MS = 2000;
  constexpr uint16_t END_HOLD_MS = 5000;
}

// namespace ProfileCfg { //profile capture during DROP and RETRIEVAL:
//   // - We’ll log a summary always.
//   // - Optionally log this many raw samples around the event (kept small for FRAM).
//   constexpr uint16_t RAW_PROFILE_HZ = 50;        // matches charge-window sample rate
//   constexpr uint16_t DROP_RAW_SAMPLES = 200;     // 4 seconds @ 50 Hz
//   constexpr uint16_t RETR_RAW_SAMPLES = 200;     // 4 seconds @ 50 Hz
// }

#endif
