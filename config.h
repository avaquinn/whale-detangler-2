#ifndef CONFIG_H
#define CONFIG_H
/*
  Central configuration header
  - Defines pin mappings, device addresses, and compile-time feature flags
  - Contains tunable constants: sample rates, timing delays (settle/charge), thresholds,
    logging settings (record version/size), and debug options
*/
#include <Arduino.h>

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
  constexpr uint8_t PYRO_CHRG = A1; //A1 or D15
  constexpr uint8_t PYRO_FIRE = A2; //A2 or D16
}

namespace I2CDevices {
  //0x6C for write, 0x6D for read; need to shift left and add bit to the right
  constexpr uint8_t MAX17048_ADDR = 0x36;
}

namespace SPIParams {
  constexpr uint32_t SLOW_HZ = 1000000; //1MHz
  constexpr uint32_t FAST_HZ = 4000000; //4MHz
  constexpr uint8_t MODE0 = 0; //ADXL and FRAM
  constexpr uint8_t MODE3 = 3; //FRAM
}

namespace Timing { //TODO: adjust later for power and responsiveness
  //main sensor sample period during normal monitoring
  constexpr uint16_t SAMPLE_PERIOD_MS = 200;//5Hz

  //may want to sample faster during charge window to confirm/abort quickly
  constexpr uint16_t CHARGE_WINDOW_SAMPLE_PERIOD_MS = 50; //20Hz

  //after enabling PFET, allow signal chain to settle (amp + filters + bridge supply)
  constexpr uint16_t PRESSURE_SETTLE_MS = 10;

  //debounce/minimum time between status LED updates (prevents flicker)
  constexpr uint16_t STATUS_TICK_MS = 50;
}

namespace Pyro {
  //minimum time to allow capacitor to charge before we permit a FIRE command
  constexpr uint16_t MIN_CHARGE_MS = 2000; //>= 2s charge window

  //maximum time we will keep charging before aborting (prevents endless charge state)
  constexpr uint16_t MAX_CHARGE_MS = 1; //TODO

  //maximum fire pulse width; TODO: might not need this, I think it's handled on pyro board
  // constexpr uint16_t FIRE_PULSE_MS = 10; //<= 10ms

  //minimum time between any two fire attempts (should normally be never, but good for guard); potentially TODO
  constexpr uint32_t MIN_REFIRE_LOCKOUT_MS = 60000UL; //60s
}

namespace Logging {
  //total amount of bytes in FRAM
  constexpr uint32_t FRAM_TOTAL_BYTES = 262144UL

  //reserve small header region for meatdata?
  constexpr uint16_t HEADER_BYTES = 256;

  //where records begin
  constexpr uint32_t DATA_START_ADDR = HEADER_BYTES;

  // constexpr uint16_t MAX_RECORD_BYTES = 64; TODO: max record size to simplify buffering?
}

namespace Detector {
  //how many consecutive suspicious samples before we request START_CHARGE
  constexpr uint8_t PRECHARGE_CONFIRM_COUNT = 5;

  //how many consecutive normal/safe samples during charge window to ABORT
  constexpr uint8_t ABORT_CONFIRM_COUNT = 5;

  //maximum time to wait in charge-confirm stage before forcing an abort
  constexpr uint16_t CHARGE_CONFIRM_TIMEOUT_MS = Pyro::MAX_CHARGE_MS;
}

#endif
