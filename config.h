#ifndef CONFIG_H
#define CONFIG_H
/*
  Central configuration header
  - Defines pin mappings, device addresses, and compile-time feature flags
  - Contains tunable constants: sample rates, timing delays (settle/charge), thresholds,
    logging settings (record version/size), and debug options
*/
#include <Arduino.h>

#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
// #define ENABLE_PYRO 0
// #define ENABLE_ACCEL_INT 0

namespace Pins {
  //I2C
  constexpr uint8_t SDA = A4; //A4 or D18
  constexpr uint8_t SCL = A5; //A5 or D19

  //Fuel gauge interrupt
  constexpr uint8_t ALERT = 2; //D2

  //SPI
  constexpr uint8_t MOSI = 11; //D11
  constexpr uint8_t MISO = 12; //D12
  constexpr uint8_t SCK = 13; //D13

  //SPI chip selects
  constexpr uint8_t ACCEL_CS = 10; //D10
  constexpr uint8_t FRAM_CS = A0; //A0 or D14

  //ADXL interrupts
  constexpr uint8_t ACCEL_INT1 = 3; //D3
  constexpr uint8_t ACCEL_INT1 = 4; //D4

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
  constexpr uint8_t MAX17048_ADDR = ; //TODO: find in datasheet
}

namespace SPIParams {
  constexpr uint32_t SLOW_HZ = ; //TODO 
  constexpr uint32_t FAST_HZ = ; //TODO
  constexpr uint8_t MODE0 = 0;
  constexpr uint8_t MODE3 = 3;
}

namespace Timing {
  //TODO
}

namespace Pyro {
  //TODO
}

namespace Logging {
  //TODO
}

namespace Detector {
  //TODO
}

#endif
