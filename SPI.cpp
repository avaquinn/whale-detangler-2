/*
  Hardware SPI abstraction
  - Wraps Arduino SPI usage with consistent begin/transfer helpers
  - Provides safe per-device transactions (clock/mode) and chip-select discipline
  - Used by SPI peripherals (accelerometer and FRAM)
*/
