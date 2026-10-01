/*
  I2C abstraction
  - Register read/write helpers with retries and a bus timeout (AVR Wire otherwise blocks forever
    on a stuck bus, which would freeze the whole device)
  - Used by the MAX17048 fuel gauge (0x36) and the optional NAU7802 bridge ADC (0x2A)
*/
#include <Wire.h>
#include "i2c_bus.h"

static constexpr uint32_t I2C_TIMEOUT_US = 25000UL;

void i2c_init() {
  Wire.begin();
  Wire.setClock(400000); //fast mode; both devices support it
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(I2C_TIMEOUT_US, true); //true = reset the TWI hardware on timeout
#endif
}

static void after_failure() {
#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif
  delay(2); //short pause before retry
}

//read 'len' bytes starting at 'reg' (repeated start between address and read)
bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len, uint8_t retries) {
  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0) {
      if (Wire.requestFrom(addr, len) == len) {
        for (uint8_t i = 0; i < len; i++) {
          out[i] = (uint8_t)Wire.read();
        }
        return true;
      }
      while (Wire.available()) { //drain a short read so the next attempt starts clean
        (void)Wire.read();
      }
    }
    after_failure();
  }
  return false;
}

//write 'len' bytes starting at 'reg'
bool i2c_write_bytes(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len, uint8_t retries) {
  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data, len);
    if (Wire.endTransmission(true) == 0) {
      return true;
    }
    after_failure();
  }
  return false;
}

bool i2c_read_u8(uint8_t addr, uint8_t reg, uint8_t &out, uint8_t retries) {
  return i2c_read_bytes(addr, reg, &out, 1, retries);
}

bool i2c_write_u8(uint8_t addr, uint8_t reg, uint8_t data, uint8_t retries) {
  return i2c_write_bytes(addr, reg, &data, 1, retries);
}

//16-bit register, MSB first (MAX17048 convention)
bool i2c_read_u16be(uint8_t addr, uint8_t reg, uint16_t &out, uint8_t retries) {
  uint8_t b[2];
  if (!i2c_read_bytes(addr, reg, b, 2, retries)) {
    return false;
  }
  out = (uint16_t(b[0]) << 8) | b[1];
  return true;
}

bool i2c_write_u16be(uint8_t addr, uint8_t reg, uint16_t data, uint8_t retries) {
  uint8_t b[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
  return i2c_write_bytes(addr, reg, b, 2, retries);
}
