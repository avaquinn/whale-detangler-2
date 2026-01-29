/*
  I2C abstraction
  - Provides simple register read/write helpers with optional retries/timeouts
  - Used primarily by the battery fuel gauge
*/
#include "I2C.h"

//initialize I2C peripheral and set clock speed
void i2c_init() {
  Wire.begin(); //enable I2C peripheral (joins bus as controller)
  Wire.setClock(400000); //set I2C clocl to 400 kHz (fast mode)
}

//writes a 16-bit value to a device register over I2C (MSB first)
/*
Input:
  addr = 7-bit I2C device address
  reg = register address inside the device to write to
  data = 16-bit register value to write
  retries = number of retries after the first attempt
Output:
  true = the write transaction completes successfully
  false = all attempts to write fail
*/
bool i2c_write(uint8_t addr, uint8_t reg, uint16_t data, uint8_t retries) {
  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    Wire.beginTransmission(addr); //START + send address + write bit
    Wire.write(reg); //send the internal register address we want to write

    //send data
    Wire.write((uint8_t)((data >> 8) & 0xFF)); //MSB
    Wire.write((uint8_t)(data & 0xFF)); //LSB

    uint8_t status = Wire.endTransmission(true); //send STOP
    if (status == 0) { //check for success
      return true;
    }

    delay(2); //short pause before retry
  }

  return false; //all attempts fail
}

//read a 16-bit tregister value over I2C (MSB first)
/*
Input:
  addr = 7-bit I2C device address
  reg = register address inside the device to read from
  data = reference to where the 16-bit result will be stored
  retries = number of retries after the first attempt
Output:
  true = exactly 2 bytes were read and combined successfully
  false = all attempts to read fail
*/
bool i2c_read(uint8_t addr, uint8_t reg, uint16_t &out, uint8_t retries) {
  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    //tell device which register we want to read
    Wire.beginTransmission(addr);
    Wire.write(reg);

    uint8_t status = Wire.endTransmission(false); //send REPEATED START for read
    if (status != 0) { //check for success
      //something went wrong, retry
      delay(2);
      continue;
    }

    //request 2 bytes from the device
    size_t received = Wire.requestFrom(addr, 2);
    if (received != 2) { //check for success
      //drain any leftover bytes so next attempt starts clean
      while (Wire.available()) {
        (void)Wire.read(); //discard byte
      }

      delay(2);
      continue;
    }

    //read and combine MSB first
    uint8_t msb = (uint8_t)Wire.read();
    uint8_t lsb = (uint8_t)Wire.read();
    out = (uint16_t(msb) << 8) | uint16_t(lsb);
    
    return true; //success
  }

  return false; //all attempts failed
}