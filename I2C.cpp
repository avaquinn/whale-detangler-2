/*
  I2C abstraction
  - Provides simple register read/write helpers with optional retries/timeouts
  - Used primarily by the battery fuel gauge
*/
# include "I2C.h"

// The I2C peripheral initialized. Using the built in wrapper to initialize the SDA (A4) & SCL (A5) pins
void i2c_init(){
  // pins A4 and A5 are now set up for I2C communication. No need to maually set them using pinmodes
    Wire.begin();
}


// I2C write function --> bool so we know if the data transmission was successful or not
// So far only writing one byte at a time. Assuming we will read from fuel gague majority of the times so as of right now only writing one byte at a time. We can fix it later if we want to write in a loop
/* varaible:
  addr = device address
  reg = internal regsiters in gague to write to for probably alerts
  data = data to write to these registers
*/
bool i2c_write(uint8_t addr, uint8_t reg, uint8_t data){

  // take care of Start condition, sending device address, Write bit, ACK bit. Now ready to write
  Wire.beginTransmission(addr);

  // pass in the internal register address to write to
  Wire.write(reg);
  // Now that we have the internal register set, we can write to it.
  Wire.write(data);


// check if the end of transmission was successfully closed
  int transmission_status = 0;
  transmission_status = Wire.endTransmission();

// checks is transmission was successfully closed, after successfully starting
  return (transmission_status == 0);

}


bool i2c_read(uint8_t addr, uint8_t reg, size_t n, uint8_t *buffer){
  /* begin transmission, have to be in write mode first, but by passing in false
  into endTransmission we can to a repetaed Start, Not Stop */
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  // Start reading from the device n bytes look into that in datasheet.
  size_t bytes_read;
  bytes_read = Wire.requestFrom(addr, n);

  if (bytes_read != n) {
        return false;  // did not receive expected bytes
    }

  // for (int i = 0, i < n, i++){
  //   buffer[i] = Wire.read();
  // }


}

