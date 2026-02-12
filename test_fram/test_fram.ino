// MOSI --> SI
// MISO --> SO
// A0 or D14 ---> CS
#include <Arduino.h>
#include "FRAM.h"

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Set pin 10 as Output to stay in master mode
  pinMode(10, OUTPUT);

  Serial.println("  FRAM Test Starting  ");

  fram_init();

  // Read status register
  uint8_t status = fram_read_status();
  Serial.print("Status Register: 0x");
  Serial.println(status, HEX);

  // Test write + read
  uint8_t write_value = 0xAA;
  uint8_t read_value = 0x00;

// address writing to, we can change tht up in testing later to test with different address points
  bool write_ok = fram_write(0x000000, &write_value, 1);
  bool read_ok  = fram_read(0x000000, &read_value, 1);


  Serial.print("Write OK: ");
  if (write_ok) {
    Serial.println("YES");
  } else {
    Serial.println("NO");
}

  Serial.print("Read OK: ");
    if (read_ok) {
    Serial.println("YES");
  } else {
    Serial.println("NO");
}

  Serial.print("Read Back Value: 0x");
  Serial.println(read_value, HEX);
}


void loop() {
  static uint8_t counter = 0;

  uint8_t write_value = counter;
  uint8_t read_value = 0;

  fram_write(0x000010, &write_value, 1);
  fram_read(0x000010, &read_value, 1);

  Serial.print("Wrote: 0x");
  Serial.println(write_value, HEX);
  Serial.print("Read: 0x");
  Serial.println(read_value, HEX);

  counter++;
  delay(1000);
}
