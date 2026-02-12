#include <Arduino.h>
#include "ADXL.h"

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("  ADXL363 Test Starting  ");


// Stop if the device was not detected
  if (!adxl_init()) {
    Serial.println("ADXL INIT FAILED!");
    while (1);   
  }

  Serial.println("ADXL INIT SUCCESS");

  uint8_t status = 0;
  if (adxl_read_status(status)) {
    Serial.print("STATUS Register: 0x");
    Serial.println(status, HEX);
  } else {
    Serial.println("Failed to read STATUS register");
  }
}


void loop() {
  int16_t x = 0, y = 0, z = 0;
  uint16_t adc = 0;

  bool accel_ok = adxl_read_xyz(x, y, z);
  bool adc_ok   = adxl_read_adc(adc);

  if (accel_ok) {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("  Y: ");
    Serial.print(y);
    Serial.print("  Z: ");
    Serial.println(z);
  } else {
    Serial.println("Failed to read XYZ");
}

  if (adc_ok) {
    Serial.print("ADC Raw: ");
    Serial.println(adc);
  } else {
    Serial.println("Failed to read ADC");
}

  Serial.println("------------------------");

  delay(500);
}