/*
  Pressure/strain sensing module built around the analog front end + ADXL363 auxiliary ADC
  - Controls the PFET power gate for the INA333/bridge circuitry to save power
  - Applies settle timing, then reads raw ADC via the ADXL363
  - Optionally applies calibration/scaling to convert raw readings to engineering units
*/
#include <Arduino.h>
#include "pressure.h"
#include "config.h"
#include "ADXL.h"

//configure pressure front-end power control (PFET) to a safe default
void pressure_init() {
  pinMode(Pins::PFET_EN, OUTPUT);
  digitalWrite(Pins::PFET_EN, LOW); //default to low-power safe state
}

//one-shot raw pressure sample using ADXL AUX ADC
//returns true on successful ADC read
bool pressure_read_raw_adc(uint16_t &adc_raw) {
  digitalWrite(Pins::PFET_EN, HIGH);
  delay(Timing::PRESSURE_SETTLE_MS);

  bool ok = adxl_read_adc(adc_raw);

  //always gate power back off after each sample attempt
  digitalWrite(Pins::PFET_EN, LOW);
  return ok;
}
