/*
  Pressure/strain sensing module built around the analog front end + ADXL363 auxiliary ADC
  - Controls the PFET power gate for the INA333/bridge circuitry to save power
  - Applies settle timing, then reads raw ADC via the ADXL363
  - Optionally applies calibration/scaling to convert raw readings to engineering units
*/
