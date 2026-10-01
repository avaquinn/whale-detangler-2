#ifndef PRESSURE_H
#define PRESSURE_H

#include "types.h"

enum class PressureResult : uint8_t {
  OK = 0,
  CLIPPED, //front end saturated (bridge imbalance / overrange); value is not trustworthy
  FAILED //bus/device error
};

bool pressure_init(); //configure the selected front end; false if it does not respond
PressureResult pressure_read_raw(int32_t &raw); //one power-gated, averaged reading
bool pressure_calibrate_afe(); //NAU7802 internal offset calibration (no-op for the ADXL path)

void pressure_setCal(const PressureCal &cal);
const PressureCal &pressure_getCal();
bool pressure_toDepthCm(int32_t raw, int16_t &depth_cm); //false if uncalibrated

#endif
