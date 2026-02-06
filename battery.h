#ifndef BATTERY_H
#define BATTERY_H

#include "types.h"

//alert result structure
struct BatteryAlerts {
  //decoded alert sources (true if that condition triggered ALRT)
  bool reset_indicator; //STATUS.RI (device powered up / reset)
  bool voltage_high; //STATUS.VH (VCELL > VALRT.MAX)
  bool voltage_low; //STATUS.VL (VCELL < VALRT.MIN)
  bool voltage_reset; //STATUS.VR (VRESET event, if enabled)
  bool soc_low; //STATUS.HD (SOC crossed CONFIG.ATHD)
  bool soc_change_1pct; //STATUS.SC (SOC changed by >= 1%, if ALSC enabled)
};

bool battery_init();
bool battery_readVoltageMv(uint16_t &out_mv);
bool battery_readSoc(uint16_t &out_soc);
bool battery_readSnapshot(BatterySnapshot &out);

bool battery_enableSocChangeAlert(bool enable);
bool battery_setSocLowThresholdPercent(uint8_t threshold_percent);
bool battery_setVoltageAlertThresholdsMv(uint16_t min_mv, uint16_t max_mv);
bool battery_setVresetThresholdMv(uint16_t vreset_mv);
bool battery_enableVresetAlert(bool enable);

bool battery_pollAlerts(BatteryAlerts &out);

#endif