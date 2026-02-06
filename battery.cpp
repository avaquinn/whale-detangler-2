/*
  Battery fuel-gauge driver (I2C)
  - Reads battery state-of-charge (SOC) and voltage
  - Configures/handles ALERT behavior
  - Provides a compact battery snapshot for logging and UI/status updates
*/
#include <Arduino.h>
#include "battery.h"
#include "I2C.h"

//MAX17048 register addresses
static constexpr uint8_t REG_VCELL = 0x02; //ADC measurement of VCELL (R)
static constexpr uint8_t REG_SOC = 0x04; //battery state of charge (R)
static constexpr uint8_t REG_CONFIG = 0x0C; //configuration and alert indicators (R/W)
static constexpr uint8_t REG_VALRT = 0x14; //MSB=MIN, LSB=MAX; configs the VCELL range outside of which alerts are generated (R)
static constexpr uint8_t REG_VRESET = 0x18; //MSB=VRESET/Dis, LSB=ID; configs VCELL threshold below which the IC resets itself (R/W, ID is R)
static constexpr uint8_t REG_STATUS = 0x1A; //indicates status alerts

//0x6C for write, 0x6D for read; need to shift left and add bit to the right
static constexpr uint8_t GAUGE_ADDR = 0x36;

//ThresholdsTODO: choose final values
static constexpr uint8_t  SOC_LOW_THRESH_PERCENT = 10; // 1..32
static constexpr uint16_t VALRT_MIN_MV = 3300;
static constexpr uint16_t VALRT_MAX_MV = 4300;
static constexpr uint16_t VRESET_MV = 3000;

static constexpr uint8_t I2C_RETRIES = 2;

//Bit masks
//STATUS (0x1A) MSB contains: X EnVR SC HD VR VL VH RI    X...
//CONFIG (0x0C) LSB contains: SLEEP ALSC ALRT ATHD[4:0]
static constexpr uint16_t STATUS_RI = (1 << 8); //bit8 (MSB bit0), anytime the bit is set, the IC is not configured
static constexpr uint16_t STATUS_VH = (1 << 9); //bit9, set when VCELL has been above VALRT.MAX
static constexpr uint16_t STATUS_VL = (1 << 10); //bit10, set when VCELL has been below VALRT.MIN
static constexpr uint16_t STATUS_VR = (1 << 11); //bit11, set after the device has been reset if EnVr is set
static constexpr uint16_t STATUS_HD = (1 << 12); //bit12, set when SOC crosses the value in CONFIG.ATHD
static constexpr uint16_t STATUS_SC = (1 << 13); //bit13, set when SOC changes by at least 1% if CONFIG.ALSC is set
static constexpr uint16_t STATUS_ENVR = (1 << 14); //bit14, when set to 1, asserts the ALRT pin when a voltage-reset event occurs under the
                                                   //conditions described by the VRESET/ID register

static constexpr uint16_t CONFIG_ALSC = (1 << 6); //bit6, enables alerting when SOC changes by at least 1%
static constexpr uint16_t CONFIG_ALRT = (1 << 5); //bit5, set by the IC when an alert occurs (ALRT pins asserts low)
static constexpr uint16_t CONFIG_ATHD_MASK = 0x001F; //bits0..4 in LSB, sets the SOC threshold

static volatile bool alert_irq = false; //interrupt flag
static void battery_alert_isr() { //ISR: just record that the ALRT pin fired
  alert_irq = true;
}

//VCELL LSb = 78.125 uV = 0.078125 mV
static uint16_t vcell_raw_to_mv(uint16_t raw) {
  //mV = raw * 0.078125 = raw * 78125 / 1,000,000
  uint32_t mv = (uint32_t)raw * 78125UL;
  mv = (mv + 500000UL) / 1000000UL; //rounding
  if (mv > 0xFFFF) {
    mv = 0xFFFF;
  }
  return (uint16_t)mv;
}

//SOC LSb = 1%/256
static uint16_t soc_raw_to_x100(uint16_t raw) {
  //x100 = raw * 100 / 256
  uint32_t x100 = (uint32_t)raw * 100UL;
  x100 = (x100 + 128UL) / 256UL; // rounding
  if (x100 > 0xFFFF) {
    x100 = 0xFFFF;
  }
  return (uint16_t)x100;
}

//VALRT thresholds LSb = 20mV
static uint8_t mv_to_valrt(uint16_t mv) {
  //clamp to 8-bit range
  uint32_t code = (mv + 10) / 20; // rounding to nearest 20mV step
  if (code > 255) {
    code = 255;
  }
  return (uint8_t)code;
}

//VRESET threshold LSb = 40mV (bits[7:1])
static uint8_t mv_to_vreset(uint16_t mv) {
  //clamp to 7-bit range
  uint32_t code = (mv + 20) / 40; //rounding
  if (code > 127) {
    code = 127;
  }
  return (uint8_t)code;
}

static bool read_config(uint16_t &cfg, uint8_t retries) {
  return i2c_read(GAUGE_ADDR, REG_CONFIG, cfg, retries);
}
static bool write_config(uint16_t cfg, uint8_t retries) {
  return i2c_write(GAUGE_ADDR, REG_CONFIG, cfg, retries);
}
static bool read_status(uint16_t &st, uint8_t retries) {
  return i2c_read(GAUGE_ADDR, REG_STATUS, st, retries);
}
static bool write_status(uint16_t st, uint8_t retries) {
  return i2c_write(GAUGE_ADDR, REG_STATUS, st, retries);
}

//clear CONFIG.ALRT to deassert ALRT pin
static bool clear_config_alrt(uint8_t retries) {
  uint16_t cfg = 0;
  if (!read_config(cfg, retries)) { //read current config register
    return false;
  }

  //clear ALRT bit (bit5 of low byte)
  cfg &= ~CONFIG_ALRT;

  return write_config(cfg, retries);
}

//verify the gauge responds and configure + enable ALRT interrupt handling
//Output: true if we can read a register successfully.
bool battery_init() {
  uint16_t raw_v = 0;
  if (!i2c_read(GAUGE_ADDR, REG_VCELL, raw_v, I2C_RETRIES)) { //try reading VCELL
    return false;
  }

  //ALRT is active-low, enable internal pull-up
  pinMode(Pins::ALERT, INPUT_PULLUP);
  //trigger interrupt on falling edge (ALRT asserts low)
  attachInterrupt(digitalPinToInterrupt(Pins::ALERT), battery_alert_isr, FALLING);

  //configure alerts
  battery_setSocLowThresholdPercent(SOC_LOW_THRESH_PERCENT);
  battery_enableSocChangeAlert(false);
  battery_setVoltageAlertThresholdsMv(VALRT_MIN_MV, VALRT_MAX_MV);
  battery_setVresetThresholdMv(VRESET_MV);
  battery_enableVresetAlert(true);
  
  return clear_config_alrt(I2C_RETRIES); //clear ALRT so it starts deasserted
}

//read voltage in millivolts
//Output parameter: voltage in mV
bool battery_readVoltageMv(uint16_t &out_mv) {
  uint16_t raw = 0;
  if (!i2c_read(GAUGE_ADDR, REG_VCELL, raw, I2C_RETRIES)) {
    return false;
  }
  out_mv = vcell_raw_to_mv(raw);
  return true;
}

//read SOC in 0.01% units (x100)
//Output parameter: SOC * 100
bool battery_readSoc(uint16_t &out_soc) {
  uint16_t raw = 0;
  if (!i2c_read(GAUGE_ADDR, REG_SOC, raw, I2C_RETRIES)) {
    return false;
  }
  out_soc = soc_raw_to_x100(raw);
  return true;
}

//read both voltage + SOC into BatterySnapshot
bool battery_readSnapshot(BatterySnapshot &out) {
  uint16_t mv = 0;
  uint16_t soc = 0;

  if (!battery_readVoltageMv(mv)) {
    return false;
  }
  if (!battery_readSoc(soc)) {
    return false;
  }

  out.voltage_mv = mv;
  out.SOC = soc;
  return true;
}

//enable/disable 1% SOC-change alert (CONFIG.ALSC)
bool battery_enableSocChangeAlert(bool enable) {
  uint16_t cfg = 0;
  if (!read_config(cfg, I2C_RETRIES)) {
    return false;
  }

  //ALSC enables 1% SOC-change alert
  if (enable) {
    cfg |= CONFIG_ALSC;
  } else {
    cfg &= ~CONFIG_ALSC;
  }

  return write_config(cfg, I2C_RETRIES);
}

//set the low-SOC alert threshold in percent (1..32)
//mapping: threshold = (32 - ATHD)%.
//this alert triggers on falling edge across the threshold
bool battery_setSocLowThresholdPercent(uint8_t threshold_percent) {
  if (threshold_percent < 1 || threshold_percent > 32) {
    return false;
  }

  uint8_t athd = (uint8_t)(32 - threshold_percent) & 0x1F;

  uint16_t cfg = 0;
  if (!read_config(cfg, I2C_RETRIES)) {
    return false;
  }

  //preserve everything except ATHD[4:0]
  cfg = (uint16_t)((cfg & ~CONFIG_ATHD_MASK) | athd);

  return write_config(cfg, I2C_RETRIES);
}

//configure undervoltage/overvoltage thresholds using VALRT register
/*
Inputs:
  min_mv: triggers STATUS.VL if VCELL < min_mv
  max_mv: triggers STATUS.VH if VCELL > max_mv
Returns false if inputs are invalid.
*/
bool battery_setVoltageAlertThresholdsMv(uint16_t min_mv, uint16_t max_mv) {
  if (min_mv > max_mv) {
    return false;
  }

  //VALRT is split: MSB = MIN, LSB = MAX. 1 LSb = 20mV
  uint8_t min_code = mv_to_valrt(min_mv);
  uint8_t max_code = mv_to_valrt(max_mv);

  uint16_t valrt = (uint16_t(min_code) << 8) | uint16_t(max_code);
  return i2c_write(GAUGE_ADDR, REG_VALRT, valrt, I2C_RETRIES);
}

//configure VRESET threshold (VRESET register)
/*
Inputs:
  vreset_mv: 2280..3480 mV typical range per datasheet guidance (use your system choice)
*/
bool battery_setVresetThresholdMv(uint16_t vreset_mv) {
  //VRESET register: MSB contains VRESET[7:1] (40mV units) and Dis bit
  uint16_t vreg = 0;
  if (!i2c_read(GAUGE_ADDR, REG_VRESET, vreg, I2C_RETRIES)) {
    return false;
  }

  uint8_t msb = (uint8_t)((vreg >> 8) & 0xFF);
  uint8_t id = (uint8_t)(vreg & 0xFF); //read-only; write will be ignored

  uint8_t vcode = mv_to_vreset(vreset_mv) & 0x7F; //7 bits
  msb = (uint8_t)(vcode << 1); //place into bits[7:1]
  msb |= 0x01; //set Dis bit to disable analog comparator in hibernate mode

  uint16_t new_vreg = (uint16_t(msb) << 8) | uint16_t(id);
  return i2c_write(GAUGE_ADDR, REG_VRESET, new_vreg, I2C_RETRIES);
}

//enable/disable VRESET alert behavior (STATUS.EnVR)
bool battery_enableVresetAlert(bool enable) {
  //EnVR bit lives in STATUS MSB
  uint16_t st = 0;
  if (!read_status(st, I2C_RETRIES)) {
    return false;
  }

  if (enable) {
    st |= STATUS_ENVR;
  } else {
    st &= ~STATUS_ENVR;
  }

  return write_status(st, I2C_RETRIES);
}

//returns true if an ALRT interrupt was observed (flag set) and we successfully read STATUS
//Output: decoded alerts + raw register values, returns true if alert was successfully read and decoded
bool battery_pollAlerts(BatteryAlerts &out) {
  //clear output defaults
  out = {};

  //check if ISR flagged an interrupt
  bool irq = false;
  noInterrupts();
  irq = alert_irq;
  alert_irq = false; //consume it
  interrupts();

  if (!irq) {
    return false; //no new alert observed
  }

  //read STATUS to determine cause
  uint16_t st = 0;
  if (!read_status(st, I2C_RETRIES)) {
    return false;
  }

  //decode flags
  out.reset_indicator = (st & STATUS_RI) != 0;
  out.voltage_high = (st & STATUS_VH) != 0;
  out.voltage_low = (st & STATUS_VL) != 0;
  out.voltage_reset = (st & STATUS_VR) != 0;
  out.soc_low = (st & STATUS_HD) != 0;
  out.soc_change_1pct = (st & STATUS_SC) != 0;

  //service the alert
  //clear the corresponding STATUS bits after servicing
  uint16_t clear_mask = STATUS_RI | STATUS_VH | STATUS_VL | STATUS_VR | STATUS_HD | STATUS_SC;
  uint16_t new_status = (uint16_t)(st & ~clear_mask);

  //preserve EnVR as-is
  new_status = (uint16_t)((new_status & ~STATUS_ENVR) | (st & STATUS_ENVR));

  if (!write_status(new_status, I2C_RETRIES)) {
    return false;
  }

  //clear CONFIG.ALRT to deassert the ALRT pin
  if (!clear_config_alrt(I2C_RETRIES)) {
    return false;
  }

  return true;
}