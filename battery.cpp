/*
  Battery fuel-gauge driver (MAX17048, I2C)
  - Reads battery state-of-charge (SOC) and cell voltage
  - Configures and services ALRT. ALRT is level-based: the gauge holds it low until STATUS and
    CONFIG.ALRT are cleared, so we poll the pin level instead of catching an edge. An edge-triggered
    flag loses the alert forever if one service attempt fails (the line never goes high again).
  - NOTE: VCELL/SOC are only meaningful if the CELL pin (U2 pin 2) is wired to VBAT
*/
#include "battery.h"
#include "i2c_bus.h"

//MAX17048 register addresses
static constexpr uint8_t REG_VCELL = 0x02; //ADC measurement of VCELL (R)
static constexpr uint8_t REG_SOC = 0x04; //battery state of charge (R)
static constexpr uint8_t REG_CONFIG = 0x0C; //configuration and alert indicators (R/W)
static constexpr uint8_t REG_VALRT = 0x14; //MSB=MIN, LSB=MAX voltage alert window (R/W)
static constexpr uint8_t REG_VRESET = 0x18; //MSB=VRESET/Dis (R/W), LSB=ID (R)
static constexpr uint8_t REG_STATUS = 0x1A; //alert status (R/W)

static constexpr uint8_t GAUGE_ADDR = 0x36; //7-bit address

//thresholds TODO: choose final values
static constexpr uint8_t SOC_LOW_THRESH_PERCENT = 10; //1..32
static constexpr uint16_t VALRT_MIN_MV = 3300;
static constexpr uint16_t VALRT_MAX_MV = 4300;
static constexpr uint16_t VRESET_MV = 3000;

static constexpr uint8_t I2C_RETRIES = 2;

//STATUS (0x1A) MSB: X EnVR SC HD VR VL VH RI
static constexpr uint16_t STATUS_RI = (1 << 8); //set at power-up: the IC is not configured
static constexpr uint16_t STATUS_VH = (1 << 9); //VCELL has been above VALRT.MAX
static constexpr uint16_t STATUS_VL = (1 << 10); //VCELL has been below VALRT.MIN
static constexpr uint16_t STATUS_VR = (1 << 11); //voltage reset occurred (if EnVR)
static constexpr uint16_t STATUS_HD = (1 << 12); //SOC crossed CONFIG.ATHD
static constexpr uint16_t STATUS_SC = (1 << 13); //SOC changed by >= 1% (if ALSC)
static constexpr uint16_t STATUS_ENVR = (1 << 14); //assert ALRT on voltage reset
static constexpr uint16_t STATUS_ALERT_MASK = STATUS_RI | STATUS_VH | STATUS_VL | STATUS_VR | STATUS_HD | STATUS_SC;

//CONFIG (0x0C) LSB: SLEEP ALSC ALRT ATHD[4:0]
static constexpr uint16_t CONFIG_ALSC = (1 << 6); //alert on 1% SOC change
static constexpr uint16_t CONFIG_ALRT = (1 << 5); //set by the IC when an alert occurs
static constexpr uint16_t CONFIG_ATHD_MASK = 0x001F; //empty-alert threshold, (32 - ATHD)%

static uint32_t g_last_alert_attempt_ms = 0;
static bool g_alert_attempted = false;

//VCELL LSb = 78.125 uV = 5/64 mV
static uint16_t vcell_raw_to_mv(uint16_t raw) {
  return (uint16_t)(((uint32_t)raw * 5UL + 32UL) / 64UL);
}

//SOC LSb = 1%/256
static uint16_t soc_raw_to_x100(uint16_t raw) {
  return (uint16_t)(((uint32_t)raw * 100UL + 128UL) / 256UL);
}

//VALRT thresholds LSb = 20 mV
static uint8_t mv_to_valrt(uint16_t mv) {
  uint32_t code = (mv + 10UL) / 20UL;
  return (uint8_t)(code > 255 ? 255 : code);
}

//VRESET threshold LSb = 40 mV, bits[7:1]
static uint8_t mv_to_vreset(uint16_t mv) {
  uint32_t code = (mv + 20UL) / 40UL;
  return (uint8_t)(code > 127 ? 127 : code);
}

static bool read_reg(uint8_t reg, uint16_t &v) {
  return i2c_read_u16be(GAUGE_ADDR, reg, v, I2C_RETRIES);
}

static bool write_reg(uint8_t reg, uint16_t v) {
  return i2c_write_u16be(GAUGE_ADDR, reg, v, I2C_RETRIES);
}

//read-modify-write: clear 'clear_mask' then set 'set_bits'
static bool modify_reg(uint8_t reg, uint16_t clear_mask, uint16_t set_bits) {
  uint16_t v = 0;
  if (!read_reg(reg, v)) return false;
  return write_reg(reg, (uint16_t)((v & ~clear_mask) | set_bits));
}

//CONFIG: empty-alert threshold, 1% change alert, and clear any pending ALRT
static bool configure_config() {
  uint8_t athd = (uint8_t)(32 - SOC_LOW_THRESH_PERCENT) & 0x1F;
  return modify_reg(REG_CONFIG, CONFIG_ATHD_MASK | CONFIG_ALRT, CONFIG_ALSC | athd);
}

//VRESET: reset threshold in bits[7:1], Dis (bit0) = 1 disables the comparator in hibernate
static bool configure_vreset() {
  uint16_t v = 0;
  if (!read_reg(REG_VRESET, v)) return false;
  uint8_t msb = (uint8_t)(((mv_to_vreset(VRESET_MV) & 0x7F) << 1) | 0x01);
  return write_reg(REG_VRESET, (uint16_t)((uint16_t(msb) << 8) | (v & 0x00FF))); //ID byte is read-only
}

bool battery_init() {
  pinMode(Pins::ALERT, INPUT_PULLUP); //ALRT is open-drain; add an external pull-up on Rev 1.1

  uint16_t probe = 0;
  if (!read_reg(REG_VCELL, probe)) {
    return false;
  }

  bool ok = true;
  ok &= configure_config();
  ok &= write_reg(REG_VALRT, (uint16_t)((uint16_t(mv_to_valrt(VALRT_MIN_MV)) << 8) | mv_to_valrt(VALRT_MAX_MV)));
  ok &= configure_vreset();
  //enable voltage-reset alert and clear every latched alert flag, including RI ("configured now")
  ok &= modify_reg(REG_STATUS, STATUS_ALERT_MASK, STATUS_ENVR);
  return ok;
}

//read both voltage + SOC into BatterySnapshot
bool battery_readSnapshot(BatterySnapshot &out) {
  uint16_t vraw = 0, sraw = 0;
  if (!read_reg(REG_VCELL, vraw) || !read_reg(REG_SOC, sraw)) {
    return false;
  }
  out.voltage_mv = vcell_raw_to_mv(vraw);
  out.soc_x100 = soc_raw_to_x100(sraw);
  return true;
}

//service ALRT if it is asserted
//returns true when an alert was read, decoded, and cleared; false if none pending or service failed
//(a failed service leaves ALRT low, so it is retried every BATTERY_ALERT_RETRY_MS)
bool battery_pollAlerts(uint32_t now_ms, BatteryAlerts &out) {
  out = {};
  if (digitalRead(Pins::ALERT) == HIGH) {
    return false;
  }
  if (g_alert_attempted && (uint32_t)(now_ms - g_last_alert_attempt_ms) < Timing::BATTERY_ALERT_RETRY_MS) {
    return false;
  }
  g_alert_attempted = true;
  g_last_alert_attempt_ms = now_ms;

  uint16_t st = 0;
  if (!read_reg(REG_STATUS, st)) {
    return false;
  }

  out.reset_indicator = (st & STATUS_RI) != 0;
  out.voltage_high = (st & STATUS_VH) != 0;
  out.voltage_low = (st & STATUS_VL) != 0;
  out.voltage_reset = (st & STATUS_VR) != 0;
  out.soc_low = (st & STATUS_HD) != 0;
  out.soc_change_1pct = (st & STATUS_SC) != 0;

  //a reset means the gauge lost its configuration; restore it before clearing flags
  if (out.reset_indicator && !(configure_config() && configure_vreset())) {
    return false;
  }

  //clear the serviced STATUS bits (EnVR kept), then CONFIG.ALRT to release the pin
  if (!write_reg(REG_STATUS, (uint16_t)((st & ~STATUS_ALERT_MASK) | STATUS_ENVR))) {
    return false;
  }
  return modify_reg(REG_CONFIG, CONFIG_ALRT, 0);
}

uint8_t battery_alertBits(const BatteryAlerts &a) {
  return (uint8_t)((a.reset_indicator << 0) | (a.voltage_high << 1) | (a.voltage_low << 2) |
                   (a.voltage_reset << 3) | (a.soc_low << 4) | (a.soc_change_1pct << 5));
}
