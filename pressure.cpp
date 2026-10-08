/*
  Pressure/strain sensing
  - Rev 1.0 (PRESSURE_FE_ADXL_AUX): quarter bridge -> INA333 (G ~ 403) -> ADXL363 aux ADC.
    The bridge is powered through P-FET Q1, which is ON when PFET_EN is LOW.
  - Rev 1.1 (PRESSURE_FE_NAU7802): half/full bridge excited from the NAU7802 AVDD LDO and measured
    ratiometrically by its 24-bit ADC; powering the NAU7802 down also removes bridge current.
  - Converts raw counts to depth with a two-point linear calibration stored in FRAM
*/
#include "pressure.h"
#include "config.h"

static PressureCal g_cal = {0, 0.0f, 0};
static void (*g_idle_hook)() = nullptr;
static bool g_enabled = true;

void pressure_setEnabled(bool enabled) {
  g_enabled = enabled;
}

void pressure_setIdleHook(void (*hook)()) {
  g_idle_hook = hook;
}

//like delay(), but lets other work (the standalone recorder) run while we wait
static void wait_ms(uint16_t ms) {
  const uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ms) {
    if (g_idle_hook) g_idle_hook();
  }
}

void pressure_setCal(const PressureCal &cal) {
  g_cal = cal;
}

const PressureCal &pressure_getCal() {
  return g_cal;
}

bool pressure_toDepthCm(int32_t raw, int16_t &depth_cm) {
  if (!g_cal.valid) {
    return false;
  }
  float cm = (float)(raw - g_cal.zero_raw) * g_cal.cm_per_count;
  if (cm > 32767.0f) cm = 32767.0f;
  if (cm < -32768.0f) cm = -32768.0f;
  depth_cm = (int16_t)(cm >= 0 ? cm + 0.5f : cm - 0.5f);
  return true;
}

#if PRESSURE_FRONTEND == PRESSURE_FE_ADXL_AUX
//---------------------------------------------------------------------------------------------
//Rev 1.0: ADXL363 auxiliary ADC
//---------------------------------------------------------------------------------------------
#include "ADXL.h"

static void bridge_power(bool on) {
  digitalWrite(Pins::PFET_EN, on ? Pins::PFET_ON : Pins::PFET_OFF);
}

bool pressure_init() {
  digitalWrite(Pins::PFET_EN, Pins::PFET_OFF); //level first, then output: no power glitch
  pinMode(Pins::PFET_EN, OUTPUT);
  return true; //the ADC lives in the ADXL363, which adxl_init() verifies
}

bool pressure_calibrate_afe() {
  return true;
}

//raw = SUM of ADXL_AVG_SAMPLES fresh conversions (keeps the averaging's extra resolution)
PressureResult pressure_read_raw(int32_t &raw) {
  if (!g_enabled) return PressureResult::DISABLED;
  bridge_power(true);
  wait_ms(Pressure::ADXL_SETTLE_MS);

  int32_t sum = 0;
  bool clipped = false;
  for (uint8_t i = 0; i < Pressure::ADXL_AVG_SAMPLES; i++) {
    if (i > 0) wait_ms(Pressure::ADXL_ODR_PERIOD_MS); //wait for a new conversion
    int16_t v = 0;
    if (!adxl_read_adc(v)) {
      bridge_power(false);
      return PressureResult::FAILED;
    }
    if (v >= Pressure::ADXL_CLIP_COUNTS || v <= -Pressure::ADXL_CLIP_COUNTS) {
      clipped = true;
    }
    sum += v;
  }

  bridge_power(false); //always gate power back off
  raw = sum;
  return clipped ? PressureResult::CLIPPED : PressureResult::OK;
}

#elif PRESSURE_FRONTEND == PRESSURE_FE_NAU7802
//---------------------------------------------------------------------------------------------
//Rev 1.1: NAU7802 24-bit bridge ADC on the shared I2C bus
//---------------------------------------------------------------------------------------------
#include "i2c_bus.h"

//registers
static constexpr uint8_t NAU_PU_CTRL = 0x00;
static constexpr uint8_t NAU_CTRL1 = 0x01;
static constexpr uint8_t NAU_CTRL2 = 0x02;
static constexpr uint8_t NAU_ADCO_B2 = 0x12;
static constexpr uint8_t NAU_ADC = 0x15;
static constexpr uint8_t NAU_PGA = 0x1B;
static constexpr uint8_t NAU_PGA_PWR = 0x1C;
static constexpr uint8_t NAU_REVISION = 0x1F;

//PU_CTRL bits
static constexpr uint8_t PU_RR = (1 << 0); //register reset
static constexpr uint8_t PU_PUD = (1 << 1); //power up digital
static constexpr uint8_t PU_PUA = (1 << 2); //power up analog (also the AVDD LDO / bridge excitation)
static constexpr uint8_t PU_PUR = (1 << 3); //power-up ready (R)
static constexpr uint8_t PU_CS = (1 << 4); //cycle start
static constexpr uint8_t PU_CR = (1 << 5); //conversion ready (R)
static constexpr uint8_t PU_AVDDS = (1 << 7); //AVDD from the internal LDO

//CTRL2 bits
static constexpr uint8_t CTRL2_CALS = (1 << 2); //start calibration / busy
static constexpr uint8_t CTRL2_CAL_ERR = (1 << 3);

static constexpr uint8_t PGA_LDOMODE = (1 << 6);
static constexpr uint8_t PGA_PWR_CAP_EN = (1 << 7); //PGA output bypass cap across VIN2P/VIN2N
static constexpr uint8_t ADC_CHP_OFF = 0x30; //REG_CHPS = 11: clock chopper off (datasheet power-on note)

static bool nau_read(uint8_t reg, uint8_t &v) {
  return i2c_read_u8(Pressure::NAU_ADDR, reg, v);
}

static bool nau_write(uint8_t reg, uint8_t v) {
  return i2c_write_u8(Pressure::NAU_ADDR, reg, v);
}

static bool nau_modify(uint8_t reg, uint8_t clear_mask, uint8_t set_bits) {
  uint8_t v = 0;
  if (!nau_read(reg, v)) return false;
  return nau_write(reg, (uint8_t)((v & ~clear_mask) | set_bits));
}

//wait for a register bit to reach a level
static bool nau_wait(uint8_t reg, uint8_t mask, bool set, uint16_t timeout_ms) {
  uint32_t start = millis();
  for (;;) {
    uint8_t v = 0;
    if (!nau_read(reg, v)) return false;
    if (((v & mask) != 0) == set) return true;
    if ((uint32_t)(millis() - start) >= timeout_ms) return false;
    wait_ms(1);
  }
}

static bool nau_power_up() {
  if (!nau_modify(NAU_PU_CTRL, 0, PU_PUD | PU_PUA)) return false;
  if (!nau_wait(NAU_PU_CTRL, PU_PUR, true, 10)) return false;
  return nau_modify(NAU_PU_CTRL, 0, PU_CS);
}

static void nau_power_down() {
  (void)nau_modify(NAU_PU_CTRL, PU_PUD | PU_PUA, 0); //<1 uA, bridge unpowered
}

static bool nau_read_conversion(int32_t &value) {
  if (!nau_wait(NAU_PU_CTRL, PU_CR, true, 50)) return false; //80 SPS = 12.5 ms per conversion
  uint8_t b[3];
  if (!i2c_read_bytes(Pressure::NAU_ADDR, NAU_ADCO_B2, b, 3)) return false;
  uint32_t u = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
  if (u & 0x800000UL) u |= 0xFF000000UL; //sign-extend 24 -> 32 bits
  value = (int32_t)u;
  return true;
}

bool pressure_calibrate_afe() {
  if (!nau_power_up()) return false;
  delay(Pressure::NAU_CAL_SETTLE_MS);
  bool ok = nau_modify(NAU_CTRL2, 0x03, 0) && //CALMOD = 00: internal offset calibration
            nau_modify(NAU_CTRL2, 0, CTRL2_CALS) &&
            nau_wait(NAU_CTRL2, CTRL2_CALS, false, 1000);
  uint8_t ctrl2 = 0;
  ok = ok && nau_read(NAU_CTRL2, ctrl2) && !(ctrl2 & CTRL2_CAL_ERR);
  nau_power_down();
  return ok;
}

bool pressure_init() {
  uint8_t rev = 0;
  if (!nau_read(NAU_REVISION, rev) || (rev & 0x0F) != 0x0F) {
    return false; //not present / wrong device
  }

  bool ok = nau_write(NAU_PU_CTRL, PU_RR);
  delay(1);
  ok = ok && nau_write(NAU_PU_CTRL, 0) && nau_power_up();
  ok = ok && nau_write(NAU_CTRL1, (uint8_t)((Pressure::NAU_LDO_CODE << 3) | Pressure::NAU_GAIN_CODE));
  ok = ok && nau_modify(NAU_PU_CTRL, 0, PU_AVDDS);
  ok = ok && nau_modify(NAU_CTRL2, 0x70, (uint8_t)(Pressure::NAU_RATE_CODE << 4)); //CRS[6:4], channel 1
  ok = ok && nau_modify(NAU_ADC, 0, ADC_CHP_OFF);
  ok = ok && nau_modify(NAU_PGA_PWR, 0, PGA_PWR_CAP_EN);
  ok = ok && nau_modify(NAU_PGA, PGA_LDOMODE, 0);
  nau_power_down();

  return ok && pressure_calibrate_afe();
}

//raw = AVERAGE of NAU_AVG_SAMPLES conversions after the power-up discard
PressureResult pressure_read_raw(int32_t &raw) {
  if (!g_enabled) return PressureResult::DISABLED;
  if (!nau_power_up()) {
    nau_power_down();
    return PressureResult::FAILED;
  }
  wait_ms(Pressure::NAU_SETTLE_MS);

  int32_t v = 0;
  for (uint8_t i = 0; i < Pressure::NAU_DISCARD; i++) {
    if (!nau_read_conversion(v)) {
      nau_power_down();
      return PressureResult::FAILED;
    }
  }

  int32_t sum = 0;
  bool clipped = false;
  for (uint8_t i = 0; i < Pressure::NAU_AVG_SAMPLES; i++) {
    if (!nau_read_conversion(v)) {
      nau_power_down();
      return PressureResult::FAILED;
    }
    if (v >= Pressure::NAU_CLIP_COUNTS || v <= -Pressure::NAU_CLIP_COUNTS) {
      clipped = true;
    }
    sum += v;
  }

  nau_power_down();
  raw = sum / Pressure::NAU_AVG_SAMPLES;
  return clipped ? PressureResult::CLIPPED : PressureResult::OK;
}

#else
#error "Unknown PRESSURE_FRONTEND"
#endif
