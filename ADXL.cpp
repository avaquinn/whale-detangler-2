/*
  Accelerometer driver (ADXL363, SPI)
  - Configures range/ODR/activity detection and reads X/Y/Z, temperature, and the auxiliary ADC
  - The auxiliary ADC digitizes the pressure/strain signal on the Rev 1.0 board
  - Activity interrupts are polled, not edge-triggered: in the default (non-linked) mode the chip
    holds INT1/INT2 high until STATUS is read, so polling the pin level can never miss an event
*/
#include "ADXL.h"
#include "config.h"
#include "spi_bus.h"

//ADXL363 SPI command bytes
static constexpr uint8_t CMD_WRITE_REG = 0x0A;
static constexpr uint8_t CMD_READ_REG = 0x0B;

//register addresses
static constexpr uint8_t REG_DEVID_AD = 0x00;
static constexpr uint8_t REG_DEVID_MST = 0x01;
static constexpr uint8_t REG_PARTID = 0x02;
static constexpr uint8_t REG_STATUS = 0x0B;
static constexpr uint8_t REG_XDATA_L = 0x0E; //X/Y/Z, 6 bytes
static constexpr uint8_t REG_TEMP_L = 0x14; //temperature, 2 bytes
static constexpr uint8_t REG_ADC_DATA_L = 0x16; //aux ADC, 2 bytes
static constexpr uint8_t REG_SOFT_RESET = 0x1F; //write 0x52 to reset
static constexpr uint8_t REG_THRESH_ACT_L = 0x20;
static constexpr uint8_t REG_THRESH_ACT_H = 0x21;
static constexpr uint8_t REG_TIME_ACT = 0x22;
static constexpr uint8_t REG_ACT_INACT_CTL = 0x27;
static constexpr uint8_t REG_INTMAP1 = 0x2A;
static constexpr uint8_t REG_INTMAP2 = 0x2B;
static constexpr uint8_t REG_FILTER_CTL = 0x2C;
static constexpr uint8_t REG_POWER_CTL = 0x2D;

//expected IDs
static constexpr uint8_t EXPECTED_DEVID_AD = 0xAD;
static constexpr uint8_t EXPECTED_DEVID_MST = 0x1D;
static constexpr uint8_t EXPECTED_PARTID = 0xF3;

static constexpr uint8_t SOFT_RESET_KEY = 0x52;

//POWER_CTL bits
static constexpr uint8_t POWER_CTL_MEASURE = (1 << 1); //MEASURE[1:0] = 10: measurement mode
static constexpr uint8_t POWER_CTL_ADC_EN = (1 << 7); //convert the ADC_IN pin every ODR period

//ACT_INACT_CTL bits
static constexpr uint8_t ACT_EN = (1 << 0);
static constexpr uint8_t ACT_REF = (1 << 1); //referenced: compare against a captured reference, not 0 g,
                                             //otherwise gravity alone exceeds the threshold

//STATUS / INTMAP activity bit
static constexpr uint8_t ACT_BIT = (1 << 4);

//FILTER_CTL: RANGE[7:6] (±2g: 0, ±4g: 1, ±8g: 2), ODR[2:0] (12.5 Hz: 000 ... 100 Hz: 011 ... 400 Hz: 101)
static constexpr uint8_t RANGE = 0b10; //±8 g, matches Accel::MG_PER_LSB = 4
static constexpr uint8_t ODR = 0b011; //100 Hz, matches Pressure::ADXL_ODR_PERIOD_MS = 10

//helpers
static uint8_t reg_read_byte(uint8_t reg) {
  spi_select(Pins::ACCEL_CS, SPI_MODE0);
  spi_txrx(CMD_READ_REG);
  spi_txrx(reg);
  uint8_t v = spi_txrx(0x00);
  spi_deselect();
  return v;
}

static void reg_write_byte(uint8_t reg, uint8_t val) {
  spi_select(Pins::ACCEL_CS, SPI_MODE0);
  spi_txrx(CMD_WRITE_REG);
  spi_txrx(reg);
  spi_txrx(val);
  spi_deselect();
}

static void reg_read_burst(uint8_t reg, uint8_t *buf, size_t n) {
  spi_select(Pins::ACCEL_CS, SPI_MODE0);
  spi_txrx(CMD_READ_REG);
  spi_txrx(reg);
  spi_transfer(NULL, buf, n);
  spi_deselect();
}

//little-endian, already sign-extended by the chip
static int16_t le16(const uint8_t *b) {
  return (int16_t)((uint16_t(b[1]) << 8) | b[0]);
}

static bool verify_ids() {
  return reg_read_byte(REG_DEVID_AD) == EXPECTED_DEVID_AD &&
         reg_read_byte(REG_DEVID_MST) == EXPECTED_DEVID_MST &&
         reg_read_byte(REG_PARTID) == EXPECTED_PARTID;
}

//(re)enable referenced activity detection; re-enabling re-captures the reference at the current
//orientation so a pot lying at a new angle does not look like continuous activity
static void arm_activity() {
  reg_write_byte(REG_ACT_INACT_CTL, 0);
  reg_write_byte(REG_ACT_INACT_CTL, ACT_EN | ACT_REF);
}

bool adxl_init() {
  pinMode(Pins::ACCEL_INT1, INPUT);
  pinMode(Pins::ACCEL_INT2, INPUT);
  spi_config_cs(Pins::ACCEL_CS);

  //soft reset for a clean known state
  reg_write_byte(REG_SOFT_RESET, SOFT_RESET_KEY);
  delay(10);

  if (!verify_ids()) {
    return false;
  }

  reg_write_byte(REG_FILTER_CTL, (uint8_t)(((RANGE & 0x03) << 6) | (ODR & 0x07)));

  //activity detection (threshold is 11 bits in LSB of the selected range)
  reg_write_byte(REG_THRESH_ACT_L, (uint8_t)(Accel::ACTIVITY_THRESHOLD & 0xFF));
  reg_write_byte(REG_THRESH_ACT_H, (uint8_t)((Accel::ACTIVITY_THRESHOLD >> 8) & 0x07));
  reg_write_byte(REG_TIME_ACT, Accel::ACTIVITY_TIME);
  //INT1 is reserved as a future wake-from-sleep source; INT2 drives the triple-tap UI.
  //both carry the same activity event and are acknowledged together by reading STATUS
  reg_write_byte(REG_INTMAP1, ACT_BIT);
  reg_write_byte(REG_INTMAP2, ACT_BIT);

  reg_write_byte(REG_POWER_CTL, POWER_CTL_ADC_EN | POWER_CTL_MEASURE);
  arm_activity();
  (void)reg_read_byte(REG_STATUS); //clear anything latched during configuration

  //confirm the configuration stuck (catches a MISO/CS wiring fault that still returned good IDs)
  return reg_read_byte(REG_POWER_CTL) == (POWER_CTL_ADC_EN | POWER_CTL_MEASURE);
}

bool adxl_read_xyz(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t buf[6];
  reg_read_burst(REG_XDATA_L, buf, sizeof(buf));
  x = le16(&buf[0]);
  y = le16(&buf[2]);
  z = le16(&buf[4]);
  return true;
}

bool adxl_read_temp(int16_t &raw) {
  uint8_t buf[2];
  reg_read_burst(REG_TEMP_L, buf, sizeof(buf));
  raw = le16(buf);
  return true;
}

//aux ADC is twos complement, sign extended; it must NOT be masked to 12 bits
//(masking turned every reading below mid-scale into a large positive number)
bool adxl_read_adc(int16_t &raw) {
  uint8_t buf[2];
  reg_read_burst(REG_ADC_DATA_L, buf, sizeof(buf));
  raw = le16(buf);
  return true;
}

bool adxl_poll_activity() {
  if (digitalRead(Pins::ACCEL_INT1) == LOW && digitalRead(Pins::ACCEL_INT2) == LOW) {
    return false; //nothing latched
  }
  uint8_t status = reg_read_byte(REG_STATUS); //reading STATUS acknowledges the interrupt
  if (status & ACT_BIT) {
    arm_activity();
    return true;
  }
  return false;
}

//integer square root (floor) for up to 32-bit inputs
static uint16_t isqrt32(uint32_t v) {
  uint32_t res = 0;
  uint32_t bit = 1UL << 30;
  while (bit > v) bit >>= 2;
  while (bit != 0) {
    if (v >= res + bit) {
      v -= res + bit;
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }
  return (uint16_t)res;
}

uint16_t adxl_magnitude_mg(int16_t x, int16_t y, int16_t z) {
  uint32_t sq = (uint32_t)((int32_t)x * x) + (uint32_t)((int32_t)y * y) + (uint32_t)((int32_t)z * z);
  uint32_t mg = (uint32_t)isqrt32(sq) * Accel::MG_PER_LSB;
  return (uint16_t)(mg > 0xFFFF ? 0xFFFF : mg);
}

//gravity baseline: a slow, time-based average of |a|. Comparing against it (instead of a fixed
//1000 mg) makes motion immune to per-part scale/offset error and to temperature drift.
static float g_gravity_mg = 0;
static uint32_t g_gravity_t_ms = 0;
static bool g_gravity_init = false;

uint16_t adxl_motion_mg(int16_t x, int16_t y, int16_t z, uint32_t t_ms) {
  const float mag = adxl_magnitude_mg(x, y, z);
  if (!g_gravity_init) {
    g_gravity_mg = mag;
    g_gravity_init = true;
  } else {
    float k = (float)(t_ms - g_gravity_t_ms) / (float)Accel::GRAVITY_TAU_MS;
    if (k > 1.0f) k = 1.0f;
    g_gravity_mg += (mag - g_gravity_mg) * k;
  }
  g_gravity_t_ms = t_ms;

  float dyn = mag - g_gravity_mg;
  if (dyn < 0) dyn = -dyn;
  return (uint16_t)(dyn > 65535.0f ? 65535 : dyn + 0.5f);
}
