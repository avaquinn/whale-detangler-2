/*
  Accelerometer driver
  - Configures the accelerometer (range/ODR/filters) and reads X/Y/Z acceleration
  - Supports interrupt configuration (INT1/INT2) for wake/motion events
  - Reads the ADXL363 auxiliary ADC channel used to digitize the pressure/strain signal path
*/
#include <Arduino.h>
#include "ADXL.h"
#include "SPI.h"

//ADXL363 SPI command bytes
static constexpr uint8_t CMD_WRITE_REG = 0x0A;
static constexpr uint8_t CMD_READ_REG = 0x0B;

//register addresses
static constexpr uint8_t REG_DEVID_AD = 0x00;
static constexpr uint8_t REG_DEVID_MST = 0x01;
static constexpr uint8_t REG_DEVID = 0x02;
static constexpr uint8_t REG_STATUS = 0x0B;

//x/y/z data start; read 6 bytes starting here
static constexpr uint8_t REG_XDATA_L = 0x0E;

//AUX ADC registers
static constexpr uint8_t REG_ADC_DATA_L = 0x16;
static constexpr uint8_t REG_THRESH_ACT_L = 0x20;
static constexpr uint8_t REG_THRESH_ACT_H = 0x21;
static constexpr uint8_t REG_TIME_ACT = 0x22;
static constexpr uint8_t REG_ACT_INACT_CTL = 0x27;
static constexpr uint8_t REG_INTMAP1 = 0x2A;
static constexpr uint8_t REG_INTMAP2 = 0x2B;
static constexpr uint8_t REG_SOFT_RESET = 0x1F; //write 0x52 to reset the ADXL
static constexpr uint8_t REG_FILTER_CTL = 0x2C;
static constexpr uint8_t REG_POWER_CTL = 0x2D;

//expected IDs
static constexpr uint8_t EXPECTED_DEVID_AD = 0xAD;
static constexpr uint8_t EXPECTED_DEVID_MST = 0x1D;
static constexpr uint8_t EXPECTED_DEVID = 0xF3;

//POWER_CTL bits
static constexpr uint8_t POWER_CTL_MEASURE = (1 << 1); //enables measurement mode
static constexpr uint8_t POWER_CTL_ADC_EN  = (1 << 7); //enables AUX ADC conversions TODO

//ACT_INACT_CTL bits
static constexpr uint8_t ACT_INACT_CTL_ACT_EN = (1 << 0);

//INTMAP bits (ADXL activity/inactivity interrupt map)
static constexpr uint8_t INTMAP_ACT = (1 << 4);

//bits for measurement mode; ±2g: 0, ±4g: 1, ±8g: 2
//TODO: check if max of peaks to determine if we can decrease for better resolution
static constexpr uint8_t RANGE = 0b10; //±8g

//output data rate; 12.5 Hz: 000, 25 Hz: 001, 50 Hz: 010, 100 Hz: 011, 200 Hz: 100, 400 Hz: 101...111
//TODO: more Hz = more power draw, if missing peaks or there are sharper shocks, bump to higher
static constexpr uint8_t ODR = 0b011; //100 Hz

//Activity detect placeholders for interrupt bring-up.
//These should be tuned on real motion data.
static constexpr uint16_t ACTIVITY_THRESHOLD = 140;
static constexpr uint8_t ACTIVITY_TIME = 8;

//helpers
//read a byte from reg to out
static bool reg_read_byte(uint8_t reg, uint8_t &out) {
  spi_begin(Pins::ACCEL_CS, 0); //SPI mode 0

  //command + address, then clock out the result by sending a dummy byte
  spi_txrx(CMD_READ_REG);
  spi_txrx(reg);
  out = spi_txrx(0x00);

  spi_end(Pins::ACCEL_CS);
  return true;
}

//write val byte to reg
static bool reg_write_byte(uint8_t reg, uint8_t val) {
  spi_begin(Pins::ACCEL_CS, 0); //SPI mode 0

  spi_txrx(CMD_WRITE_REG);
  spi_txrx(reg);
  spi_txrx(val);

  spi_end(Pins::ACCEL_CS);
  return true;
}

//burst read N bytes into buf, starting at reg
static bool reg_read_burst(uint8_t reg, uint8_t *buf, size_t n) {
  spi_begin(Pins::ACCEL_CS, 0); //SPI mode 0

  spi_txrx(CMD_READ_REG);
  spi_txrx(reg);

  for (size_t i = 0; i < n; i++) {
    buf[i] = spi_txrx(0x00);
  }

  spi_end(Pins::ACCEL_CS);
  return true;
}

//verify device IDs
static bool verify_ids() {
  uint8_t v = 0;
  if (!reg_read_byte(REG_DEVID_AD, v) || v != EXPECTED_DEVID_AD) return false;
  if (!reg_read_byte(REG_DEVID_MST, v) || v != EXPECTED_DEVID_MST) return false;
  if (!reg_read_byte(REG_DEVID, v) || v != EXPECTED_DEVID) return false;
  return true;
}

//initialize SPI and configure ADXL registers to start measuring
bool adxl_init() {
  //interrupt inputs
  pinMode(Pins::ACCEL_INT1, INPUT);
  pinMode(Pins::ACCEL_INT2, INPUT);

  spi_config_cs(Pins::ACCEL_CS); //configure CS pin

  //soft reset for a clean known state
  reg_write_byte(REG_SOFT_RESET, 0x52);
  delay(10);

  if (!verify_ids()) { //confirm IDs
    return false;
  }

  //configure FILTER_CTL
  uint8_t filter = 0;
  filter |= (RANGE & 0x03) << 6;
  filter |= (ODR & 0x07);
  reg_write_byte(REG_FILTER_CTL, filter);

  //configure POWER_CTL
  uint8_t power = 0;
  power |= POWER_CTL_ADC_EN;
  power |= POWER_CTL_MEASURE;
  reg_write_byte(REG_POWER_CTL, power);

  //configure activity interrupt generation:
  //INT1 -> motion detection path
  //INT2 -> second activity pulse path (used by firmware triple-tap counter)
  reg_write_byte(REG_THRESH_ACT_L, (uint8_t)(ACTIVITY_THRESHOLD & 0xFF));
  reg_write_byte(REG_THRESH_ACT_H, (uint8_t)((ACTIVITY_THRESHOLD >> 8) & 0x07));
  reg_write_byte(REG_TIME_ACT, ACTIVITY_TIME);
  reg_write_byte(REG_ACT_INACT_CTL, ACT_INACT_CTL_ACT_EN);
  reg_write_byte(REG_INTMAP1, INTMAP_ACT);
  reg_write_byte(REG_INTMAP2, INTMAP_ACT);

  return true;
}

//read raw acceleration samples
bool adxl_read_xyz(int16_t &x, int16_t &y, int16_t &z) {
  //read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  uint8_t buf[6] = {0};

  if (!reg_read_burst(REG_XDATA_L, buf, sizeof(buf))) {
    return false;
  }

  x = (int16_t)((uint16_t(buf[1]) << 8) | uint16_t(buf[0]));
  y = (int16_t)((uint16_t(buf[3]) << 8) | uint16_t(buf[2]));
  z = (int16_t)((uint16_t(buf[5]) << 8) | uint16_t(buf[4]));

  return true;
}

//read raw 12-bit output of on-chip ADC
bool adxl_read_adc(uint16_t &adc_raw) {
  //ADC data in ADC_DATA_L/H
  uint8_t buf[2] = {0};

  if (!reg_read_burst(REG_ADC_DATA_L, buf, sizeof(buf))) {
    return false;
  }

  int16_t v = (int16_t)((uint16_t(buf[1]) << 8) | uint16_t(buf[0]));
  adc_raw = (uint16_t)(v & 0x0FFF);

  return true;
}

//read STATUS register
bool adxl_read_status(uint8_t &status) {
  return reg_read_byte(REG_STATUS, status);
}