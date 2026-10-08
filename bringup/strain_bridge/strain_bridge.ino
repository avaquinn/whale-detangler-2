/*
  Strain-gauge / pressure front-end characterisation (standalone, independent of the main firmware)
  Rev 1.0 path: BF350 quarter bridge -> INA333 (G = 1 + 100k/249 ~ 403, REF ~ 0.6 V) -> ADXL363 aux ADC
  Rev 1.1 path: NAU7802 24-bit bridge ADC at I2C 0x2A (auto-detected; e.g. a SparkFun Qwiic Scale
                wired to A4/A5/3.3V/GND for prototyping before the board respin)
  Bypass path:  if the ADXL363 does not answer, send 'a' to read the INA333 output on the Pro Mini's
                own ADC instead (10-bit, A7). Needs one jumper wire from the ADC_IN net (either end
                of R16) to A7. A7 is analog-only and unused on the schematic.

  Tests, in order (send 'r' to rerun):
    1. Bridge OFF vs ON (catches an inverted/failed PFET and an INA333 stuck at a rail)
    2. Warm-up curve after power-on (use it to choose the settle time)
    3. Noise floor with the bridge held on (std-dev in counts and in microstrain)
  Then it prints a 16-sample mean every 2 s, for shunt-calibration and pressure checks.

  Board: Arduino Pro or Pro Mini, ATmega328P (3.3V, 8 MHz). Serial monitor: 38400.
*/
#include <SPI.h>
#include <Wire.h>

//pins (must match config.h / schematic)
static constexpr uint8_t PIN_ACCEL_CS = 10;
static constexpr uint8_t PIN_FRAM_CS = A0;
static constexpr uint8_t PIN_PFET_EN = 9; //Q1 P-FET: LOW = bridge ON
static constexpr uint8_t PIN_PYRO_CHG = A1;
static constexpr uint8_t PIN_PYRO_FIRE = A2;

//Rev 1.0 analog chain (for unit conversion only)
static constexpr float VS_MV = 3300.0f;
static constexpr float ADC_MV_PER_CODE = 0.8f * VS_MV / 4096.0f; //10%..90% of VS over 12 bits
static constexpr float ADC_MID_MV = VS_MV / 2.0f; //ASSUMED code 0 = mid-supply: verify with a DMM
static constexpr float INA_GAIN = 1.0f + 100000.0f / 249.0f;
static constexpr float GAUGE_FACTOR = 2.0f; //BF350 ~2.0-2.2: use the value on the gauge packet
static constexpr int16_t CLIP_CODE = 2040;

//bypass: Pro Mini ADC on A7, reference = VCC (3.3 V), 10 bits
static constexpr uint8_t PIN_ALT_ADC = A7;
static constexpr float MCU_MV_PER_CODE = VS_MV / 1024.0f;
static constexpr int16_t MCU_CLIP_LO = 20; //INA333 output cannot get closer than ~50 mV to its rails
static constexpr int16_t MCU_CLIP_HI = 1003;

//NAU7802
static constexpr uint8_t NAU_ADDR = 0x2A;
static constexpr float NAU_GAIN = 128.0f;

static const SPISettings ADXL_SPI(4000000UL, MSBFIRST, SPI_MODE0);
enum class FrontEnd : uint8_t { NAU7802, ADXL_AUX, MCU_A7 };
static FrontEnd g_fe = FrontEnd::ADXL_AUX;

//---------------------------------------------------------------------------------------------
//helpers
//---------------------------------------------------------------------------------------------
static void drive_pin(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}

static void bridge(bool on) {
  digitalWrite(PIN_PFET_EN, on ? LOW : HIGH);
}

static void adxl_write(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(ADXL_SPI);
  digitalWrite(PIN_ACCEL_CS, LOW);
  SPI.transfer(0x0A);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(PIN_ACCEL_CS, HIGH);
  SPI.endTransaction();
}

static uint16_t adxl_read16(uint8_t reg) {
  SPI.beginTransaction(ADXL_SPI);
  digitalWrite(PIN_ACCEL_CS, LOW);
  SPI.transfer(0x0B);
  SPI.transfer(reg);
  uint8_t lo = SPI.transfer(0);
  uint8_t hi = SPI.transfer(0);
  digitalWrite(PIN_ACCEL_CS, HIGH);
  SPI.endTransaction();
  return (uint16_t(hi) << 8) | lo;
}

static int16_t adxl_adc() {
  return (int16_t)adxl_read16(0x16); //twos complement, sign extended
}

static bool adxl_setup() {
  adxl_write(0x1F, 0x52); //soft reset
  delay(10);
  if ((adxl_read16(0x00) & 0xFF) != 0xAD) return false;
  adxl_write(0x2C, (0b10 << 6) | 0b011); //±8 g, 100 Hz
  adxl_write(0x2D, (1 << 7) | (1 << 1)); //ADC_EN + measurement
  delay(20);
  return true;
}

static bool nau_write(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(NAU_ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static bool nau_read(uint8_t reg, uint8_t &v) {
  Wire.beginTransmission(NAU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(NAU_ADDR, (uint8_t)1) != 1) return false;
  v = Wire.read();
  return true;
}

static bool nau_wait(uint8_t reg, uint8_t mask, bool set, uint16_t timeout_ms) {
  uint32_t start = millis();
  uint8_t v = 0;
  while (nau_read(reg, v)) {
    if (((v & mask) != 0) == set) return true;
    if (millis() - start > timeout_ms) return false;
  }
  return false;
}

//rate: 0b000 = 10 SPS, 0b011 = 80 SPS
static bool nau_setup(uint8_t rate) {
  uint8_t rev = 0;
  if (!nau_read(0x1F, rev) || (rev & 0x0F) != 0x0F) return false;
  nau_write(0x00, 0x01); //register reset
  delay(1);
  nau_write(0x00, 0x06); //PUD | PUA
  if (!nau_wait(0x00, 0x08, true, 10)) return false; //PUR
  nau_write(0x01, (0b101 << 3) | 0b111); //LDO 3.0 V, gain 128
  nau_write(0x00, 0x06 | 0x80 | 0x10); //+ AVDDS, CS
  nau_write(0x02, (uint8_t)(rate << 4)); //rate, channel 1, CALMOD 00
  uint8_t v = 0;
  nau_read(0x15, v);
  nau_write(0x15, v | 0x30); //clock chopper off
  nau_read(0x1C, v);
  nau_write(0x1C, v | 0x80); //PGA output cap enable
  delay(300); //LDO + bridge settle before the offset calibration
  nau_read(0x02, v);
  nau_write(0x02, v | 0x04); //CALS
  return nau_wait(0x02, 0x04, false, 1000);
}

static bool nau_conversion(int32_t &out) {
  if (!nau_wait(0x00, 0x20, true, 200)) return false; //CR
  Wire.beginTransmission(NAU_ADDR);
  Wire.write(0x12);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(NAU_ADDR, (uint8_t)3) != 3) return false;
  uint32_t u = (uint32_t)Wire.read() << 16;
  u |= (uint32_t)Wire.read() << 8;
  u |= Wire.read();
  if (u & 0x800000UL) u |= 0xFF000000UL;
  out = (int32_t)u;
  return true;
}

static int32_t read_counts() {
  switch (g_fe) {
    case FrontEnd::NAU7802: {
      int32_t v = 0;
      nau_conversion(v);
      return v;
    }
    case FrontEnd::MCU_A7:
      delay(1);
      return analogRead(PIN_ALT_ADC);
    case FrontEnd::ADXL_AUX:
    default:
      delay(10); //one ADXL ODR period -> fresh conversion
      return adxl_adc();
  }
}

//microstrain per count, quarter bridge (Vd = Vex * GF * eps / 4)
static float ue_per_count() {
  switch (g_fe) {
    case FrontEnd::NAU7802:
      return 4.0f / (NAU_GAIN * 16777216.0f * GAUGE_FACTOR) * 1e6f; //ratiometric: Vex = VREF
    case FrontEnd::MCU_A7:
      return 4.0f * (MCU_MV_PER_CODE / INA_GAIN) / (VS_MV * GAUGE_FACTOR) * 1e6f;
    case FrontEnd::ADXL_AUX:
    default:
      return 4.0f * (ADC_MV_PER_CODE / INA_GAIN) / (VS_MV * GAUGE_FACTOR) * 1e6f;
  }
}

//INA333 output voltage implied by a reading (not meaningful for the NAU7802)
static float ina_out_mv(float code) {
  return g_fe == FrontEnd::MCU_A7 ? code * MCU_MV_PER_CODE : ADC_MID_MV + code * ADC_MV_PER_CODE;
}

static bool is_clipped(float code) {
  if (g_fe == FrontEnd::MCU_A7) return code <= MCU_CLIP_LO || code >= MCU_CLIP_HI;
  return code >= CLIP_CODE || code <= -CLIP_CODE;
}

static void print_adc_pin_mv(float code) {
  Serial.print(F(" (~"));
  Serial.print(ina_out_mv(code), 0);
  Serial.print(F(" mV at ADC_IN)"));
}

//---------------------------------------------------------------------------------------------
//tests
//---------------------------------------------------------------------------------------------
static void test_off_on() {
  Serial.println(F("\n[1] bridge OFF vs ON"));
  bridge(false);
  delay(200);
  int32_t off = 0;
  for (uint8_t i = 0; i < 16; i++) off += read_counts();
  bridge(true);
  delay(500);
  int32_t on = 0;
  for (uint8_t i = 0; i < 16; i++) on += read_counts();

  Serial.print(F("  OFF mean: "));
  Serial.print(off / 16.0f, 1);
  if (g_fe != FrontEnd::NAU7802) print_adc_pin_mv(off / 16.0f);
  Serial.print(F("\n  ON  mean: "));
  Serial.print(on / 16.0f, 1);
  if (g_fe != FrontEnd::NAU7802) print_adc_pin_mv(on / 16.0f);
  Serial.println();

  if (g_fe != FrontEnd::NAU7802) {
    if (is_clipped(on / 16.0f)) {
      Serial.println(F("  FAIL: INA333 output is at its limit with the bridge powered."));
      Serial.println(F("        The bridge imbalance x 403 exceeds the output range (only about -0.7 mV..+5.9 mV"));
      Serial.println(F("        of bridge offset fits). Measure bridge-midpoint difference with a DMM."));
    } else if (abs(on - off) < 16 * 5) {
      Serial.println(F("  WARN: ON ~= OFF. Is the PFET switching (PFET_EN LOW = on)? Measure the bridge top node."));
    } else {
      Serial.println(F("  OK: in range and responds to bridge power"));
    }
  }
}

static void test_warmup() {
  Serial.println(F("\n[2] warm-up after power-on: WARM,ms,counts (pick settle time where it flattens)"));
  if (g_fe == FrontEnd::NAU7802) {
    //power-cycle only the analog side (PUA); the bridge is excited from AVDD
    nau_write(0x00, 0x02 | 0x80);
    delay(500);
    nau_write(0x00, 0x06 | 0x80 | 0x10);
  } else {
    bridge(false);
    delay(500);
    bridge(true);
  }
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    int32_t c = read_counts();
    Serial.print(F("WARM,"));
    Serial.print(millis() - t0);
    Serial.print(',');
    Serial.println(c);
  }
}

static void test_noise() {
  Serial.println(F("\n[3] noise with bridge held on, 128 samples"));
  bridge(true);
  delay(1000);
  const uint8_t N = 128;
  float mean = 0, m2 = 0;
  int32_t lo = INT32_MAX, hi = INT32_MIN;
  for (uint8_t i = 0; i < N; i++) { //Welford running variance
    int32_t c = read_counts();
    float d = c - mean;
    mean += d / (i + 1);
    m2 += d * (c - mean);
    if (c < lo) lo = c;
    if (c > hi) hi = c;
  }
  float sd = sqrtf(m2 / (N - 1));
  Serial.print(F("  mean="));
  Serial.print(mean, 1);
  Serial.print(F(" sd="));
  Serial.print(sd, 2);
  Serial.print(F(" p-p="));
  Serial.print(hi - lo);
  Serial.println(F(" counts"));
  Serial.print(F("  resolution: "));
  Serial.print(ue_per_count(), 4);
  Serial.print(F(" ue/count, noise "));
  Serial.print(sd * ue_per_count(), 3);
  Serial.println(F(" ue RMS (quarter-bridge formula, GF=2.0)"));
}

//explain a failed detection: what the ADXL363 returned, and what is on the I2C bus
static void print_diagnostics() {
  uint16_t ids = adxl_read16(0x00); //DEVID_AD (low byte), DEVID_MST (high byte)
  Serial.print(F("  ADXL363 ID bytes: 0x"));
  Serial.print(ids & 0xFF, HEX);
  Serial.print(F(" 0x"));
  Serial.print(ids >> 8, HEX);
  Serial.println(F("  (expected 0xAD 0x1D)"));
  if (ids == 0xFFFF) {
    Serial.println(F("  all 0xFF: nothing is driving MISO (D12). Board not connected/powered, or ACCEL_CS (D10) not reaching U4."));
  } else if (ids == 0x0000) {
    Serial.println(F("  all 0x00: MISO held low. Check U4 power (VS/VDDIO = 3.3 V) and solder joints."));
  } else {
    Serial.println(F("  wrong values: chip answers but garbled. Check SCK/MOSI/MISO wiring and solder joints."));
  }

  Serial.println(F("  I2C scan:"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("    device at 0x"));
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) {
    Serial.println(F("    nothing. Fuel gauge (0x36) missing too, so the Pro Mini is probably not on the logger PCB / PCB not powered."));
  } else {
    Serial.println(F("    0x36 = fuel gauge (PCB is connected). 0x2A would be a NAU7802."));
  }
  Serial.println(F("  To bypass the ADXL363: jumper the ADC_IN net (either end of R16) to A7, then send 'a'."));
}

static void run_tests() {
  test_off_on();
  test_warmup();
  test_noise();
  Serial.println(F("\n[4] MEAN,ms,counts every 2 s (bridge held on). Send 'r' to rerun tests."));
}

void setup() {
  drive_pin(PIN_PYRO_CHG, LOW);
  drive_pin(PIN_PYRO_FIRE, LOW);
  drive_pin(PIN_PFET_EN, HIGH); //bridge off
  drive_pin(PIN_ACCEL_CS, HIGH);
  drive_pin(PIN_FRAM_CS, HIGH);

  Serial.begin(38400); //115200 is 3.5% off at 8 MHz: the board can send but cannot receive
  delay(200);
  Serial.println(F("\n== strain bridge characterisation =="));

  SPI.begin();
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif

  //retry until a front end answers, so an intermittent connection can be found by pressing on joints
  for (;;) {
    if (nau_setup(0b000)) { //10 SPS: lowest noise
      g_fe = FrontEnd::NAU7802;
      Serial.println(F("front end: NAU7802 at 0x2A (10 SPS, gain 128, AVDD 3.0 V)"));
      break;
    }
    if (adxl_setup()) {
      g_fe = FrontEnd::ADXL_AUX;
      Serial.println(F("front end: INA333 -> ADXL363 aux ADC (Rev 1.0)"));
      break;
    }
    Serial.println(F("no front end found (ADXL363 ID read failed, no NAU7802 on I2C)"));
    print_diagnostics();
    Serial.println(F("  retrying in 2 s...\n"));

    //wait 2 s, but switch to the A7 bypass immediately if 'a' arrives
    uint32_t wait_start = millis();
    bool bypass = false;
    while (millis() - wait_start < 2000 && !bypass) {
      bypass = Serial.available() && Serial.read() == 'a';
    }
    if (bypass) {
      g_fe = FrontEnd::MCU_A7;
      Serial.println(F("front end: INA333 -> Pro Mini A7 (bypass, 10-bit, ~4.8 ue/count)"));
      break;
    }
  }
  run_tests();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') run_tests();
  }
  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  last = millis();

  int32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += read_counts();
  Serial.print(F("MEAN,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.println(sum / 16.0f, 1);
}
