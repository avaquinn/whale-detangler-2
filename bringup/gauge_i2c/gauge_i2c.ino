/*
  MAX17048 fuel gauge I2C bring-up (standalone, independent of the main firmware)
  - Parks pyro outputs low, bridge PFET off, and both SPI chip selects high
  - Scans the I2C bus, dumps the gauge registers once, then streams VCELL/SOC/ALRT every 2 s
  Board: Arduino Pro or Pro Mini, ATmega328P (3.3V, 8 MHz). Serial monitor: 115200.
*/
#include <Wire.h>

//pins (must match config.h / schematic)
static constexpr uint8_t PIN_ALERT = 2; //MAX17048 ALRT, open-drain, active-low
static constexpr uint8_t PIN_PFET_EN = 9; //Q1 P-FET gate: HIGH = bridge OFF, LOW = bridge ON
static constexpr uint8_t PIN_ACCEL_CS = 10;
static constexpr uint8_t PIN_FRAM_CS = A0;
static constexpr uint8_t PIN_PYRO_CHG = A1;
static constexpr uint8_t PIN_PYRO_FIRE = A2;

static constexpr uint8_t GAUGE_ADDR = 0x36;

//MAX17048 registers
static constexpr uint8_t REG_VCELL = 0x02;
static constexpr uint8_t REG_SOC = 0x04;
static constexpr uint8_t REG_MODE = 0x06;
static constexpr uint8_t REG_VERSION = 0x08;
static constexpr uint8_t REG_HIBRT = 0x0A;
static constexpr uint8_t REG_CONFIG = 0x0C;
static constexpr uint8_t REG_VALRT = 0x14;
static constexpr uint8_t REG_CRATE = 0x16;
static constexpr uint8_t REG_VRESET_ID = 0x18;
static constexpr uint8_t REG_STATUS = 0x1A;

//write the level first so the pin never glitches the wrong way when it becomes an output
static void drive_pin(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}

static void park_outputs() {
  drive_pin(PIN_PYRO_CHG, LOW);
  drive_pin(PIN_PYRO_FIRE, LOW);
  drive_pin(PIN_PFET_EN, HIGH);
  drive_pin(PIN_ACCEL_CS, HIGH);
  drive_pin(PIN_FRAM_CS, HIGH);
}

//read a 16-bit register (MSB first); false on any bus error
static bool read_reg(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(GAUGE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; //repeated start

  if (Wire.requestFrom(GAUGE_ADDR, (uint8_t)2) != 2) return false;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  out = (uint16_t(msb) << 8) | lsb;
  return true;
}

static void print_hex16(uint16_t v) {
  Serial.print(F("0x"));
  if (v < 0x1000) Serial.print('0');
  if (v < 0x100) Serial.print('0');
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

static void dump_reg(const __FlashStringHelper *name, uint8_t reg) {
  uint16_t v = 0;
  Serial.print(name);
  if (read_reg(reg, v)) {
    print_hex16(v);
  } else {
    Serial.print(F("READ FAILED"));
  }
  Serial.println();
}

static void scan_bus() {
  Serial.println(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  device at 0x"));
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  nothing found: check SDA/SCL (A4/A5), R1/R2 pull-ups, and that U2 VDD has power"));
  }
}

//VCELL LSb = 78.125 uV = 5/64 mV
static uint16_t vcell_mv(uint16_t raw) {
  return (uint16_t)(((uint32_t)raw * 5UL) / 64UL);
}

static void print_measurements() {
  uint16_t vraw = 0, sraw = 0, craw = 0;
  bool ok = read_reg(REG_VCELL, vraw) && read_reg(REG_SOC, sraw) && read_reg(REG_CRATE, craw);
  if (!ok) {
    Serial.println(F("measurement read FAILED"));
    return;
  }

  uint16_t mv = vcell_mv(vraw);
  Serial.print(F("VCELL="));
  Serial.print(mv);
  Serial.print(F("mV  SOC="));
  Serial.print(sraw >> 8); //SOC LSb = 1/256 %
  Serial.print('.');
  uint16_t frac = ((sraw & 0xFF) * 100U) / 256U;
  if (frac < 10) Serial.print('0');
  Serial.print(frac);
  Serial.print(F("%  CRATE="));
  Serial.print(((int32_t)(int16_t)craw * 208L) / 1000L); //0.208 %/hr per LSb
  Serial.print(F("%/hr  ALRT="));
  Serial.println(digitalRead(PIN_ALERT) == LOW ? F("ASSERTED(low)") : F("idle(high)"));

  if (mv < 2500) {
    Serial.println(F("  VCELL implausibly low: U2 pin 2 (CELL) must connect to VBAT, it looks no-connect on the schematic"));
  }
}

void setup() {
  park_outputs();
  pinMode(PIN_ALERT, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n== MAX17048 I2C bring-up =="));

  Wire.begin();
  Wire.setClock(100000); //start at 100 kHz for bring-up; firmware uses 400 kHz
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true); //never hang on a stuck bus
#endif

  scan_bus();

  uint16_t version = 0;
  if (!read_reg(REG_VERSION, version)) {
    Serial.println(F("MAX17048 did not answer at 0x36"));
    return;
  }

  Serial.println(F("Register dump (reset defaults in brackets):"));
  dump_reg(F("  VERSION   [0x001x] "), REG_VERSION);
  dump_reg(F("  VCELL              "), REG_VCELL);
  dump_reg(F("  SOC                "), REG_SOC);
  dump_reg(F("  MODE      [0x0000] "), REG_MODE);
  dump_reg(F("  HIBRT     [0x8030] "), REG_HIBRT);
  dump_reg(F("  CONFIG    [0x971C] "), REG_CONFIG);
  dump_reg(F("  VALRT     [0x00FF] "), REG_VALRT);
  dump_reg(F("  CRATE              "), REG_CRATE);
  dump_reg(F("  VRESET/ID [0x96xx] "), REG_VRESET_ID);
  dump_reg(F("  STATUS             "), REG_STATUS);
  Serial.println(F("  (STATUS bit8 RI=1 after power-up means the gauge is unconfigured, which is expected here)"));
  Serial.println();
}

void loop() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - last_ms) < 2000) return;
  last_ms = now;
  print_measurements();
}
