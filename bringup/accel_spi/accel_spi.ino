/*
  ADXL363 accelerometer SPI bring-up (standalone, independent of the main firmware)
  - Parks pyro outputs low, bridge PFET off, and BOTH SPI chip selects high before SPI starts
  - Tests, in order (send "r" to rerun):
      1. ID reliability: 200 reads of the ID registers; anything below 200/200 is a bad connection
      2. Register write/readback (proves MOSI and the chip accepting writes)
      3. Gravity check: |a| should be ~1000 mg with the board still
  - Then streams x/y/z (mg), |a|, motion and raw temperature every 250 ms, re-checks the ID every
    second ("ID LOST" pinpoints the moment an intermittent joint opens), and reports taps
  - Commands (type, then Enter):
      r          rerun the tests
      c          six-position check: separates per-axis offset from scale error
      live on    25 Hz "S," lines for tools/viewer.py (live off to stop)
  Board: Arduino Pro or Pro Mini, ATmega328P (3.3V, 8 MHz). Serial monitor: 38400, "New Line".
*/
#include <SPI.h>

//pins (must match config.h / schematic)
static constexpr uint8_t PIN_ACCEL_CS = 10;
static constexpr uint8_t PIN_FRAM_CS = A0;
static constexpr uint8_t PIN_INT1 = 3;
static constexpr uint8_t PIN_INT2 = 4;
static constexpr uint8_t PIN_PFET_EN = 9; //HIGH = bridge off
static constexpr uint8_t PIN_PYRO_CHG = A1;
static constexpr uint8_t PIN_PYRO_FIRE = A2;

//ADXL363 commands and registers
static constexpr uint8_t CMD_WRITE = 0x0A;
static constexpr uint8_t CMD_READ = 0x0B;
static constexpr uint8_t REG_DEVID_AD = 0x00; //0xAD
static constexpr uint8_t REG_STATUS = 0x0B;
static constexpr uint8_t REG_XDATA_L = 0x0E;
static constexpr uint8_t REG_TEMP_L = 0x14;
static constexpr uint8_t REG_SOFT_RESET = 0x1F;
static constexpr uint8_t REG_THRESH_ACT_L = 0x20;
static constexpr uint8_t REG_THRESH_ACT_H = 0x21;
static constexpr uint8_t REG_TIME_ACT = 0x22;
static constexpr uint8_t REG_ACT_INACT_CTL = 0x27;
static constexpr uint8_t REG_INTMAP1 = 0x2A;
static constexpr uint8_t REG_INTMAP2 = 0x2B;
static constexpr uint8_t REG_FILTER_CTL = 0x2C;
static constexpr uint8_t REG_POWER_CTL = 0x2D;

static const uint8_t EXPECTED_ID[3] = {0xAD, 0x1D, 0xF3}; //DEVID_AD, DEVID_MST, PARTID

//±2 g range for the best resolution on the bench: 1 mg per LSB
static constexpr uint8_t FILTER_2G_100HZ = (0b00 << 6) | 0b011;
static constexpr uint8_t MEASURE = (1 << 1);
static constexpr uint8_t ACT_BIT = (1 << 4); //STATUS / INTMAP activity bit

//tap filter (same idea as the main firmware): a tap is a short burst of activity, then quiet
static constexpr uint16_t TAP_THRESHOLD_MG = 1000; //referenced: 1 g change starts a burst
static constexpr uint16_t TAP_MAX_MS = 80;
static constexpr uint16_t TAP_QUIET_MS = 150;

//motion = |a| minus a slow gravity baseline (immune to scale/offset error)
static constexpr float GRAVITY_TAU_MS = 60000.0f;
static constexpr uint16_t LIVE_PERIOD_MS = 40;

static const SPISettings ADXL_SPI(4000000UL, MSBFIRST, SPI_MODE0);

static uint32_t g_taps = 0;
static uint32_t g_id_failures = 0;
static bool g_live = false;

static bool g_burst_active = false;
static uint32_t g_burst_start_ms = 0;
static uint32_t g_last_activity_ms = 0;

static float g_gravity_mg = 0;
static uint32_t g_gravity_t_ms = 0;
static bool g_gravity_init = false;

static char g_line[24];
static uint8_t g_len = 0;

//---------------------------------------------------------------------------------------------
//helpers
//---------------------------------------------------------------------------------------------
static void drive_pin(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}

static void reg_write(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(ADXL_SPI);
  digitalWrite(PIN_ACCEL_CS, LOW);
  SPI.transfer(CMD_WRITE);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(PIN_ACCEL_CS, HIGH);
  SPI.endTransaction();
}

static void reg_read(uint8_t reg, uint8_t *buf, uint8_t n) {
  SPI.beginTransaction(ADXL_SPI);
  digitalWrite(PIN_ACCEL_CS, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer(reg);
  for (uint8_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(PIN_ACCEL_CS, HIGH);
  SPI.endTransaction();
}

static uint8_t reg_read1(uint8_t reg) {
  uint8_t v;
  reg_read(reg, &v, 1);
  return v;
}

static int16_t le16(const uint8_t *b) {
  return (int16_t)((uint16_t(b[1]) << 8) | b[0]);
}

static bool id_ok() {
  uint8_t id[3];
  reg_read(REG_DEVID_AD, id, 3);
  return memcmp(id, EXPECTED_ID, 3) == 0;
}

static void print_hex(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

static bool report(const __FlashStringHelper *name, bool pass) {
  Serial.print(pass ? F("[PASS] ") : F("[FAIL] "));
  Serial.println(name);
  return pass;
}

static void arm_activity() {
  reg_write(REG_ACT_INACT_CTL, 0x00);
  reg_write(REG_ACT_INACT_CTL, 0x03); //ACT_EN | ACT_REF (re-captures the reference orientation)
}

//configure for the bench: ±2 g, 100 Hz, referenced activity on INT1 + INT2
static void configure() {
  reg_write(REG_SOFT_RESET, 0x52);
  delay(10);
  reg_write(REG_FILTER_CTL, FILTER_2G_100HZ);
  reg_write(REG_THRESH_ACT_L, (uint8_t)(TAP_THRESHOLD_MG & 0xFF));
  reg_write(REG_THRESH_ACT_H, (uint8_t)((TAP_THRESHOLD_MG >> 8) & 0x07));
  reg_write(REG_TIME_ACT, 1);
  reg_write(REG_INTMAP1, ACT_BIT);
  reg_write(REG_INTMAP2, ACT_BIT);
  reg_write(REG_POWER_CTL, MEASURE);
  arm_activity();
  delay(50); //a few samples so the data registers are fresh
  (void)reg_read1(REG_STATUS); //clear anything latched during setup
}

static void read_mg(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6];
  reg_read(REG_XDATA_L, b, 6);
  x = le16(&b[0]); //±2 g range: 1 mg per LSB
  y = le16(&b[2]);
  z = le16(&b[4]);
}

static int16_t read_temp() {
  uint8_t t[2];
  reg_read(REG_TEMP_L, t, 2);
  return le16(t);
}

static float magnitude(int16_t x, int16_t y, int16_t z) {
  return sqrtf((float)x * x + (float)y * y + (float)z * z);
}

static uint16_t motion_mg(float mag, uint32_t now) {
  if (!g_gravity_init) {
    g_gravity_mg = mag;
    g_gravity_init = true;
  } else {
    float k = (now - g_gravity_t_ms) / GRAVITY_TAU_MS;
    g_gravity_mg += (mag - g_gravity_mg) * (k > 1.0f ? 1.0f : k);
  }
  g_gravity_t_ms = now;
  float d = fabsf(mag - g_gravity_mg);
  return (uint16_t)(d > 65535.0f ? 65535 : d);
}

//block until the user presses Enter (an empty line or any text)
static void wait_for_enter() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}
  delay(20);
  while (Serial.available()) Serial.read();
}

//---------------------------------------------------------------------------------------------
//tests
//---------------------------------------------------------------------------------------------
static bool test_id_reliability() {
  Serial.println(F("\n[1] ID reliability, 200 reads"));
  uint8_t id[4];
  reg_read(REG_DEVID_AD, id, 4);
  Serial.print(F("  first read: "));
  for (uint8_t i = 0; i < 4; i++) {
    print_hex(id[i]);
    Serial.print(' ');
  }
  Serial.println(F(" (expected AD 1D F3 + revision)"));

  uint16_t good = 0;
  for (uint16_t i = 0; i < 200; i++) {
    if (id_ok()) good++;
    delayMicroseconds(500);
  }
  Serial.print(F("  good reads: "));
  Serial.print(good);
  Serial.println(F("/200"));

  if (good == 0) {
    if (id[0] == 0xFF && id[1] == 0xFF) {
      Serial.println(F("  all FF: chip not answering. FRAM passing means D11-D13 are fine, so suspect"));
      Serial.println(F("  D10 (ACCEL_CS) -> U4 pin 8, U4 power (measure across C5), or U4's solder pads."));
    } else if (id[0] == 0x00 && id[1] == 0x00) {
      Serial.println(F("  all 00: MISO held low. Check U4 power (across C5) and for solder bridges."));
    } else {
      Serial.println(F("  garbled: chip partly answering. Suspect a U4 pad making poor contact."));
    }
  } else if (good < 200) {
    Serial.println(F("  INTERMITTENT: the chip answers sometimes. A U4 pad or the D10 joint is marginal."));
  }
  return report(F("ID reads reliably"), good == 200);
}

static bool test_write_readback() {
  Serial.println(F("\n[2] register write/readback"));
  const uint8_t patterns[3] = {0x55, 0xAA, 0x13};
  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    reg_write(REG_THRESH_ACT_L, patterns[i]); //harmless scratch use of a threshold register
    uint8_t back = reg_read1(REG_THRESH_ACT_L);
    if (back != patterns[i]) {
      ok = false;
      Serial.print(F("  wrote "));
      print_hex(patterns[i]);
      Serial.print(F(", read "));
      print_hex(back);
      Serial.println();
    }
  }
  configure(); //restore the real configuration
  return report(F("writes stick (MOSI + chip accepting writes)"), ok);
}

//average 'n' readings 20 ms apart
static void average_mg(uint8_t n, float &x, float &y, float &z) {
  int32_t sx = 0, sy = 0, sz = 0;
  for (uint8_t i = 0; i < n; i++) {
    int16_t ax, ay, az;
    read_mg(ax, ay, az);
    sx += ax;
    sy += ay;
    sz += az;
    delay(20);
  }
  x = (float)sx / n;
  y = (float)sy / n;
  z = (float)sz / n;
}

static bool test_gravity() {
  Serial.println(F("\n[3] gravity check: keep the board still for 2 s"));
  float x, y, z;
  average_mg(100, x, y, z);
  float mag = sqrtf(x * x + y * y + z * z);
  Serial.print(F("  mean x/y/z = "));
  Serial.print(x, 0);
  Serial.print(F(" / "));
  Serial.print(y, 0);
  Serial.print(F(" / "));
  Serial.print(z, 0);
  Serial.print(F(" mg   |a| = "));
  Serial.print(mag, 0);
  Serial.println(F(" mg (expect ~1000)"));
  bool ok = mag >= 850 && mag <= 1150;
  if (!ok) {
    Serial.println(F("  outside 850-1150 mg: run 'c' (six-position check) to see whether it is an offset"));
    Serial.println(F("  on one axis or a scale error. The firmware's motion detection tolerates either."));
  }
  return report(F("|a| within 850-1150 mg"), ok);
}

//six-position check: with an axis pointing up and then down, the true readings are +1000 / -1000 mg,
//so offset = (up + down) / 2 and scale = (up - down) / 2000
static void six_position_check() {
  static const char *const names[3] = {"X", "Y", "Z"};
  float up[3], down[3];
  Serial.println(F("\n[six-position check] for each prompt, hold the board still, then press Enter"));
  for (uint8_t axis = 0; axis < 3; axis++) {
    for (uint8_t dir = 0; dir < 2; dir++) {
      Serial.print(F("  point the +"));
      Serial.print(names[axis]);
      Serial.print(dir == 0 ? F(" axis straight UP") : F(" axis straight DOWN"));
      Serial.println(F(" (check the ADXL363 silkscreen arrows), then Enter"));
      wait_for_enter();
      float v[3];
      average_mg(50, v[0], v[1], v[2]);
      (dir == 0 ? up : down)[axis] = v[axis];
      Serial.print(F("    "));
      Serial.print(names[axis]);
      Serial.print(F(" = "));
      Serial.print(v[axis], 0);
      Serial.println(F(" mg"));
    }
  }
  Serial.println(F("\n  axis   offset(mg)   scale (1.000 = nominal 1 mg/LSB)"));
  bool scale_off = false, offset_off = false;
  for (uint8_t axis = 0; axis < 3; axis++) {
    float offset = (up[axis] + down[axis]) / 2.0f;
    float scale = (up[axis] - down[axis]) / 2000.0f;
    Serial.print(F("  "));
    Serial.print(names[axis]);
    Serial.print(F("      "));
    Serial.print(offset, 0);
    Serial.print(F("          "));
    Serial.println(scale, 3);
    if (fabsf(scale - 1.0f) > 0.10f) scale_off = true;
    if (fabsf(offset) > 150.0f) offset_off = true;
  }
  if (scale_off) {
    Serial.println(F("  a scale is >10% from nominal: confirm the range setting and the part number (ADXL363)."));
  }
  if (offset_off) {
    Serial.println(F("  an offset is >150 mg: board stress on the chip or a damaged part; compare another board."));
  }
  if (!scale_off && !offset_off) {
    Serial.println(F("  offsets and scales look reasonable."));
  }
  configure();
}

static void run_tests() {
  bool ok = test_id_reliability();
  if (id_ok()) { //later tests are meaningless if the chip does not answer at all
    configure();
    ok &= test_write_readback();
    ok &= test_gravity();
  } else {
    ok = false;
    Serial.println(F("  skipping tests 2-3: chip not answering"));
  }
  Serial.println(ok ? F("\nACCEL OK") : F("\nACCEL FAILED"));
  Serial.println(F("\nstreaming: x,y,z mg | |a| | motion | temp_raw   (tap the board to test taps)"));
  Serial.println(F("commands: r = rerun, c = six-position check, live on/off = viewer stream"));
}

//---------------------------------------------------------------------------------------------
//runtime
//---------------------------------------------------------------------------------------------
static void handle_command(const char *cmd) {
  if (strcmp(cmd, "r") == 0) {
    run_tests();
  } else if (strcmp(cmd, "c") == 0) {
    six_position_check();
  } else if (strcmp(cmd, "live on") == 0) {
    g_live = true;
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "live off") == 0) {
    g_live = false;
    Serial.println(F("OK"));
  } else if (cmd[0] != '\0') {
    Serial.println(F("ERR commands: r, c, live on, live off"));
  }
}

static void poll_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_len] = '\0';
      handle_command(g_line);
      g_len = 0;
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    }
  }
}

//activity bursts -> taps; long bursts are movement and are reported but not counted
static void poll_taps(uint32_t now) {
  bool int1 = digitalRead(PIN_INT1) == HIGH;
  bool int2 = digitalRead(PIN_INT2) == HIGH;
  if (int1 || int2) {
    uint8_t status = reg_read1(REG_STATUS); //acknowledges the interrupt
    if (status & ACT_BIT) {
      if (!(int1 && int2) && !g_live) {
        Serial.println(F("  only one INT pin went high: check the other pin's trace (INT1 -> D3, INT2 -> D4)"));
      }
      if (!g_burst_active) {
        g_burst_active = true;
        g_burst_start_ms = now;
      }
      g_last_activity_ms = now;
      arm_activity();
    }
  }

  if (g_burst_active && now - g_last_activity_ms >= TAP_QUIET_MS) {
    g_burst_active = false;
    uint32_t len = g_last_activity_ms - g_burst_start_ms;
    if (g_live) return;
    if (len <= TAP_MAX_MS) {
      g_taps++;
      Serial.print(F("TAP #"));
      Serial.print(g_taps);
      Serial.print(F(" (burst "));
      Serial.print(len);
      Serial.println(F(" ms)"));
    } else {
      Serial.print(F("movement, not a tap (burst "));
      Serial.print(len);
      Serial.println(F(" ms)"));
    }
  }
}

void setup() {
  drive_pin(PIN_ACCEL_CS, HIGH);
  drive_pin(PIN_FRAM_CS, HIGH);
  drive_pin(PIN_PYRO_CHG, LOW);
  drive_pin(PIN_PYRO_FIRE, LOW);
  drive_pin(PIN_PFET_EN, HIGH);
  pinMode(PIN_INT1, INPUT);
  pinMode(PIN_INT2, INPUT);

  Serial.begin(38400); //115200 is 3.5% off at 8 MHz: the board can send but cannot receive
  delay(200);
  Serial.println(F("\n== ADXL363 accelerometer bring-up =="));

  SPI.begin();
  configure();
  run_tests();
}

void loop() {
  poll_serial();
  const uint32_t now = millis();
  poll_taps(now);

  static uint32_t last_stream = 0, last_id = 0;
  static bool id_was_ok = true;

  if (now - last_id >= 1000) {
    last_id = now;
    bool ok = id_ok();
    if (!ok && id_was_ok) {
      g_id_failures++;
      Serial.print(F("!! ID LOST at t="));
      Serial.print(now);
      Serial.print(F(" ms (dropouts so far: "));
      Serial.print(g_id_failures);
      Serial.println(F(")"));
    } else if (ok && !id_was_ok) {
      Serial.println(F("!! ID back: chip answering again"));
      configure();
    }
    id_was_ok = ok;
  }

  const uint16_t period = g_live ? LIVE_PERIOD_MS : 250;
  if (!id_was_ok || now - last_stream < period) return;
  last_stream = now;

  int16_t x, y, z;
  read_mg(x, y, z);
  float mag = magnitude(x, y, z);
  uint16_t motion = motion_mg(mag, now);
  int16_t temp = read_temp();

  if (g_live) {
    //S,t_ms,state,phase,ax_mg,ay_mg,az_mg,motion_mg,temp_raw,p_raw,depth_cm,mv,soc_x100,flags
    //(state/phase/pressure/battery are not measured by this sketch and are left empty)
    Serial.print(F("S,"));
    Serial.print(now);
    Serial.print(F(",,,"));
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(',');
    Serial.print(z);
    Serial.print(',');
    Serial.print(motion);
    Serial.print(',');
    Serial.print(temp);
    Serial.println(F(",,,,,"));
  } else {
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(',');
    Serial.print(z);
    Serial.print(F(" mg | "));
    Serial.print(mag, 0);
    Serial.print(F(" | "));
    Serial.print(motion);
    Serial.print(F(" | "));
    Serial.println(temp);
  }
}
