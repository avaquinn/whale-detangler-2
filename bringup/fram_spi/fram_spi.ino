/*
  MB85RS2MTA SPI FRAM bring-up (standalone, independent of the main firmware)
  - Parks pyro outputs low, bridge PFET off, and BOTH SPI chip selects high before SPI starts
  - Checks device ID (RDID), write-enable latch (WREN/WRDI), then a non-destructive
    write/readback test: original bytes are saved and restored afterwards
  Board: Arduino Pro or Pro Mini, ATmega328P (3.3V, 8 MHz). Serial monitor: 38400.
*/
#include <SPI.h>

//pins (must match config.h / schematic)
static constexpr uint8_t PIN_FRAM_CS = A0;
static constexpr uint8_t PIN_ACCEL_CS = 10; //ADXL363 shares the bus; keep it deselected
static constexpr uint8_t PIN_PFET_EN = 9; //Q1 P-FET gate: HIGH = bridge OFF
static constexpr uint8_t PIN_PYRO_CHG = A1;
static constexpr uint8_t PIN_PYRO_FIRE = A2;

//MB85RS2MTA: 2 Mbit = 262,144 bytes, 24-bit address, SPI mode 0 or 3
static constexpr uint32_t FRAM_BYTES = 262144UL;
static const SPISettings FRAM_SPI(4000000UL, MSBFIRST, SPI_MODE0); //4 MHz is the max at 8 MHz F_CPU

static constexpr uint8_t CMD_WREN = 0x06;
static constexpr uint8_t CMD_WRDI = 0x04;
static constexpr uint8_t CMD_RDSR = 0x05;
static constexpr uint8_t CMD_READ = 0x03;
static constexpr uint8_t CMD_WRITE = 0x02;
static constexpr uint8_t CMD_RDID = 0x9F;

static constexpr uint8_t SR_WEL = (1 << 1);

//expected RDID: Fujitsu manufacturer 0x04, continuation 0x7F, product 0x48 0x03
static const uint8_t EXPECTED_ID[4] = {0x04, 0x7F, 0x48, 0x03};

//scratch window for the test: last 64 bytes of the device (restored afterwards)
static constexpr uint32_t TEST_ADDR = FRAM_BYTES - 64;
static constexpr uint8_t TEST_LEN = 64;

static void drive_pin(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}

static void select() {
  SPI.beginTransaction(FRAM_SPI);
  digitalWrite(PIN_FRAM_CS, LOW);
}

static void deselect() {
  digitalWrite(PIN_FRAM_CS, HIGH);
  SPI.endTransaction();
}

static void command(uint8_t cmd) {
  select();
  SPI.transfer(cmd);
  deselect();
}

static void send_addr(uint32_t addr) {
  SPI.transfer((uint8_t)(addr >> 16));
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t)addr);
}

static uint8_t read_status() {
  select();
  SPI.transfer(CMD_RDSR);
  uint8_t sr = SPI.transfer(0x00);
  deselect();
  return sr;
}

static void read_id(uint8_t id[4]) {
  select();
  SPI.transfer(CMD_RDID);
  for (uint8_t i = 0; i < 4; i++) id[i] = SPI.transfer(0x00);
  deselect();
}

static void fram_read(uint32_t addr, uint8_t *dst, uint16_t len) {
  select();
  SPI.transfer(CMD_READ);
  send_addr(addr);
  for (uint16_t i = 0; i < len; i++) dst[i] = SPI.transfer(0x00);
  deselect();
}

//WREN is cleared by the chip at the end of every WRITE, so each write needs its own WREN
static void fram_write(uint32_t addr, const uint8_t *src, uint16_t len) {
  command(CMD_WREN);
  select();
  SPI.transfer(CMD_WRITE);
  send_addr(addr);
  for (uint16_t i = 0; i < len; i++) SPI.transfer(src[i]);
  deselect();
}

static void print_bytes(const uint8_t *b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    Serial.print(F(" "));
    if (b[i] < 0x10) Serial.print('0');
    Serial.print(b[i], HEX);
  }
  Serial.println();
}

static bool report(const __FlashStringHelper *name, bool pass) {
  Serial.print(pass ? F("[PASS] ") : F("[FAIL] "));
  Serial.println(name);
  return pass;
}

static bool test_id() {
  uint8_t id[4];
  read_id(id);
  Serial.print(F("RDID:"));
  print_bytes(id, 4);

  bool all_ff = true, all_00 = true;
  for (uint8_t i = 0; i < 4; i++) {
    all_ff &= (id[i] == 0xFF);
    all_00 &= (id[i] == 0x00);
  }
  if (all_ff || all_00) {
    Serial.println(F("  no response: check FRAM_CS (A0), MISO (D12), and U5 power/decoupling"));
  }
  return report(F("device ID matches MB85RS2MTA (04 7F 48 03)"), memcmp(id, EXPECTED_ID, 4) == 0);
}

static bool test_wel() {
  command(CMD_WREN);
  bool set = (read_status() & SR_WEL) != 0;
  command(CMD_WRDI);
  bool cleared = (read_status() & SR_WEL) == 0;
  return report(F("WREN sets / WRDI clears the write-enable latch"), set && cleared);
}

static bool check_pattern(uint8_t seed) {
  uint8_t out[TEST_LEN], in[TEST_LEN];
  for (uint8_t i = 0; i < TEST_LEN; i++) out[i] = (uint8_t)(i * 37 + seed);
  fram_write(TEST_ADDR, out, TEST_LEN);
  fram_read(TEST_ADDR, in, TEST_LEN);
  return memcmp(out, in, TEST_LEN) == 0;
}

static bool test_readback() {
  uint8_t saved[TEST_LEN], restored[TEST_LEN];
  fram_read(TEST_ADDR, saved, TEST_LEN);

  //two complementary patterns so every bit is exercised as both 0 and 1
  bool ok = check_pattern(0x5A) && check_pattern(0xA5);
  report(F("64-byte write/readback at end of array"), ok);

  fram_write(TEST_ADDR, saved, TEST_LEN);
  fram_read(TEST_ADDR, restored, TEST_LEN);
  bool back = memcmp(saved, restored, TEST_LEN) == 0;
  report(F("original bytes restored"), back);
  return ok && back;
}

void setup() {
  //all CS lines high BEFORE any SPI clocking so the ADXL363 and FRAM never both listen
  drive_pin(PIN_FRAM_CS, HIGH);
  drive_pin(PIN_ACCEL_CS, HIGH);
  drive_pin(PIN_PYRO_CHG, LOW);
  drive_pin(PIN_PYRO_FIRE, LOW);
  drive_pin(PIN_PFET_EN, HIGH);

  Serial.begin(38400); //115200 is 3.5% off at 8 MHz: the board can send but cannot receive
  delay(200);
  Serial.println(F("\n== MB85RS2MTA SPI FRAM bring-up =="));

  SPI.begin();

  bool ok = test_id();
  ok &= test_wel();
  ok &= test_readback();

  Serial.println(ok ? F("\nFRAM OK") : F("\nFRAM FAILED"));
}

void loop() {}
