/*
  Battery / 3.3 V rail power test (standalone; does NOT use the accelerometer, FRAM, or fuel gauge)
  - Every second, powers the strain-gauge bridge for ~70 ms exactly like the main firmware does
    (P-FET Q1: PFET_EN LOW = on) and measures the MCU's own supply voltage (VCC) before and during
    the pulse, using the internal 1.1 V bandgap reference
  - Keeps a history in the ATmega's internal EEPROM, which survives resets and power loss:
    number of boots, reset causes (power-on / reset pin / brown-out / watchdog), lowest VCC at idle
    and during the bridge pulse, and whether VCC was falling when the last reset hit
  - Run it on battery (laptop unplugged) for a few minutes, then plug in and read the report

  Serial monitor 38400, "New Line". Commands:
    r           print the report
    clear       erase the history
    bridge on   pulse the bridge every second (default)
    bridge off  never power the bridge (A/B comparison)
    vcc <mV>    calibrate: type the VCC you measured with a multimeter right now, e.g. "vcc 3305"
  Board: Arduino Pro or Pro Mini, ATmega328P (3.3V, 8 MHz)
*/
#include <EEPROM.h>
#include <avr/wdt.h>

//pins (must match config.h / schematic)
static constexpr uint8_t PIN_PFET_EN = 9; //LOW = bridge powered
static constexpr uint8_t PIN_ACCEL_CS = 10;
static constexpr uint8_t PIN_FRAM_CS = A0;
static constexpr uint8_t PIN_PYRO_CHG = A1;
static constexpr uint8_t PIN_PYRO_FIRE = A2;

//same timing as the main firmware's pressure reading: 30 ms settle + 4 reads 10 ms apart
static constexpr uint16_t PULSE_MS = 70;
static constexpr uint16_t PERIOD_MS = 1000;

//dip thresholds recorded the moment VCC first crosses them during a pulse (the ATmega's
//brown-out detector on a 3.3 V / 8 MHz Pro Mini resets it at ~2.7 V)
static const uint16_t DIP_MV[] = {3200, 3000, 2850};

struct __attribute__((packed)) History {
  uint16_t magic;
  uint16_t boots;
  uint16_t por, ext, bor, wdt; //reset causes
  uint32_t pulses;
  uint16_t min_idle_mv;
  uint16_t min_pulse_mv;
  uint8_t last_dip; //deepest threshold index crossed in the most recent pulse, +1 (0 = none)
  uint8_t pulse_open; //1 while a pulse is in progress: still set at boot = reset hit mid-pulse
  uint8_t bridge_off;
  uint16_t bandgap_mv; //calibrated internal reference, nominally 1100 mV
  uint16_t resets_mid_pulse;
};
static constexpr uint16_t MAGIC = 0x5054; //"PT"

static History h;
uint8_t g_reset_cause __attribute__((section(".noinit")));
void capture_reset_cause() __attribute__((naked, used, section(".init3")));
void capture_reset_cause() {
  g_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

static char g_line[24];
static uint8_t g_len = 0;

static void save() {
  EEPROM.put(0, h); //put() only rewrites bytes that changed
}

static void drive_pin(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}

//VCC from the bandgap: ADC = 1024 * Vbg / VCC  ->  VCC = 1024 * Vbg / ADC
static void select_bandgap() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); //AVcc reference, measure the 1.1 V bandgap
  delay(2); //bandgap settles after switching
}

static uint16_t read_vcc_mv() {
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  uint16_t adc = ADC;
  return adc ? (uint16_t)((1024UL * h.bandgap_mv) / adc) : 0;
}

static uint16_t idle_vcc_mv() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += read_vcc_mv();
  return (uint16_t)(sum / 16);
}

static void reset_history() {
  uint16_t bg = (h.magic == MAGIC && h.bandgap_mv > 900 && h.bandgap_mv < 1300) ? h.bandgap_mv : 1100;
  memset(&h, 0, sizeof(h));
  h.magic = MAGIC;
  h.bandgap_mv = bg; //keep the calibration
  h.min_idle_mv = 0xFFFF;
  h.min_pulse_mv = 0xFFFF;
  save();
}

static void print_mv(uint16_t mv) {
  if (mv == 0xFFFF) {
    Serial.print(F("-"));
  } else {
    Serial.print(mv / 1000);
    Serial.print('.');
    uint16_t frac = mv % 1000;
    if (frac < 100) Serial.print('0');
    if (frac < 10) Serial.print('0');
    Serial.print(frac);
    Serial.print(F(" V"));
  }
}

static void report() {
  Serial.println(F("\n---- power test history (survives resets) ----"));
  Serial.print(F("boots: "));
  Serial.print(h.boots);
  Serial.print(F("   pulses: "));
  Serial.print(h.pulses);
  Serial.print(F("   bridge: "));
  Serial.println(h.bridge_off ? F("OFF") : F("on"));
  Serial.print(F("reset causes: power-on "));
  Serial.print(h.por);
  Serial.print(F(", reset pin "));
  Serial.print(h.ext);
  Serial.print(F(", BROWN-OUT "));
  Serial.print(h.bor);
  Serial.print(F(", watchdog "));
  Serial.println(h.wdt);
  Serial.print(F("resets that hit DURING a bridge pulse: "));
  Serial.println(h.resets_mid_pulse);
  Serial.print(F("lowest VCC idle: "));
  print_mv(h.min_idle_mv);
  Serial.print(F("   lowest VCC during a pulse: "));
  print_mv(h.min_pulse_mv);
  Serial.println();
  Serial.print(F("(bandgap calibration "));
  Serial.print(h.bandgap_mv);
  Serial.println(F(" mV; absolute volts are +-10% until calibrated with 'vcc <mV>', dips are accurate)"));
  Serial.println(F("reset-pin resets come from plugging the cable in/out; brown-outs are the problem."));
}

static void handle(const char *cmd) {
  if (strcmp(cmd, "r") == 0) {
    report();
  } else if (strcmp(cmd, "clear") == 0) {
    reset_history();
    Serial.println(F("OK history cleared"));
  } else if (strcmp(cmd, "bridge on") == 0 || strcmp(cmd, "bridge off") == 0) {
    h.bridge_off = cmd[8] == 'f';
    save();
    Serial.println(h.bridge_off ? F("OK bridge OFF") : F("OK bridge on"));
  } else if (strncmp(cmd, "vcc ", 4) == 0) {
    long actual = atol(cmd + 4);
    uint16_t measured = idle_vcc_mv();
    if (actual < 2000 || actual > 5500 || measured == 0) {
      Serial.println(F("ERR vcc <mV>, e.g. vcc 3305"));
      return;
    }
    h.bandgap_mv = (uint16_t)((uint32_t)h.bandgap_mv * actual / measured);
    save();
    Serial.print(F("OK bandgap now "));
    Serial.print(h.bandgap_mv);
    Serial.println(F(" mV"));
  } else if (cmd[0]) {
    Serial.println(F("ERR commands: r, clear, bridge on|off, vcc <mV>"));
  }
}

static void poll_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_len] = '\0';
      handle(g_line);
      g_len = 0;
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    }
  }
}

void setup() {
  //everything with a physical effect safe, bridge off, both SPI chips deselected
  drive_pin(PIN_PYRO_CHG, LOW);
  drive_pin(PIN_PYRO_FIRE, LOW);
  drive_pin(PIN_PFET_EN, HIGH);
  drive_pin(PIN_ACCEL_CS, HIGH);
  drive_pin(PIN_FRAM_CS, HIGH);

  EEPROM.get(0, h);
  if (h.magic != MAGIC) reset_history();

  h.boots++;
  if (g_reset_cause & _BV(PORF)) h.por++;
  if (g_reset_cause & _BV(EXTRF)) h.ext++;
  if (g_reset_cause & _BV(BORF)) h.bor++;
  if (g_reset_cause & _BV(WDRF)) h.wdt++;
  if (h.pulse_open) h.resets_mid_pulse++; //the previous boot died with the bridge on
  h.pulse_open = 0;
  save();

  Serial.begin(38400);
  Serial.println(F("\n== power test (bridge pulse + VCC monitor) =="));
  Serial.print(F("this boot's reset cause:"));
  if (g_reset_cause & _BV(PORF)) Serial.print(F(" power-on"));
  if (g_reset_cause & _BV(EXTRF)) Serial.print(F(" reset-pin"));
  if (g_reset_cause & _BV(BORF)) Serial.print(F(" BROWN-OUT"));
  if (g_reset_cause & _BV(WDRF)) Serial.print(F(" watchdog"));
  Serial.println();
  report();
  Serial.println(F("\nlive: pulse,idle_mV,min_during_pulse_mV   (unplug the laptop to test on battery)"));

  select_bandgap();
}

void loop() {
  poll_serial();
  static uint32_t last = 0;
  if (millis() - last < PERIOD_MS) return;
  last = millis();

  uint16_t idle = idle_vcc_mv();
  if (idle < h.min_idle_mv) h.min_idle_mv = idle;

  uint16_t pulse_min = 0xFFFF;
  if (!h.bridge_off) {
    h.pulse_open = 1;
    h.last_dip = 0;
    save();
    digitalWrite(PIN_PFET_EN, LOW); //bridge on
    uint32_t t0 = millis();
    while (millis() - t0 < PULSE_MS) {
      uint16_t v = read_vcc_mv();
      if (v < pulse_min) pulse_min = v;
      //record a dip the moment it happens, in case the next instant is a reset
      for (uint8_t i = 0; i < sizeof(DIP_MV) / sizeof(DIP_MV[0]); i++) {
        if (v < DIP_MV[i] && h.last_dip < i + 1) {
          h.last_dip = i + 1;
          if (v < h.min_pulse_mv) h.min_pulse_mv = v;
          save();
        }
      }
    }
    digitalWrite(PIN_PFET_EN, HIGH); //bridge off
    h.pulse_open = 0;
    h.pulses++;
    if (pulse_min < h.min_pulse_mv) h.min_pulse_mv = pulse_min;
  }
  save();

  Serial.print(h.pulses);
  Serial.print(',');
  Serial.print(idle);
  Serial.print(',');
  if (pulse_min != 0xFFFF) Serial.print(pulse_min);
  Serial.println();
}
