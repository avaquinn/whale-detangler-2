/*
  Serial command console (FTDI header, SERIAL_BAUD, newline-terminated)
  - Log offload (dump), identity (serial), depth calibration, pressure streaming for bench tests
  - Commands that block, change calibration, or erase data only run while the device is safe
    (surface / fired / fault), never while armed underwater or charging
*/
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>
#include "console.h"
#include "config.h"
#include "types.h"
#include "logger.h"
#include "pressure.h"
#include "ADXL.h"

static constexpr uint8_t LINE_MAX = 40;
static char g_line[LINE_MAX + 1];
static uint8_t g_len = 0;

static int32_t g_pending_zero = 0; //"cal zero" result waiting for "cal span"
static bool g_have_pending_zero = false;

static bool g_live = false;
static bool g_log_surface = Logging::LOG_SURFACE_SAMPLES;

bool console_liveMode() {
  return g_live;
}

bool console_logSurface() {
  return g_log_surface;
}

//parse "on" / "off"; false if neither
static bool parse_on_off(const char *arg, bool &value) {
  if (arg && strcmp(arg, "on") == 0) {
    value = true;
    return true;
  }
  if (arg && strcmp(arg, "off") == 0) {
    value = false;
    return true;
  }
  return false;
}

static void print_help() {
  Serial.println(F("commands:"));
  Serial.println(F("  help | info | dump"));
  Serial.println(F("  live on|off              stream S lines at 25 Hz for tools/viewer.py"));
  Serial.println(F("  logsurface on|off        also log samples to FRAM at the surface (bench)"));
  Serial.println(F("  rec <hz> YES             ERASE logs, record accel at 1-50 Hz to FRAM (no laptop needed)"));
  Serial.println(F("  rec | rec stop           recording status / stop and keep the recording"));
  Serial.println(F("  rec clear YES            erase the recording, back to normal logging"));
  Serial.println(F("  bridge on|off            power the strain-gauge bridge or not (kept across resets)"));
  Serial.println(F("  serial <id>              set board serial (e.g. 26.10.10001)"));
  Serial.println(F("  stream <seconds>         PRS,t_ms,raw,depth_cm,temp_raw,result at ~10 Hz"));
  Serial.println(F("  cal zero                 record raw at 0 depth (surface / atmospheric)"));
  Serial.println(F("  cal span <depth_cm>      record raw at a known applied depth, save cal"));
  Serial.println(F("  cal set <zero> <cm/cnt>  set calibration directly"));
  Serial.println(F("  cal clear                invalidate calibration (device will not arm)"));
  Serial.println(F("  afecal                   NAU7802 internal offset calibration"));
  Serial.println(F("  erase YES                delete all records"));
  Serial.println(F("  rearm YES                clear the FIRED latch (bench only), then reset"));
}

//average CAL_READINGS good readings; false if any reading fails or clips
static bool averaged_raw(int32_t &out) {
  int64_t sum = 0;
  for (uint8_t i = 0; i < Pressure::CAL_READINGS; i++) {
    int32_t raw = 0;
    PressureResult r = pressure_read_raw(raw);
    if (r != PressureResult::OK) {
      Serial.println(r == PressureResult::CLIPPED    ? F("ERR front end clipped")
                     : r == PressureResult::DISABLED ? F("ERR bridge is off ('bridge on')")
                                                     : F("ERR read failed"));
      return false;
    }
    sum += raw;
    wdt_reset();
  }
  out = (int32_t)(sum / Pressure::CAL_READINGS);
  return true;
}

static void save_cal(const PressureCal &cal) {
  pressure_setCal(cal);
  bool ok = logger_setCal(cal);
  (void)logger_logEvent(millis(), EventCode::CAL_CHANGED, cal.valid, 0);
  Serial.println(ok ? F("OK cal saved") : F("ERR cal not persisted (logger not ready)"));
}

static void cmd_cal(char *args) {
  char *sub = strtok(args, " ");
  if (!sub) {
    Serial.println(F("ERR cal zero|span|set|clear"));
    return;
  }

  if (strcmp(sub, "zero") == 0) {
    if (!averaged_raw(g_pending_zero)) return;
    g_have_pending_zero = true;
    Serial.print(F("OK zero_raw="));
    Serial.println(g_pending_zero);
  } else if (strcmp(sub, "span") == 0) {
    char *d = strtok(NULL, " ");
    if (!d || !g_have_pending_zero) {
      Serial.println(F("ERR run 'cal zero' first, then 'cal span <depth_cm>'"));
      return;
    }
    int32_t raw = 0;
    if (!averaged_raw(raw)) return;
    int32_t span = raw - g_pending_zero;
    if (labs(span) < Pressure::CAL_MIN_SPAN_COUNTS) {
      Serial.println(F("ERR span too small: is pressure applied? is the bridge powered?"));
      return;
    }
    PressureCal cal = {g_pending_zero, (float)atol(d) / (float)span, 1};
    Serial.print(F("span_raw="));
    Serial.print(raw);
    Serial.print(F(" cm_per_count="));
    Serial.println(cal.cm_per_count, 6);
    save_cal(cal);
  } else if (strcmp(sub, "set") == 0) {
    char *z = strtok(NULL, " ");
    char *k = strtok(NULL, " ");
    if (!z || !k) {
      Serial.println(F("ERR cal set <zero_raw> <cm_per_count>"));
      return;
    }
    PressureCal cal = {atol(z), (float)atof(k), 1};
    save_cal(cal);
  } else if (strcmp(sub, "clear") == 0) {
    PressureCal cal = pressure_getCal();
    cal.valid = 0;
    save_cal(cal);
  } else {
    Serial.println(F("ERR cal zero|span|set|clear"));
  }
}

//stream pressure readings for bench/pressure-chamber tests; any received byte stops it
static void cmd_stream(char *args) {
  uint32_t seconds = args ? strtoul(args, NULL, 10) : 10;
  uint32_t start = millis();
  Serial.println(F("# PRS,t_ms,raw,depth_cm,temp_raw,result(0=ok,1=clip,2=fail)"));
  while ((uint32_t)(millis() - start) < seconds * 1000UL) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      break;
    }
    int32_t raw = 0;
    int16_t depth = 0, temp = 0;
    PressureResult r = pressure_read_raw(raw);
    bool have_depth = (r == PressureResult::OK) && pressure_toDepthCm(raw, depth);
    adxl_read_temp(temp);

    Serial.print(F("PRS,"));
    Serial.print(millis());
    Serial.print(',');
    Serial.print(raw);
    Serial.print(',');
    if (have_depth) Serial.print(depth);
    Serial.print(',');
    Serial.print(temp);
    Serial.print(',');
    Serial.println((uint8_t)r);

    wdt_reset();
    delay(100);
  }
  Serial.println(F("END"));
}

//rec <hz> YES | rec stop | rec clear YES
static void cmd_rec(char *args) {
  char *a = strtok(args, " ");
  char *b = strtok(NULL, " ");
  if (a && strcmp(a, "stop") == 0) {
    Serial.println(logger_recStop() ? F("OK recording stopped and kept; 'dump' to read it") : F("ERR not recording"));
  } else if (a && strcmp(a, "clear") == 0 && b && strcmp(b, "YES") == 0) {
    Serial.println(logger_erase() ? F("OK recording erased, normal logging") : F("ERR"));
  } else if (a && b && strcmp(b, "YES") == 0) {
    long hz = atol(a);
    if (hz < Logging::REC_MIN_HZ || hz > Logging::REC_MAX_HZ) {
      Serial.println(F("ERR rate must be 1-50 Hz"));
      return;
    }
    Serial.println(F("erasing (about 1 s)..."));
    if (logger_recStart((uint8_t)hz)) {
      Serial.print(F("OK "));
      logger_printRecStatus(Serial);
      Serial.println(F("You can unplug the laptop; recording continues on battery (green flash every 2 s)."));
    } else {
      Serial.println(F("ERR could not start (FRAM?)"));
    }
  } else {
    Serial.println(F("ERR rec <hz> YES | rec stop | rec clear YES   (starting a recording ERASES the logs)"));
  }
}

static void execute(char *line, bool safe) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;
  char *rest = strtok(NULL, "");

  if (strcmp(cmd, "help") == 0) {
    print_help();
    return;
  }
  if (strcmp(cmd, "info") == 0) {
    logger_printInfo(Serial);
    Serial.print(F("frontend: "));
    Serial.println(PRESSURE_FRONTEND == PRESSURE_FE_NAU7802 ? F("NAU7802") : F("ADXL363 aux ADC"));
    Serial.print(F("bench_no_pyro: "));
    Serial.println(BENCH_NO_PYRO);
    Serial.print(F("bridge: "));
    Serial.println(logger_bridgeEnabled() ? F("on") : F("OFF"));
    Serial.print(F("live: "));
    Serial.print(g_live);
    Serial.print(F("  logsurface: "));
    Serial.println(g_log_surface);
    return;
  }
  //read-only, so allowed in every state
  if (strcmp(cmd, "live") == 0) {
    Serial.println(parse_on_off(rest, g_live) ? F("OK") : F("ERR live on|off"));
    return;
  }
  if (strcmp(cmd, "rec") == 0 && !rest) {
    logger_printRecStatus(Serial);
    return;
  }

  if (!safe) {
    Serial.println(F("ERR device is armed; only help/info available"));
    return;
  }

  if (strcmp(cmd, "dump") == 0) {
    logger_dump(Serial);
  } else if (strcmp(cmd, "rec") == 0) {
    cmd_rec(rest);
  } else if (strcmp(cmd, "bridge") == 0) {
    bool on = true;
    if (!parse_on_off(rest, on)) {
      Serial.println(F("ERR bridge on|off"));
    } else {
      pressure_setEnabled(on);
      Serial.println(logger_setBridgeEnabled(on) ? (on ? F("OK bridge on") : F("OK bridge off: no pressure/depth until 'bridge on'"))
                                                  : F("ERR not saved (logger not ready)"));
    }
  } else if (strcmp(cmd, "logsurface") == 0) {
    Serial.println(parse_on_off(rest, g_log_surface) ? F("OK") : F("ERR logsurface on|off"));
  } else if (strcmp(cmd, "serial") == 0 && rest) {
    Serial.println(logger_setSerial(rest) ? F("OK") : F("ERR"));
  } else if (strcmp(cmd, "stream") == 0) {
    cmd_stream(rest);
  } else if (strcmp(cmd, "cal") == 0 && rest) {
    cmd_cal(rest);
  } else if (strcmp(cmd, "afecal") == 0) {
    Serial.println(pressure_calibrate_afe() ? F("OK") : F("ERR"));
  } else if (strcmp(cmd, "erase") == 0 && rest && strcmp(rest, "YES") == 0) {
    Serial.println(logger_erase() ? F("OK erased") : F("ERR"));
  } else if (strcmp(cmd, "rearm") == 0 && rest && strcmp(rest, "YES") == 0) {
    bool ok = logger_setFired(false);
    (void)logger_logEvent(millis(), EventCode::REARMED, 0, 0);
    Serial.println(ok ? F("OK latch cleared, reset the board") : F("ERR"));
  } else {
    Serial.println(F("ERR unknown command, try 'help'"));
  }
}

void console_poll(bool safe) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_len] = '\0';
      execute(g_line, safe);
      g_len = 0;
    } else if (g_len < LINE_MAX) {
      g_line[g_len++] = c;
    }
  }
}
