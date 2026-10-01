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

static void print_help() {
  Serial.println(F("commands:"));
  Serial.println(F("  help | info | dump"));
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
      Serial.println(r == PressureResult::CLIPPED ? F("ERR front end clipped") : F("ERR read failed"));
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
    return;
  }

  if (!safe) {
    Serial.println(F("ERR device is armed; only help/info available"));
    return;
  }

  if (strcmp(cmd, "dump") == 0) {
    logger_dump(Serial);
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
