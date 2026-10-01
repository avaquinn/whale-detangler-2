/*
  Data logging layer for FRAM
  - Header: two CRC-protected slots written alternately (A/B) so a power cut mid-write can
    never lose the write pointers, identity, calibration, or the FIRED latch
  - Journal ring (boot / event / cycle-summary records): large enough for 12+ years of cycles
  - Sample ring: raw snapshots while underwater; overwrites the oldest samples, never the journal
  - Every record is framed with a marker, sequence number, boot count, and CRC-16, so a reader can
    resynchronise after a ring wraps and reject torn or stale records
*/
#include <string.h>
#include <util/crc16.h>
#include <avr/wdt.h>
#include "logger.h"
#include "FRAM.h"

static constexpr uint32_t LOG_MAGIC = 0x57444C32UL; //"WDL2"
static constexpr uint8_t LAYOUT_VERSION = 2;
static constexpr uint8_t RECORD_MARKER = 0xA5;
static constexpr uint8_t MAX_PAYLOAD = 48;

struct __attribute__((packed)) LogHeader {
  uint32_t magic;
  uint8_t layout;
  uint8_t fired; //persistent one-shot latch
  uint16_t boot_count;
  uint32_t generation; //newer slot wins
  uint32_t seq; //next record sequence number
  uint32_t journal_next;
  uint32_t sample_next;
  uint8_t journal_wrapped;
  uint8_t sample_wrapped;
  uint32_t cycle_count;
  char board_serial[DeviceInfo::BOARD_SERIAL_MAX_LEN];
  PressureCal cal;
  uint16_t crc; //must stay last
};
static_assert(sizeof(LogHeader) <= Logging::HEADER_SLOT_BYTES, "header slot too small");
static_assert(2 * Logging::HEADER_SLOT_BYTES <= Logging::JOURNAL_START, "header overlaps journal");
static_assert(Logging::SAMPLE_END <= FRAM_TOTAL_BYTES, "layout exceeds FRAM");

struct __attribute__((packed)) FrameHeader {
  uint8_t marker;
  uint8_t type;
  uint8_t len;
  uint16_t boot;
  uint32_t seq;
};
static constexpr uint8_t FRAME_OVERHEAD = sizeof(FrameHeader) + 2; //+ CRC-16

struct Region {
  uint32_t start;
  uint32_t end;
  uint32_t LogHeader::*next;
  uint8_t LogHeader::*wrapped;
};
static const Region JOURNAL = {Logging::JOURNAL_START, Logging::JOURNAL_END, &LogHeader::journal_next, &LogHeader::journal_wrapped};
static const Region SAMPLES = {Logging::SAMPLE_START, Logging::SAMPLE_END, &LogHeader::sample_next, &LogHeader::sample_wrapped};

static LogHeader g_hdr;
static bool g_ready = false;

static uint16_t crc16(uint16_t crc, const uint8_t *p, size_t n) {
  while (n--) crc = _crc_xmodem_update(crc, *p++); //CRC-16/CCITT polynomial 0x1021
  return crc;
}

static uint16_t header_crc(const LogHeader &h) {
  return crc16(0xFFFF, reinterpret_cast<const uint8_t *>(&h), offsetof(LogHeader, crc));
}

static bool header_valid(const LogHeader &h) {
  return h.magic == LOG_MAGIC && h.layout == LAYOUT_VERSION && h.crc == header_crc(h) &&
         h.journal_next >= JOURNAL.start && h.journal_next <= JOURNAL.end &&
         h.sample_next >= SAMPLES.start && h.sample_next <= SAMPLES.end;
}

//load the newest valid slot
static bool header_load() {
  LogHeader a, b;
  bool va = fram_read(0, reinterpret_cast<uint8_t *>(&a), sizeof(a)) && header_valid(a);
  bool vb = fram_read(Logging::HEADER_SLOT_BYTES, reinterpret_cast<uint8_t *>(&b), sizeof(b)) && header_valid(b);
  if (!va && !vb) return false;
  g_hdr = (va && (!vb || (int32_t)(a.generation - b.generation) > 0)) ? a : b;
  return true;
}

//write to the slot NOT holding the current copy, so one valid copy always survives
static bool header_store() {
  g_hdr.generation++;
  g_hdr.crc = header_crc(g_hdr);
  uint32_t slot = (g_hdr.generation & 1) ? Logging::HEADER_SLOT_BYTES : 0;
  return fram_write(slot, reinterpret_cast<const uint8_t *>(&g_hdr), sizeof(g_hdr));
}

static bool erase_regions() {
  g_hdr.journal_next = JOURNAL.start;
  g_hdr.sample_next = SAMPLES.start;
  g_hdr.journal_wrapped = 0;
  g_hdr.sample_wrapped = 0;
  return fram_fill(JOURNAL.start, 0, JOURNAL.end - JOURNAL.start) &&
         fram_fill(SAMPLES.start, 0, SAMPLES.end - SAMPLES.start);
}

static bool format() {
  memset(&g_hdr, 0, sizeof(g_hdr));
  g_hdr.magic = LOG_MAGIC;
  g_hdr.layout = LAYOUT_VERSION;
  strncpy(g_hdr.board_serial, DeviceInfo::BOARD_SERIAL_DEFAULT, sizeof(g_hdr.board_serial) - 1);
  return erase_regions() && header_store() && header_store(); //populate both slots
}

static bool append(const Region &rg, LogRecordType type, const void *payload, uint8_t len) {
  if (!g_ready || len > MAX_PAYLOAD) return false;

  uint8_t buf[sizeof(FrameHeader) + MAX_PAYLOAD + 2];
  FrameHeader fh = {RECORD_MARKER, (uint8_t)type, len, g_hdr.boot_count, g_hdr.seq};
  memcpy(buf, &fh, sizeof(fh));
  memcpy(buf + sizeof(fh), payload, len);
  uint16_t crc = crc16(0xFFFF, buf + 1, sizeof(fh) - 1 + len); //everything after the marker
  buf[sizeof(fh) + len] = (uint8_t)(crc & 0xFF);
  buf[sizeof(fh) + len + 1] = (uint8_t)(crc >> 8);

  const uint8_t frame_len = (uint8_t)(len + FRAME_OVERHEAD);
  uint32_t addr = g_hdr.*rg.next;
  if (addr + frame_len > rg.end) { //does not fit before the end: wrap to the start
    addr = rg.start;
    g_hdr.*rg.wrapped = 1;
  }
  if (!fram_write(addr, buf, frame_len)) return false;

  g_hdr.*rg.next = addr + frame_len;
  g_hdr.seq++;
  return header_store();
}

bool logger_init(uint8_t reset_cause) {
  g_ready = false;
  if (!fram_init()) {
    return false; //FRAM missing or not answering
  }
  if (!header_load() && !format()) {
    return false;
  }
  g_hdr.boot_count++;
  if (!header_store()) {
    return false;
  }
  g_ready = true;

  BootRecord rec = {};
  rec.reset_cause = reset_cause;
  strncpy(rec.fw_version, DeviceInfo::FW_VERSION_STR, sizeof(rec.fw_version) - 1);
  return append(JOURNAL, LogRecordType::BOOT, &rec, sizeof(rec));
}

bool logger_ready() {
  return g_ready;
}

bool logger_appendSample(const SensorSnapshot &sample) {
  return append(SAMPLES, LogRecordType::SAMPLE, &sample, sizeof(sample));
}

bool logger_logEvent(uint32_t t_ms, EventCode code, uint8_t data0, uint16_t data1) {
  EventRecord ev = {t_ms, code, data0, data1};
  return append(JOURNAL, LogRecordType::EVENT, &ev, sizeof(ev));
}

bool logger_appendCycle(CycleSummary &summary) {
  if (!g_ready) return false;
  summary.cycle_idx = g_hdr.cycle_count++;
  return append(JOURNAL, LogRecordType::CYCLE_SUMMARY, &summary, sizeof(summary));
}

bool logger_isFired() {
  return g_ready && g_hdr.fired;
}

bool logger_setFired(bool fired) {
  if (!g_ready) return false;
  g_hdr.fired = fired ? 1 : 0;
  return header_store();
}

bool logger_getCal(PressureCal &cal) {
  if (!g_ready) return false;
  cal = g_hdr.cal;
  return true;
}

bool logger_setCal(const PressureCal &cal) {
  if (!g_ready) return false;
  g_hdr.cal = cal;
  return header_store();
}

bool logger_setSerial(const char *serial) {
  if (!g_ready) return false;
  memset(g_hdr.board_serial, 0, sizeof(g_hdr.board_serial));
  strncpy(g_hdr.board_serial, serial, sizeof(g_hdr.board_serial) - 1);
  return header_store();
}

bool logger_erase() {
  if (!g_ready) return false;
  return erase_regions() && header_store();
}

//---------------------------------------------------------------------------------------------
//readback
//---------------------------------------------------------------------------------------------
static const __FlashStringHelper *event_name(uint8_t code) {
  switch ((EventCode)code) {
    case EventCode::STATE_CHANGE: return F("STATE_CHANGE");
    case EventCode::PHASE_CHANGE: return F("PHASE_CHANGE");
    case EventCode::CHARGE_REQUESTED: return F("CHARGE_REQUESTED");
    case EventCode::CHARGE_STARTED: return F("CHARGE_STARTED");
    case EventCode::CHARGE_ABORTED: return F("CHARGE_ABORTED");
    case EventCode::CHARGE_TIMEOUT: return F("CHARGE_TIMEOUT");
    case EventCode::FIRED: return F("FIRED");
    case EventCode::SENSOR_FAULT: return F("SENSOR_FAULT");
    case EventCode::SENSOR_RECOVERED: return F("SENSOR_RECOVERED");
    case EventCode::STORAGE_FAULT: return F("STORAGE_FAULT");
    case EventCode::PYRO_FAULT: return F("PYRO_FAULT");
    case EventCode::BATTERY_ALERT: return F("BATTERY_ALERT");
    case EventCode::CAL_CHANGED: return F("CAL_CHANGED");
    case EventCode::REARMED: return F("REARMED");
    default: return F("UNKNOWN");
  }
}

static void print_csv(Print &out, int32_t v) {
  out.print(',');
  out.print(v);
}

static void print_profile(Print &out, const AccelProfile &p) {
  print_csv(out, (int32_t)p.duration_ms);
  print_csv(out, p.sample_count);
  print_csv(out, p.peak_mg);
  print_csv(out, p.rms_mg);
}

static void print_record(Print &out, const FrameHeader &fh, const uint8_t *payload) {
  switch ((LogRecordType)fh.type) {
    case LogRecordType::BOOT: {
      BootRecord r;
      memcpy(&r, payload, sizeof(r));
      out.print(F("BOOT"));
      print_csv(out, fh.boot);
      print_csv(out, (int32_t)fh.seq);
      print_csv(out, r.reset_cause);
      out.print(',');
      out.println(r.fw_version);
      break;
    }
    case LogRecordType::EVENT: {
      EventRecord r;
      memcpy(&r, payload, sizeof(r));
      out.print(F("EVT"));
      print_csv(out, fh.boot);
      print_csv(out, (int32_t)fh.seq);
      print_csv(out, (int32_t)r.t_ms);
      out.print(',');
      out.print(event_name((uint8_t)r.code));
      print_csv(out, r.data0);
      print_csv(out, r.data1);
      out.println();
      break;
    }
    case LogRecordType::SAMPLE: {
      SensorSnapshot s;
      memcpy(&s, payload, sizeof(s));
      out.print(F("SMP"));
      print_csv(out, fh.boot);
      print_csv(out, (int32_t)fh.seq);
      print_csv(out, (int32_t)s.t_ms);
      print_csv(out, s.state);
      print_csv(out, s.phase);
      print_csv(out, s.ax);
      print_csv(out, s.ay);
      print_csv(out, s.az);
      print_csv(out, s.temp_raw);
      print_csv(out, s.pressure_raw);
      print_csv(out, s.depth_cm);
      print_csv(out, s.batt.voltage_mv);
      print_csv(out, s.batt.soc_x100);
      print_csv(out, s.flags);
      out.println();
      break;
    }
    case LogRecordType::CYCLE_SUMMARY: {
      CycleSummary c;
      memcpy(&c, payload, sizeof(c));
      out.print(F("CYC"));
      print_csv(out, fh.boot);
      print_csv(out, (int32_t)fh.seq);
      print_csv(out, (int32_t)c.cycle_idx);
      print_csv(out, (int32_t)c.start_t_ms);
      print_csv(out, (int32_t)c.end_t_ms);
      print_csv(out, c.bottom_depth_cm);
      print_csv(out, c.soak_min_depth_cm);
      print_csv(out, c.soak_max_depth_cm);
      print_csv(out, (int32_t)c.soak_duration_ms);
      print_profile(out, c.drop);
      print_profile(out, c.retrieval);
      out.println();
      break;
    }
  }
}

//try to parse a record at addr; returns its total length or 0 if there is no valid record there
static uint8_t read_record(uint32_t addr, uint32_t end, FrameHeader &fh, uint8_t *payload) {
  if (addr + FRAME_OVERHEAD > end) return 0;
  if (!fram_read(addr, reinterpret_cast<uint8_t *>(&fh), sizeof(fh))) return 0;
  if (fh.marker != RECORD_MARKER || fh.len > MAX_PAYLOAD || fh.type > (uint8_t)LogRecordType::CYCLE_SUMMARY) return 0;
  const uint8_t total = (uint8_t)(fh.len + FRAME_OVERHEAD);
  if (addr + total > end) return 0;

  uint8_t crc_bytes[2];
  if (!fram_read(addr + sizeof(fh), payload, fh.len)) return 0;
  if (!fram_read(addr + sizeof(fh) + fh.len, crc_bytes, 2)) return 0;

  uint16_t crc = crc16(0xFFFF, reinterpret_cast<const uint8_t *>(&fh) + 1, sizeof(fh) - 1);
  crc = crc16(crc, payload, fh.len);
  if (crc != (uint16_t)(crc_bytes[0] | (crc_bytes[1] << 8))) return 0;
  return total;
}

//print every valid record in [from, to), resynchronising byte-by-byte over stale or torn data
static void dump_span(Print &out, uint32_t from, uint32_t to) {
  uint8_t payload[MAX_PAYLOAD];
  FrameHeader fh;
  uint32_t addr = from;
  while (addr < to) {
    uint8_t len = read_record(addr, to, fh, payload);
    if (len) {
      print_record(out, fh, payload);
      addr += len;
    } else {
      addr++;
    }
    wdt_reset();
  }
}

//oldest first: after a wrap the oldest data starts just past the write pointer
static void dump_region(Print &out, const Region &rg) {
  uint32_t next = g_hdr.*rg.next;
  if (g_hdr.*rg.wrapped) {
    dump_span(out, next, rg.end);
  }
  dump_span(out, rg.start, next);
}

void logger_dump(Print &out) {
  if (!g_ready) {
    out.println(F("ERR logger not ready"));
    return;
  }
  out.println(F("# BOOT,boot,seq,reset_cause,fw"));
  out.println(F("# EVT,boot,seq,t_ms,code,data0,data1"));
  out.println(F("# CYC,boot,seq,cycle,start_ms,end_ms,bottom_cm,soak_min_cm,soak_max_cm,soak_ms,"
                "drop_ms,drop_n,drop_peak_mg,drop_rms_mg,ret_ms,ret_n,ret_peak_mg,ret_rms_mg"));
  out.println(F("# SMP,boot,seq,t_ms,state,phase,ax,ay,az,temp_raw,p_raw,depth_cm,mv,soc_x100,flags"));
  dump_region(out, JOURNAL);
  dump_region(out, SAMPLES);
  out.println(F("END"));
}

void logger_printInfo(Print &out) {
  if (!g_ready) {
    out.println(F("logger: NOT READY (FRAM missing or unreadable)"));
    return;
  }
  out.print(F("serial: "));
  out.println(g_hdr.board_serial);
  out.print(F("fw: "));
  out.println(DeviceInfo::FW_VERSION_STR);
  out.print(F("boot_count: "));
  out.println(g_hdr.boot_count);
  out.print(F("cycle_count: "));
  out.println(g_hdr.cycle_count);
  out.print(F("fired_latch: "));
  out.println(g_hdr.fired);
  out.print(F("cal: valid="));
  out.print(g_hdr.cal.valid);
  out.print(F(" zero_raw="));
  out.print(g_hdr.cal.zero_raw);
  out.print(F(" cm_per_count="));
  out.println(g_hdr.cal.cm_per_count, 6);
  out.print(F("journal used: "));
  out.print(g_hdr.journal_wrapped ? (JOURNAL.end - JOURNAL.start) : (g_hdr.journal_next - JOURNAL.start));
  out.print('/');
  out.println(JOURNAL.end - JOURNAL.start);
  out.print(F("samples used: "));
  out.print(g_hdr.sample_wrapped ? (SAMPLES.end - SAMPLES.start) : (g_hdr.sample_next - SAMPLES.start));
  out.print('/');
  out.println(SAMPLES.end - SAMPLES.start);
}
