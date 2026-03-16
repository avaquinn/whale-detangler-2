/*
  Data logging layer for FRAM
  - Defines log record formats (samples/events) and a storage strategy
  - Writes sensor snapshots, detector decisions, and state transitions to FRAM
  - Provides initialization, append/write helpers, and optional readback/debug dump utilities
*/
#include <Arduino.h>
#include <string.h>
#include "logger.h"
#include "FRAM.h"
#include "config.h"

//physical FRAM capacity for this board (2 Mbit => 262,144 bytes)
static constexpr uint32_t FRAM_TOTAL_BYTES = 262144UL;
//magic value used to validate logger metadata at FRAM address 0
static constexpr uint32_t LOG_META_MAGIC = 0x57444C47UL; // "WDLG"
//fixed marker byte at the start of each record frame
static constexpr uint8_t LOG_RECORD_MARKER = 0xA5;

//persistent metadata written in FRAM header region
//allows logging to continue after power loss/reboot without erasing data
struct __attribute__((packed)) LoggerMeta {
  uint32_t magic;
  uint32_t write_addr;
  uint8_t boot_written;
  uint8_t reserved[7];
};

// Per-record framing for mixed record types in a single stream
struct __attribute__((packed)) RecordHeader {
  uint8_t marker;
  uint8_t type;
  uint16_t len;
};

// Runtime copy of logger metadata loaded from FRAM.
static LoggerMeta g_meta = {};
// Guard to prevent append calls before initialization.
static bool g_logger_initialized = false;

// Validate loaded metadata before trusting persistent write pointer.
static bool meta_is_valid(const LoggerMeta &m) {
  if (m.magic != LOG_META_MAGIC) return false;
  if (m.write_addr < Logging::DATA_START_ADDR) return false;
  if (m.write_addr >= FRAM_TOTAL_BYTES) return false;
  return true;
}

// Persist in-memory metadata to FRAM.
static bool meta_store() {
  return fram_write(0, reinterpret_cast<const uint8_t*>(&g_meta), sizeof(g_meta));
}

// Append a framed payload to FRAM with simple ring-buffer wrap at end-of-device.
// Data persistence behavior:
// - Existing log contents are never erased during init.
// - New appends advance write_addr and overwrite oldest area only after wrap.
static bool record_append(LogRecordType type, const void *payload, uint16_t payload_len) {
  if (!g_logger_initialized) {
    return false;
  }

  const uint32_t record_len = sizeof(RecordHeader) + payload_len;
  if (record_len > (FRAM_TOTAL_BYTES - Logging::DATA_START_ADDR)) {
    return false;
  }

  uint32_t addr = g_meta.write_addr;
  if (addr + record_len > FRAM_TOTAL_BYTES) {
    addr = Logging::DATA_START_ADDR;
  }

  RecordHeader hdr = {LOG_RECORD_MARKER, static_cast<uint8_t>(type), payload_len};
  if (!fram_write(addr, reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr))) {
    return false;
  }
  if (payload_len > 0 && !fram_write(addr + sizeof(hdr), reinterpret_cast<const uint8_t*>(payload), payload_len)) {
    return false;
  }

  g_meta.write_addr = addr + record_len;
  if (g_meta.write_addr >= FRAM_TOTAL_BYTES) {
    g_meta.write_addr = Logging::DATA_START_ADDR;
  }
  return meta_store();
}

// Initialize logger and recover persistent write pointer from FRAM metadata.
bool logger_init() {
  fram_init();

  LoggerMeta loaded = {};
  if (!fram_read(0, reinterpret_cast<uint8_t*>(&loaded), sizeof(loaded))) {
    return false;
  }

  if (!meta_is_valid(loaded)) {
    g_meta = {};
    g_meta.magic = LOG_META_MAGIC;
    g_meta.write_addr = Logging::DATA_START_ADDR;
    g_meta.boot_written = 0;
    if (!meta_store()) {
      return false;
    }
    g_logger_initialized = true;
    return true;
  }

  g_meta = loaded;
  g_logger_initialized = true;
  return true;
}

// Write one BootRecord only once per FRAM image.
// This keeps boot identity metadata present without repeating each reset.
bool logger_writeBootRecordOnce(uint32_t t_ms) {
  if (!g_logger_initialized) {
    return false;
  }

  if (g_meta.boot_written) {
    return true;
  }

  BootRecord rec = {};
  rec.t_ms = t_ms;

  strncpy(rec.board_serial, DeviceInfo::BOARD_SERIAL_DEFAULT, sizeof(rec.board_serial) - 1);
  rec.board_serial[sizeof(rec.board_serial) - 1] = '\0';

  strncpy(rec.fw_version, DeviceInfo::FW_VERSION_STR, sizeof(rec.fw_version) - 1);
  rec.fw_version[sizeof(rec.fw_version) - 1] = '\0';

  if (!record_append(LogRecordType::BOOT, &rec, sizeof(rec))) {
    return false;
  }

  g_meta.boot_written = 1;
  return meta_store();
}

// Append sensor sample payload.
bool logger_appendSample(const SensorSnapshot &sample) {
  return record_append(LogRecordType::SAMPLE, &sample, sizeof(sample));
}

// Append event payload.
bool logger_appendEvent(const EventRecord &event) {
  return record_append(LogRecordType::EVENT, &event, sizeof(event));
}

// Helper for building and appending an EventRecord in one call.
bool logger_logEvent(uint32_t t_ms, EventCode code, uint8_t data0, uint16_t data1) {
  EventRecord event = {};
  event.t_ms = t_ms;
  event.code = code;
  event.data0 = data0;
  event.data1 = data1;
  return logger_appendEvent(event);
}
