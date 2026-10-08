#ifndef LOGGER_H
#define LOGGER_H

#include "types.h"

bool logger_init(uint8_t reset_cause); //load or format the FRAM layout, count the boot, write a BOOT record
bool logger_ready();

bool logger_appendSample(const SensorSnapshot &sample);
bool logger_logEvent(uint32_t t_ms, EventCode code, uint8_t data0, uint16_t data1);
bool logger_appendCycle(CycleSummary &summary); //assigns summary.cycle_idx

//persistent identity/state kept in the header
bool logger_isFired();
bool logger_setFired(bool fired);
bool logger_getCal(PressureCal &cal);
bool logger_setCal(const PressureCal &cal);
bool logger_setSerial(const char *serial);

bool logger_erase(); //clear all records and leave recording mode (keeps serial, calibration, counters, fired latch)

//power diagnostics: a breadcrumb that survives resets, and a persistent "bridge off" bench option
void logger_markStage(uint8_t stage); //ResetStage; STAGE_NONE when the activity finished
uint8_t logger_resetStage(); //stage the previous boot died in (STAGE_NONE = clean)
bool logger_bridgeEnabled();
bool logger_setBridgeEnabled(bool enabled);

//standalone recording: erases the logs, then every record goes to one linear area until full
bool logger_recStart(uint8_t hz);
bool logger_recStop(); //keep the recording (HOLDING) until logger_erase()
bool logger_appendRecBlock(const RecBlock &block);
RecMode logger_recMode();
uint8_t logger_recHz();
void logger_printRecStatus(Print &out);
void logger_dump(Print &out); //every record, oldest first, as CSV lines; ends with "END"
void logger_printInfo(Print &out);

#endif
