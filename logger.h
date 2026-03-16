#ifndef LOGGER_H
#define LOGGER_H

#include "types.h"

bool logger_init();
bool logger_writeBootRecordOnce(uint32_t t_ms);
bool logger_appendSample(const SensorSnapshot &sample);
bool logger_appendEvent(const EventRecord &event);
bool logger_logEvent(uint32_t t_ms, EventCode code, uint8_t data0, uint16_t data1);

#endif