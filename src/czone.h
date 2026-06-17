/*
  czone.h
  Minimal CZone parser integration header
*/
#ifndef CZONE_H
#define CZONE_H

#include <Arduino.h>
#include <N2kMessages.h>

// Initialize CZone module (optional)
void CZone_Init();

// Periodic CZone housekeeping (heartbeat/state broadcast)
void CZone_Loop();

// Handle an incoming N2k message; return true if handled and no further processing should occur
bool CZone_HandleMsg(const tN2kMsg &msg);

#endif
