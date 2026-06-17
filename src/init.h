#ifndef _INIT_H_
#define _INIT_H_

#include <Arduino.h>
#include "configuration.h"

// Function declarations (implementations are in init.cpp)
void initSerial();
void initLittleFS();
void initRelays();
String relayState(int numRelay);
void SetRelayLogical(int relayIndex, bool on);
bool GetRelayLogicalState(int relayIndex);
void initNMEA2000();

#endif // _INIT_H_