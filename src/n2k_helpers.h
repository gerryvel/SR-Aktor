/*
 * n2k_helpers.h
 * Small header collecting NMEA2000 helper prototypes used across the project
 */
#ifndef N2K_HELPERS_H
#define N2K_HELPERS_H

#include <N2kMessages.h>

// Helpers for canonical PGNs used by the project

void SetN2kPGN127502(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus);

void SetN2kPGN126208(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus);

void SetN2kSwitchBankCommand(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus);

void SetN2kGroupFunctionCommand126208(tN2kMsg &N2kMsg, unsigned char DestinationId,
                                      unsigned char FieldNoOfParam, unsigned char FieldValue);

void SR_SetN2kPGN127501(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kOnOff Status1, tN2kOnOff Status2, tN2kOnOff Status3, tN2kOnOff Status4);

// SetSwitch helper
void SetSwitch(unsigned char DeviceBankInstance, uint8_t SwitchIndex, bool ItemStatus);

#endif // N2K_HELPERS_H
