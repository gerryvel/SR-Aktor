/*
  czone.cpp
  Ported/cleaned subset of user-supplied CZone handler.
  Uses SetRelayLogical(relayIndex, bool) from project's init.h
*/

#include "czone.h"
#include "configuration.h"
// Avoid including NMEA2000_CAN which creates a global instance in multiple TUs
#include "NMEA2000.h"
extern tNMEA2000 &NMEA2000;

// Relay helpers are implemented in init.h
void SetRelayLogical(int relayIndex, bool on);
bool GetRelayLogicalState(int relayIndex);

// Use central N2K helper prototypes
#include "n2k_helpers.h"

// CZone constants (from the supplied snippet)
#define CZoneMessage 0x9927
#define CZoneMessageAlt 0x9913
#define CzSwitchBank1SerialNumDefault  0x19
#define CzSwitchBank2SerialNumDefault  0x1b
#define CzDipSwitch     200
#define BinaryDeviceInstance 0x00
#define SwitchBankInstance 0x00
#define NumberOfSwitches 4
#define CzConfig0 0x05
#define CzConfig1 0x03
#define CzConfig2 0x0F

// Internal states
static uint8_t CzSwitchState1 = 0;
static uint8_t CzSwitchState2 = 0;
static uint8_t CzMfdDisplaySyncState1 = 0;
static uint8_t CzMfdDisplaySyncState2 = 0;
static bool CzConfigAuthenticated = false;
static uint8_t CzSwitchBank1SerialNum = CzSwitchBank1SerialNumDefault;
static uint8_t CzSwitchBank2SerialNum = CzSwitchBank2SerialNumDefault;
static uint16_t CzTxHeader = CZoneMessage;
static uint8_t CzLastRequester = 255;

static uint8_t GetCZoneReplyDestination() {
  return (CzLastRequester == 255) ? 255 : CzLastRequester;
}

static void LearnCZoneHeader(uint16_t hdr) {
  if (hdr == CZoneMessage || hdr == CZoneMessageAlt) {
    CzTxHeader = hdr;
  }
}

static void SendCZoneMsgWithMirroredHeader(const tN2kMsg &msg) {
  NMEA2000.SendMsg(msg);
  if (msg.DataLen < 2) return;

  const uint16_t altHeader = (CzTxHeader == CZoneMessage) ? CZoneMessageAlt : CZoneMessage;
  tN2kMsg mirror = msg;
  mirror.Data[0] = (uint8_t)(altHeader & 0xFF);
  mirror.Data[1] = (uint8_t)((altHeader >> 8) & 0xFF);
  NMEA2000.SendMsg(mirror);
}

// Forward declarations for local helpers
static void SetChangeSwitchState_CZ(uint8_t SwitchIndex, bool ItemStatus);
static void SetCZoneSwitchState127501_CZ(unsigned char DeviceInstance);
static void SetCZoneSwitchChangeRequest127502_CZ(unsigned char DeviceInstance, uint8_t SwitchIndex, bool ItemStatus);
static void SetCZoneSwitchChangeAck65283_CZ(unsigned char CzSwitchBankSerialNum);
static void SetCZoneSwitchHeartbeat65284_CZ(unsigned char CzSwitchBankSerialNum);
static void SetCZoneSendConfigToMFD65290_CZ(unsigned char CzSwitchBankSerial, uint8_t CZoneConfig0, uint8_t CZoneConfig1, uint8_t CZoneConfig2);
static void SetCZoneSwitchStateBroadcast130817_CZ(unsigned char CzSwitchBankSerialNum);
static void SyncCZoneStateFromRelays();

void CZone_Init() {
  // nothing to init presently; handlers are checked by caller
}

void CZone_Loop() {
  static unsigned long nextHeartbeat = 0;
  static unsigned long nextStateBroadcast = 0;
  const unsigned long now = millis();

  SyncCZoneStateFromRelays();

  if ((long)(now - nextHeartbeat) >= 0) {
    nextHeartbeat = now + 1000UL;
    SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
    SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
  }

  if ((long)(now - nextStateBroadcast) >= 0) {
    nextStateBroadcast = now + 500UL;
    SetCZoneSwitchStateBroadcast130817_CZ(CzSwitchBank1SerialNum);
  }
}

// Return true if message handled by CZone parser
bool CZone_HandleMsg(const tN2kMsg &msg) {
  // Only handle known PGNs
  switch (msg.PGN) {
    case 65280UL: // MFD -> switch change
      {
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) {
          if (N2kDebug) Serial.printf("CZone 65280: wrong header %04X\n", msg.Get2ByteUInt(idx));
          return false;
        }
        CzLastRequester = msg.Source;
        LearnCZoneHeader(hdr);
        // Log what the MFD actually sends so we can verify the dip/state bytes
        if (N2kDebug) {
          Serial.printf("CZone 65280: b2=%02X b3=%02X b4=%02X b5=%02X b6=%02X b7=%02X\n",
            msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5], msg.Data[6], msg.Data[7]);
        }
        idx = 6; // state byte
        uint8_t iState = msg.GetByte(idx);
        if (iState == 0xF1 || iState == 0xF2 || iState == 0x41 || iState == 0x42 || iState == 0x40 || iState == 0x02 || iState == 0x03) {
          const uint8_t bit2 = msg.Data[2];
          const uint8_t bit3 = msg.Data[3];
          const uint8_t bankFromMsg = msg.Data[5];
          uint8_t bit = 0;

          // Learn active bank serial from commands sent by MFD (commonly in b5).
          if (bankFromMsg != 0x00 && bankFromMsg != 0xFF && bankFromMsg != CzSwitchBank1SerialNum) {
            if (N2kDebug) Serial.printf("CZone: learn bank serial old=%u new=%u\n", (unsigned)CzSwitchBank1SerialNum, (unsigned)bankFromMsg);
            CzSwitchBank1SerialNum = bankFromMsg;
          }

          // Vulcan variant (header 0x9913): prefer b3 as legacy switch code (0x05..0x0C).
          // Some frames encode selector in b2 as 0x04..0x0B, so keep that as fallback.
          if (hdr == CZoneMessageAlt && bit3 >= 0x05 && bit3 <= 0x0C) {
            bit = bit3;
          } else if (hdr == CZoneMessageAlt && bit2 >= 0x04 && bit2 <= 0x0B) {
            bit = bit2 + 1; // normalize to legacy 0x05..0x0C mapping
          } else if (bit2 >= 0x05 && bit2 <= 0x0C) {
            bit = bit2;
          } else if (bit3 >= 0x05 && bit3 <= 0x0C) {
            bit = bit3;
          } else {
            return true; // handled but unknown bit layout
          }

          uint8_t switchIndex = 0;
          switch (bit) {
            case 0x05: switchIndex = 1; break;
            case 0x06: switchIndex = 2; break;
            case 0x07: switchIndex = 3; break;
            case 0x08: switchIndex = 4; break;
            case 0x09: switchIndex = 5; break;
            case 0x0A: switchIndex = 6; break;
            case 0x0B: switchIndex = 7; break;
            case 0x0C: switchIndex = 8; break;
            default: return true; // handled but unknown bit
          }

          uint8_t *state = (switchIndex <= 4) ? &CzSwitchState1 : &CzSwitchState2;
          uint8_t *syncState = (switchIndex <= 4) ? &CzMfdDisplaySyncState1 : &CzMfdDisplaySyncState2;
          uint8_t mask = (switchIndex <= 4) ? (1 << (switchIndex - 1)) : (1 << (switchIndex - 5));

          bool handleAbs = false;
          bool absOut = false;
          bool ignoreSwitchCommand = false;
          if (iState == 0xF1 || iState == 0x01 || iState == 0x11) {
            // Common ON encodings seen on CZone pages.
            handleAbs = true;
            absOut = true;
          } else if (iState == 0xF2 || iState == 0x41 || iState == 0x00 || iState == 0x10) {
            // Common OFF encodings seen on CZone pages.
            handleAbs = true;
            absOut = false;
          } else if (iState == 0x42) {
            // Seen as follow-up/release frame on some MFDs; ignore to avoid false re-trigger.
            ignoreSwitchCommand = true;
          } else if (iState == 0x02 || iState == 0x03) {
            if (hdr == CZoneMessageAlt) {
              // 0x9913 with 0x02/0x03 appears periodically as display sync on Vulcan.
              // Do not actuate relays from these frames to avoid auto-retrigger after local OFF.
              ignoreSwitchCommand = true;
            } else {
              // On non-0x9913 variants still allow explicit absolute state from b4/b5 if present.
              const uint8_t b4 = msg.Data[4];
              const uint8_t b5 = msg.Data[5];
              if (b4 <= 0x01) {
                handleAbs = true;
                absOut = (b4 == 0x01);
              } else if (b5 <= 0x01) {
                handleAbs = true;
                absOut = (b5 == 0x01);
              }
            }
          }

          if (N2kDebug) {
            Serial.printf("CZone 65280 decode: sw=%u iState=%02X b4=%02X b5=%02X mode=%s out=%s ignore=%u\n",
              (unsigned)switchIndex,
              iState,
              msg.Data[4],
              msg.Data[5],
              handleAbs ? "ABS" : "TOGGLE",
              absOut ? "ON" : "OFF",
              ignoreSwitchCommand ? 1U : 0U);
          }

          if (ignoreSwitchCommand) {
            return true;
          }

          if (handleAbs) {
            if (absOut) {
              *state |= mask;
              *syncState |= mask;
            } else {
              *state &= ~mask;
              *syncState &= ~mask;
            }
            SetChangeSwitchState_CZ(switchIndex, absOut);
            // Send updated state immediately after command
            SetCZoneSwitchStateBroadcast130817_CZ(CzSwitchBank1SerialNum);
            SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
            SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
          } else if (iState != 0x40) {
            // Fallback for undocumented command states: behave as toggle.
            *state ^= mask;
            *syncState ^= mask;
            SetChangeSwitchState_CZ(switchIndex, ((*state & mask) != 0));
            SetCZoneSwitchStateBroadcast130817_CZ(CzSwitchBank1SerialNum);
            SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
            SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
          } else if (iState == 0x40) {
            // end-of-change ack from MFD -> send display sync ack
            if (bit > 0x08) SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank2SerialNum);
            else SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
          }
          return true;
        }
      }
      break;

    case 65290UL: // Config Request from MFD
      {
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) return false;
        CzLastRequester = msg.Source;
        LearnCZoneHeader(hdr);
        idx = 7;
        if (CzDipSwitch != msg.GetByte(idx)) return false;
        SetCZoneSendConfigToMFD65290_CZ(CzSwitchBank1SerialNum, CzConfig0, CzConfig1, CzConfig2);
        SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
        SetCZoneSwitchStateBroadcast130817_CZ(CzSwitchBank1SerialNum);
        if (NumberOfSwitches == 8) SetCZoneSendConfigToMFD65290_CZ(CzSwitchBank2SerialNum, CzConfig0, CzConfig1, CzConfig2);
        return true;
      }
      break;

    case 65284UL: // MFD heartbeat/auth
      {
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) return false;
        CzLastRequester = msg.Source;
        LearnCZoneHeader(hdr);
        // Identify message origin/authentication
        CzConfigAuthenticated = true;
        return true;
      }
      break;

    case 65288UL:
      // MFD sync/auth request: answer with config and current state so the UI can show status.
      {
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) return false;
        CzLastRequester = msg.Source;
        LearnCZoneHeader(hdr);
        CzConfigAuthenticated = true;
        SetCZoneSendConfigToMFD65290_CZ(CzSwitchBank1SerialNum, CzConfig0, CzConfig1, CzConfig2);
        SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
        SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
        SetCZoneSwitchStateBroadcast130817_CZ(CzSwitchBank1SerialNum);
        return true;
      }
      break;

    default:
      return false;
  }
  return false;
}

static void SyncCZoneStateFromRelays() {
  CzSwitchState1 = 0;
  CzMfdDisplaySyncState1 = 0;
  CzSwitchState2 = 0;
  CzMfdDisplaySyncState2 = 0;

  for (uint8_t relayIndex = 0; relayIndex < NUM_RELAYS; relayIndex++) {
    if (!GetRelayLogicalState(relayIndex)) continue;
    const uint8_t mask = 1 << relayIndex;
    CzSwitchState1 |= mask;
    CzMfdDisplaySyncState1 |= mask;
  }
}

// Helpers: when state changed, set real relay (via project's SetRelayLogical) and send N2k compat messages
static void SetChangeSwitchState_CZ(uint8_t SwitchIndex, bool ItemStatus) {
  // Map SwitchIndex (1..8) to local relay indices: we only have 3 relays -> map 1..3
  if (SwitchIndex >= 1 && SwitchIndex <= NUM_RELAYS) {
    // During boot-block do not actuate relays; only seed internal state
    if (millis() > BootBlockUntil) {
      SetRelayLogical(SwitchIndex - 1, ItemStatus);
    } else {
      if (N2kDebug) Serial.printf("CZone: boot-block active, seeding state for switch %u -> %s\n", (unsigned)SwitchIndex, ItemStatus?"ON":"OFF");
    }
  }
  // notify network only after boot-block to avoid noisy startup bus actions
  if (millis() > BootBlockUntil) {
    SetCZoneSwitchState127501_CZ(BinaryDeviceInstance);
    SetCZoneSwitchChangeRequest127502_CZ(SwitchBankInstance, SwitchIndex, ItemStatus);
    SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
    SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
    SetCZoneSwitchStateBroadcast130817_CZ(CzSwitchBank1SerialNum);
  }
}

// The following functions build and send N2k messages similarly to the original code but use
// the NMEA2000 helper functions already present in the main project (SetN2kPGN127501 etc.)
static void SetCZoneSwitchState127501_CZ(unsigned char DeviceInstance) {
  tN2kMsg N2kMsg;
  tN2kBinaryStatus BankStatus;
  N2kResetBinaryStatus(BankStatus);
  const bool s1 = GetRelayLogicalState(0);
  const bool s2 = GetRelayLogicalState(1);
  const bool s3 = GetRelayLogicalState(2);

  const bool report1 = InvertN2kStatus ? !s1 : s1;
  const bool report2 = InvertN2kStatus ? !s2 : s2;
  const bool report3 = InvertN2kStatus ? !s3 : s3;

  N2kSetStatusBinaryOnStatus(BankStatus, report1 ? N2kOnOff_On : N2kOnOff_Off, 1);
  N2kSetStatusBinaryOnStatus(BankStatus, report2 ? N2kOnOff_On : N2kOnOff_Off, 2);
  N2kSetStatusBinaryOnStatus(BankStatus, report3 ? N2kOnOff_On : N2kOnOff_Off, N2K_THIRD_SWITCH_ITEM);

  SetN2kPGN127501(N2kMsg, DeviceInstance, BankStatus);
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}

static void SetCZoneSwitchChangeRequest127502_CZ(unsigned char DeviceInstance, uint8_t SwitchIndex, bool ItemStatus)
{
  tN2kMsg N2kMsg;
  tN2kBinaryStatus status;
  N2kResetBinaryStatus(status);
  N2kSetStatusBinaryOnStatus(status, ItemStatus ? N2kOnOff_On : N2kOnOff_Off, SwitchIndex);
  SetN2kPGN127502(N2kMsg, SwitchBankInstance, status);
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}

static void SetCZoneSwitchChangeAck65283_CZ(unsigned char CzSwitchBankSerialNum) {
  tN2kMsg N2kMsg;
  if (N2kDebug) Serial.printf("CZone TX 65283 bank=%u sync1=%02X sync2=%02X\n",
    (unsigned)CzSwitchBankSerialNum,
    CzMfdDisplaySyncState1,
    CzMfdDisplaySyncState2);
  N2kMsg.SetPGN(65283L);
  N2kMsg.Priority = 7;
  N2kMsg.Destination = GetCZoneReplyDestination();
  N2kMsg.Add2ByteUInt(CzTxHeader);
  N2kMsg.AddByte(CzSwitchBankSerialNum);
  if (CzSwitchBankSerialNum == CzSwitchBank1SerialNum) N2kMsg.AddByte(CzMfdDisplaySyncState1);
  else  N2kMsg.AddByte(CzMfdDisplaySyncState2);
  N2kMsg.Add2ByteUInt(0x0000);
  N2kMsg.AddByte(0x00);
  N2kMsg.AddByte(0x10);
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}

static void SetCZoneSwitchHeartbeat65284_CZ(unsigned char CzSwitchBankSerialNum) {
  tN2kMsg N2kMsg;
  if (N2kDebug) Serial.printf("CZone TX 65284 bank=%u state1=%02X state2=%02X\n",
    (unsigned)CzSwitchBankSerialNum,
    CzSwitchState1,
    CzSwitchState2);
  N2kMsg.SetPGN(65284L);
  N2kMsg.Priority = 7;
  N2kMsg.Destination = GetCZoneReplyDestination();
  N2kMsg.Add2ByteUInt(CzTxHeader);
  N2kMsg.AddByte(CzSwitchBankSerialNum);
  N2kMsg.AddByte(0x0f);
  if (CzSwitchBankSerialNum == CzSwitchBank1SerialNum)
    N2kMsg.AddByte(CzSwitchState1);
  else N2kMsg.AddByte(CzSwitchState2);
  N2kMsg.Add2ByteUInt(0x0000);
  N2kMsg.AddByte(0x00);
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}

static void SetCZoneSendConfigToMFD65290_CZ(unsigned char CzSwitchBankSerial, uint8_t CZoneConfig0, uint8_t CZoneConfig1, uint8_t CZoneConfig2) {
  tN2kMsg N2kMsg;
  if (N2kDebug) Serial.printf("CZone TX 65290 bank=%u cfg=%02X,%02X,%02X auth=%u\n",
    (unsigned)CzSwitchBankSerial,
    CZoneConfig0,
    CZoneConfig1,
    CZoneConfig2,
    CzConfigAuthenticated ? 1U : 0U);
  N2kMsg.SetPGN(65290L);
  N2kMsg.Priority = 7;
  N2kMsg.Add2ByteUInt(CzTxHeader);
  N2kMsg.Destination = GetCZoneReplyDestination();
  N2kMsg.AddByte(CZoneConfig0);
  N2kMsg.AddByte(CZoneConfig1);
  N2kMsg.AddByte(CZoneConfig2);
  N2kMsg.Add2ByteUInt(0x0000);
  N2kMsg.AddByte(CzSwitchBankSerial);
  CzConfigAuthenticated = true;
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}

static void SetCZoneSwitchStateBroadcast130817_CZ(unsigned char CzSwitchBankSerialNum) {
  tN2kMsg N2kMsg;
  if (N2kDebug) Serial.printf("CZone TX 130817 bank=%u state1=%02X state2=%02X\n",
    (unsigned)CzSwitchBankSerialNum,
    CzSwitchState1,
    CzSwitchState2);
  N2kMsg.SetPGN(130817L);
  N2kMsg.Priority = 7;
  N2kMsg.Add2ByteUInt(CzTxHeader);
  N2kMsg.Destination = GetCZoneReplyDestination();
  N2kMsg.AddByte(0x01);
  if (CzSwitchBankSerialNum == CzSwitchBank1SerialNum) {
    N2kMsg.AddByte(CzSwitchBank1SerialNum);
    for (uint8_t i = 0; i < 4; i++) {
      if (CzSwitchState1 & (1 << i)) N2kMsg.AddByte(0x01);
      else N2kMsg.AddByte(0x00);
      N2kMsg.Add2ByteUInt(0x0000);
    }
  } else {
    N2kMsg.AddByte(CzSwitchBank2SerialNum);
    for (uint8_t i = 0; i < 4; i++) {
      if (CzSwitchState2 & (1 << i)) N2kMsg.AddByte(0x01);
      else N2kMsg.AddByte(0x00);
      N2kMsg.Add2ByteUInt(0x0000);
    }
  }
  N2kMsg.Add3ByteInt(0);
  N2kMsg.Add3ByteInt(0);
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}
