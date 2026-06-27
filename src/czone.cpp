/*
  czone.cpp
  Ported/cleaned subset of user-supplied CZone handler.
  Uses SetRelayLogical(relayIndex, bool) from project's init.h
*/

#include "czone.h"
#include "configuration.h"
// Avoid including NMEA2000_CAN which creates a global instance in multiple TUs
#include "NMEA2000.h"
#include <LittleFS.h>
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

#if ENABLE_CZONE
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
static const char *CzZcfTmpPath = "/czone_upload.tmp";
static const char *CzZcfFinalPath = "/config.zcf";
static bool CzZcfRxActive = false;
static uint16_t CzZcfNextBlock = 0;
static size_t CzZcfBytesReceived = 0;
static unsigned long CzZcfRxLastMs = 0;
static const unsigned long CzZcfRxTimeoutMs = 4000;
static unsigned long CzLastConfigPushMs = 0;
static const unsigned long CzConfigPushCooldownMs = 8000;
static const unsigned long CzTransferStartMinUptimeMs = 15000;
static const bool CzAllowOutboundConfigTx = false;
static unsigned long CzLastTxRejectMs = 0;
static const unsigned long CzTxRejectCooldownMs = 1000;
static unsigned long CzTransferRejectUntilMs = 0;
static unsigned long CzAnnounceUntilMs = 0;
static unsigned long CzNextAnnounceMs = 0;
static unsigned long CzLast65288ResponseMs = 0;
static const unsigned long Cz65288ResponseMinIntervalMs = 15000;
static uint8_t CzLastReportedState1 = 0xFF;
static uint8_t CzLastReportedState2 = 0xFF;
// ZCF TX state machine (module → MFD block-by-block)
static bool CzZcfTxActive = false;
static uint16_t CzZcfTxBlock = 0;
static unsigned long CzZcfTxLastMs = 0;
static const unsigned long CzZcfTxRetryMs = 2000;
static uint8_t CzZcfTxRetries = 0;
static const uint8_t CzZcfTxRetryMax = 3;
static const uint8_t CzModuleAddress = 0x18;
static const size_t CZoneDataBlockHeaderLen = 23;
static const size_t CZoneDataBlockChunkMax = 200;

// Pending-ack handling for press/release button pairs (Latching/Toggle Output Function
// on B&G Vulcan sends a press frame like 0x71 immediately followed by a release frame
// like 0x40/0x62). The relay is switched immediately on the press frame, but the
// sync-state broadcast (which the MFD reads to update its display) is deferred until
// the matching release frame arrives, or until a timeout, to avoid confusing the MFD
// with an ack sent mid-press-cycle.
static bool CzAckPending = false;
static unsigned long CzAckPendingSince = 0;
static const unsigned long CzAckPendingTimeoutMs = 400;

static uint8_t GetCZoneReplyDestination() {
  return (CzLastRequester == 255) ? 255 : CzLastRequester;
}

static void LearnCZoneHeader(uint16_t hdr) {
  if (hdr == CZoneMessage || hdr == CZoneMessageAlt) {
    CzTxHeader = hdr;
  }
}

static inline bool IsCZoneAbsOn(uint8_t iState) {
  return iState == 0xF1 || iState == 0x01 || iState == 0x11;
}

static inline bool IsCZoneTogglePress(uint8_t iState, uint8_t b7) {
  return iState >= 0x71 && iState <= 0x7F && b7 == 0x08;
}

static inline bool IsCZoneToggleRange(uint8_t iState) {
  return iState >= 0x71 && iState <= 0x7F;
}

static inline bool IsCZoneToggleRelease(uint8_t iState, uint8_t b7) {
  return iState == 0x40 && b7 == 0x08;
}

static inline bool IsCZoneAbsOff(uint8_t iState) {
  return iState == 0xF2 || iState == 0x41 || iState == 0x40 || iState == 0x00 || iState == 0x10;
}

static inline bool IsCZoneReleaseFrame(uint8_t iState) {
  return iState == 0x40 || iState == 0x42 || iState == 0x62;
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
static void SetCZoneModuleClaim65290_CZ();
static void SetCZoneSwitchStateBroadcast130817_CZ(unsigned char CzSwitchBankSerialNum);
static void SetCZoneDataBlockAck65291_CZ(uint8_t target, uint16_t blockIndex, uint8_t status);
static void CzSendOneBlock(uint16_t blockIdx);
static void SendCZoneConfigAs130816_CZ();
static void SyncCZoneStateFromRelays();
#endif // ENABLE_CZONE

void CZone_Init() {
#if ENABLE_CZONE
  // nothing to init presently; handlers are checked by caller
#endif
}

void CZone_Loop() {
#if ENABLE_CZONE
  const unsigned long now = millis();

  SyncCZoneStateFromRelays();

  // Safety net: if a press frame (0x71) set CzAckPending but no matching release frame
  // (0x40/0x62/0x42) arrived within CzAckPendingTimeoutMs, send the deferred ack anyway -
  // otherwise the relay state and the MFD display would stay out of sync indefinitely.
  if (CzAckPending && (long)(now - CzAckPendingSince) >= (long)CzAckPendingTimeoutMs) {
    CzAckPending = false;
    SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
    SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
  }

  // Only announce config/claim in a short request-triggered window to avoid UI churn on CZ pages.
  if ((long)(CzAnnounceUntilMs - now) > 0 && (long)(now - CzNextAnnounceMs) >= 0) {
    CzNextAnnounceMs = now + 2000UL;
    if ((long)(CzTransferRejectUntilMs - now) <= 0) {
      SetCZoneSendConfigToMFD65290_CZ(CzSwitchBank1SerialNum, CzConfig0, CzConfig1, CzConfig2);
    }
    // Claim only when no local config exists; otherwise Vulcan may re-enter
    // "Empfange Konfiguration" repeatedly and stick at 0%.
    if (!LittleFS.exists(CzZcfFinalPath)) {
      SetCZoneModuleClaim65290_CZ();
    }
  }

  if (!CzAllowOutboundConfigTx && CzZcfTxActive) {
    CzZcfTxActive = false;
    CzZcfTxRetries = 0;
    if (N2kDebug) Serial.println("CZone 130816 tx: disabled");
  }

  // ZCF TX retry: resend current block if Vulcan hasn't ACKed within timeout
  if (CzZcfTxActive && (long)(now - CzZcfTxLastMs) >= (long)CzZcfTxRetryMs) {
    CzZcfTxRetries++;
    if (CzZcfTxRetries > CzZcfTxRetryMax) {
      if (N2kDebug) Serial.printf("CZone 130816 tx abort: block=%u retries=%u\n",
                                  (unsigned)CzZcfTxBlock,
                                  (unsigned)CzZcfTxRetries);
      CzZcfTxActive = false;
      CzZcfTxRetries = 0;
    } else {
      if (N2kDebug) Serial.printf("CZone 130816 tx retry: block=%u try=%u\n",
                                  (unsigned)CzZcfTxBlock,
                                  (unsigned)CzZcfTxRetries);
      CzSendOneBlock(CzZcfTxBlock);
    }
  }

  // Abort stale RX transfers so a half-started upload cannot trap the node in "receiving" mode.
  if (CzZcfRxActive && (long)(now - CzZcfRxLastMs) >= (long)CzZcfRxTimeoutMs) {
    if (N2kDebug) {
      Serial.printf("CZone 130816 rx timeout: nextBlock=%u bytes=%u\n",
                    (unsigned)CzZcfNextBlock,
                    (unsigned)CzZcfBytesReceived);
    }
    CzZcfRxActive = false;
    LittleFS.remove(CzZcfTmpPath);
  }
#endif
}

// Return true if message handled by CZone parser
bool CZone_HandleMsg(const tN2kMsg &msg) {
#if !ENABLE_CZONE
  (void)msg;
  return false; // CZone deaktiviert: Nachricht nicht konsumieren, an Standard-Handler weitergeben
#else
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
        if (iState == 0xF1 || iState == 0xF2 || iState == 0x41 || iState == 0x42 || iState == 0x40 ||
            (iState >= 0x71 && iState <= 0x7F) || iState == 0x62 ||
            iState == 0x02 || iState == 0x03) {
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
          bool doToggle = false;
          const uint8_t b7 = msg.Data[7];
          if (IsCZoneAbsOn(iState)) {
            // Common ON encodings seen on CZone pages (Momentary-style absolute ON).
            handleAbs = true;
            absOut = true;
          } else if (IsCZoneTogglePress(iState, b7)) {
            // "Single Throw Latching" tap on B&G Vulcan (Output Function "On/Off" or "Toggle"):
            // one tap toggles the relay state. Always paired with b7=0x08 in this mode.
            // Confirmed by log: the MFD alternates between 0x71 and 0x72 as the press code
            // on successive taps (likely an internal sequence/toggle bit in the protocol).
            // Accept the whole 0x71-0x7F band defensively in case the MFD cycles through
            // further values - all must be treated identically as "toggle", otherwise some
            // taps would be silently dropped. Each press is immediately followed by a
            // release frame (0x62 or 0x40 depending on Output Function setting - both
            // handled as ignore below via the b7==0x08 marker).
            doToggle = true;
          } else if (IsCZoneToggleRange(iState)) {
            // 0x7x without the Latching marker is treated as non-switch traffic.
            ignoreSwitchCommand = true;
          } else if (IsCZoneToggleRelease(iState, b7)) {
            // Release frame following 0x71 when Output Function = "Toggle" (b7=0x08 marks
            // this Latching/Toggle frame family). Must be ignored - it is NOT an OFF command,
            // just the button release, otherwise every tap would toggle twice.
            ignoreSwitchCommand = true;
          } else if (IsCZoneAbsOff(iState)) {
            // Common OFF encodings seen on CZone pages (Momentary-style absolute OFF,
            // b7=0x00 frame family - plain "Single Throw Momentary" Output Function).
            // 0x40 here confirmed as the "OFF" button code on B&G Vulcan in Momentary mode
            // (mirrors 0xF1 for ON, same frame layout b2=05,b3=00,b4=00,b5=01, b7=00).
            handleAbs = true;
            absOut = false;
          } else if (iState == 0x42 || iState == 0x62) {
            // Release/follow-up frame on B&G Vulcan: a single tap on a "Single Throw Latching"
            // button sends 0x71 (press) immediately followed by 0x62 (release) - confirmed by
            // log timing (~325ms apart for one tap). Must be ignored, otherwise every tap
            // would toggle twice (once on press, once on release).
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
              handleAbs ? "ABS" : (doToggle ? "TOGGLE-LATCH" : "IGNORED/OTHER"),
              absOut ? "ON" : "OFF",
              ignoreSwitchCommand ? 1U : 0U);
          }

          if (ignoreSwitchCommand) {
            // If we have a pending ack from a preceding press frame, this release frame
            // is the right moment to send it - the MFD has now seen the full press/release
            // cycle and should accept the status update without re-triggering its own toggle.
            const bool isReleaseFrame = IsCZoneReleaseFrame(iState);
            if (isReleaseFrame && CzAckPending) {
              CzAckPending = false;
              SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
              SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
            }
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
            SetCZoneSwitchChangeAck65283_CZ(CzSwitchBank1SerialNum);
            SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
          } else if (doToggle) {
            // Switch the relay immediately, but defer the ack/status broadcast until the
            // matching release frame (0x40/0x62/0x42) arrives - see CzAckPending above.
            *state ^= mask;
            *syncState ^= mask;
            SetChangeSwitchState_CZ(switchIndex, ((*state & mask) != 0));
            // Sync to global state variables so the next heartbeat reflects the new state
            if (switchIndex <= 4) {
              if ((*state & mask) != 0) CzSwitchState1 |= mask;
              else CzSwitchState1 &= ~mask;
              if ((*syncState & mask) != 0) CzMfdDisplaySyncState1 |= mask;
              else CzMfdDisplaySyncState1 &= ~mask;
            }
            CzAckPending = true;
            CzAckPendingSince = millis();
          } else {
            // Unsupported/unknown command state: never toggle as fallback.
            if (N2kDebug) {
              Serial.printf("CZone 65280 ignore unsupported iState=%02X b7=%02X\n",
                            iState,
                            b7);
            }
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
        if (N2kDebug) {
          Serial.printf("CZone 65290 rx: b2=%02X b3=%02X b4=%02X b5=%02X b6=%02X b7=%02X\n",
                        msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5], msg.Data[6], msg.Data[7]);
        }
        // Do not reject 65290 by dip byte; Vulcan variants use different encodings here.
        // Write-announce frames (seen with b7==0x01) can also appear during startup sync.
        // Do not auto-start 130816 transfer here, otherwise SR-Aktor jumps into
        // "Empfange Konfiguration" directly after reboot and can stall at 0%.
        if (msg.DataLen >= 8 && msg.Data[7] == 0x01) {
          if (!CzAllowOutboundConfigTx) {
            CzAnnounceUntilMs = 0;
            CzTransferRejectUntilMs = millis() + 5000UL;
            SetCZoneDataBlockAck65291_CZ(CzModuleAddress, 0, 1);
            if (N2kDebug) Serial.println("CZone 65290 rx: reject config-load request");
          }
          // If a local config already exists, avoid prolonged announce traffic.
          // This reduces CZ-page flapping/empty transitions on Vulcan.
          if (CzAllowOutboundConfigTx && !LittleFS.exists(CzZcfFinalPath)) {
            CzAnnounceUntilMs = millis() + 4000UL;
            CzNextAnnounceMs = 0;
          } else {
            CzAnnounceUntilMs = 0;
          }
        }
        CzConfigAuthenticated = true;
        if ((long)(CzTransferRejectUntilMs - millis()) <= 0) {
          SetCZoneSendConfigToMFD65290_CZ(CzSwitchBank1SerialNum, CzConfig0, CzConfig1, CzConfig2);
        }
        if (NumberOfSwitches == 8 && (long)(CzTransferRejectUntilMs - millis()) <= 0) {
          SetCZoneSendConfigToMFD65290_CZ(CzSwitchBank2SerialNum, CzConfig0, CzConfig1, CzConfig2);
        }
        return true;
      }
      break;

    case 130816UL: // ZCF datablock transfer (fast packet assembled by NMEA2000 lib)
      {
        if (msg.DataLen < CZoneDataBlockHeaderLen) return true;
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) return false;
        CzLastRequester = msg.Source;
        LearnCZoneHeader(hdr);

        int blockIdxPos = 2;
        const uint16_t blockIndex = msg.Get2ByteUInt(blockIdxPos);
        // Vulcan 130816 payload does not carry a simple dip/target byte at a fixed offset
        // in our observed traces, so do not filter by a guessed target field.
        // Use bank serial byte (b5) as ACK target when available.
        uint8_t target = CzModuleAddress;
        if (msg.DataLen > 5) {
          target = msg.Data[5];
        }

        const size_t chunkLen = (msg.DataLen > CZoneDataBlockHeaderLen) ? (msg.DataLen - CZoneDataBlockHeaderLen) : 0;
        if (N2kDebug) {
          Serial.printf("CZone 130816 rx: block=%u len=%u b4=%02X b5=%02X\n",
                        (unsigned)blockIndex,
                        (unsigned)chunkLen,
                        msg.Data[4],
                        msg.Data[5]);
        }
        if (chunkLen > CZoneDataBlockChunkMax) {
          SetCZoneDataBlockAck65291_CZ(target, blockIndex, 1);
          return true;
        }

        if (!CzZcfRxActive) {
          // Keep normal operation stable once a local config exists: reject unsolicited
          // upload starts instead of entering a potentially endless 0% receive state.
          if (LittleFS.exists(CzZcfFinalPath)) {
            SetCZoneDataBlockAck65291_CZ(target, blockIndex, 1);
            if (N2kDebug) {
              Serial.printf("CZone 130816 rx: reject block=%u (config already present)\n",
                            (unsigned)blockIndex);
            }
            return true;
          }
          if (blockIndex != 0) {
            if (chunkLen == 0) SetCZoneDataBlockAck65291_CZ(target, blockIndex, 0);
            return true;
          }
          LittleFS.remove(CzZcfTmpPath);
          CzZcfRxActive = true;
          CzZcfNextBlock = 0;
          CzZcfBytesReceived = 0;
          CzZcfRxLastMs = millis();
        }

        if (blockIndex < CzZcfNextBlock) {
          SetCZoneDataBlockAck65291_CZ(target, blockIndex, 0);
          return true;
        }
        if (blockIndex > CzZcfNextBlock) {
          return true;
        }

        if (chunkLen > 0) {
          File wf = LittleFS.open(CzZcfTmpPath, "a");
          if (!wf) {
            SetCZoneDataBlockAck65291_CZ(target, blockIndex, 2);
            CzZcfRxActive = false;
            return true;
          }
          for (size_t i = 0; i < chunkLen; i++) {
            int payloadIdx = (int)(CZoneDataBlockHeaderLen + i);
            wf.write(msg.GetByte(payloadIdx));
          }
          wf.close();
          CzZcfBytesReceived += chunkLen;
          CzZcfRxLastMs = millis();
        }

        SetCZoneDataBlockAck65291_CZ(target, blockIndex, 0);
        CzZcfNextBlock++;

        if (chunkLen < CZoneDataBlockChunkMax) {
          LittleFS.remove(CzZcfFinalPath);
          if (!LittleFS.rename(CzZcfTmpPath, CzZcfFinalPath)) {
            if (N2kDebug) Serial.println("CZone 130816: commit rename failed");
          }
          if (N2kDebug) Serial.printf("CZone 130816: RX done bytes=%u blocks=%u\n", (unsigned)CzZcfBytesReceived, (unsigned)CzZcfNextBlock);
          CzZcfRxActive = false;
        }

        return true;
      }
      break;

    case 130822UL: // Vulcan requests config transfer details/chunks
      {
        if (msg.DataLen < 6) return true;
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) return false;
        CzLastRequester = msg.Source;
        LearnCZoneHeader(hdr);
        if (N2kDebug) {
          Serial.printf("CZone 130822 rx: b2=%02X b3=%02X b4=%02X b5=%02X b6=%02X\n",
                        msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5], msg.Data[6]);
        }
        const unsigned long now = millis();
        if (CzAllowOutboundConfigTx && now >= CzTransferStartMinUptimeMs) {
          CzAnnounceUntilMs = now + 4000UL;
          CzNextAnnounceMs = 0;
          SendCZoneConfigAs130816_CZ();
        } else {
          // Vulcan can repeatedly request transfer chunks (130822). If outbound TX is
          // disabled, actively NACK to avoid staying in "Empfang ... 0%".
          uint16_t requestedBlock = 0;
          if (msg.DataLen >= 13) {
            // Observed 130822 layout carries requested block near the end (b11/b12).
            requestedBlock = (uint16_t)msg.Data[11] | ((uint16_t)msg.Data[12] << 8);
          } else if (msg.DataLen >= 7) {
            requestedBlock = (uint16_t)msg.Data[5] | ((uint16_t)msg.Data[6] << 8);
          }
          if ((long)(now - CzLastTxRejectMs) >= (long)CzTxRejectCooldownMs) {
            CzLastTxRejectMs = now;
            SetCZoneDataBlockAck65291_CZ(CzModuleAddress, requestedBlock, 1);
          }
          if (N2kDebug) {
            Serial.printf("CZone 130822 rx: reject tx trigger, block=%u uptime=%lu ms\n",
                          (unsigned)requestedBlock,
                          now);
          }
        }
        return true;
      }
      break;

    case 65291UL: // ACK from Vulcan for our outgoing 130816 block
      {
        if (!CzZcfTxActive) return true;
        if (msg.DataLen < 7) return true;
        int idx = 0;
        const uint16_t hdr = msg.Get2ByteUInt(idx);
        if (hdr != CZoneMessage && hdr != CZoneMessageAlt) return false;
        // Format: hdr(2) + target(1) + status(1) + blockIndex(2) + ...
        const uint8_t status = msg.Data[3];
        int bidx = 4;
        const uint16_t ackedBlock = msg.Get2ByteUInt(bidx);
        if (N2kDebug) {
          Serial.printf("CZone 65291 rx: ackedBlock=%u status=%u\n",
                        (unsigned)ackedBlock, (unsigned)status);
        }
        if (status == 0 && ackedBlock == CzZcfTxBlock) {
          CzZcfTxRetries = 0;
          CzZcfTxBlock++;
          CzSendOneBlock(CzZcfTxBlock);
        }
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
        const unsigned long now = millis();
        const bool stateChanged =
          (CzSwitchState1 != CzLastReportedState1) ||
          (CzSwitchState2 != CzLastReportedState2);
        const bool keepaliveDue =
          (long)(now - CzLast65288ResponseMs) >= (long)Cz65288ResponseMinIntervalMs;
        if (stateChanged || keepaliveDue) {
          CzLast65288ResponseMs = now;
          SetCZoneSwitchHeartbeat65284_CZ(CzSwitchBank1SerialNum);
        }
        return true;
      }
      break;

    default:
      return false;
  }
  return false;
#endif // ENABLE_CZONE
}

#if ENABLE_CZONE
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
  N2kMsg.Destination = 255;
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
  N2kMsg.Add2ByteUInt(CzTxHeader);
  N2kMsg.Destination = 255;
  N2kMsg.AddByte(CzSwitchBankSerialNum);
  // Switch summary field identifier (Contact 6 Plus / N2kCZone parser): 0x1E
  N2kMsg.AddByte(0x1e);
  if (CzSwitchBankSerialNum == CzSwitchBank1SerialNum)
    N2kMsg.AddByte(CzSwitchState1);
  else N2kMsg.AddByte(CzSwitchState2);
  N2kMsg.Add2ByteUInt(0x0000);
  N2kMsg.AddByte(0x00);
  SendCZoneMsgWithMirroredHeader(N2kMsg);

  if (CzSwitchBankSerialNum == CzSwitchBank1SerialNum) CzLastReportedState1 = CzSwitchState1;
  else CzLastReportedState2 = CzSwitchState2;
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

static void SetCZoneModuleClaim65290_CZ() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(65290L);
  N2kMsg.Priority = 7;
  N2kMsg.Destination = 255;
  N2kMsg.Add2ByteUInt(CzTxHeader);
  // 0x03 is the claim style used by CZone modules during config transfer arbitration.
  N2kMsg.AddByte(0x03);
  // Keep config-id simple/non-zero so requester treats us as a valid claimant.
  N2kMsg.Add4ByteUInt(0x00000001UL);
  N2kMsg.AddByte(CzModuleAddress);
  N2kMsg.AddByte(0xFF);
  if (N2kDebug) {
    Serial.printf("CZone TX 65290 module-claim: dip=%u\n", (unsigned)CzModuleAddress);
  }
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
  N2kMsg.Destination = 255;
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

static void SetCZoneDataBlockAck65291_CZ(uint8_t target, uint16_t blockIndex, uint8_t status) {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(65291L);
  N2kMsg.Priority = 7;
  N2kMsg.Destination = GetCZoneReplyDestination();
  N2kMsg.Add2ByteUInt(CzTxHeader);
  N2kMsg.AddByte(target);
  N2kMsg.AddByte(status);
  N2kMsg.Add2ByteUInt(blockIndex);
  N2kMsg.AddByte(CzModuleAddress);
  N2kMsg.AddByte(0xFF);
  if (N2kDebug) {
    Serial.printf("CZone TX 65291 ack: target=%u block=%u status=%u\n",
                  (unsigned)target, (unsigned)blockIndex, (unsigned)status);
  }
  SendCZoneMsgWithMirroredHeader(N2kMsg);
}

static void CzSendOneBlock(uint16_t blockIdx) {
  File rf = LittleFS.open(CzZcfFinalPath, "r");
  if (!rf) {
    if (N2kDebug) Serial.println("CZone TX: open failed");
    CzZcfTxActive = false;
    return;
  }
  const size_t fileSize = (size_t)rf.size();
  const size_t offset = (size_t)blockIdx * CZoneDataBlockChunkMax;
  if (offset >= fileSize) {
    rf.close();
    if (N2kDebug) Serial.printf("CZone 130816 tx done: %u blocks\n", (unsigned)blockIdx);
    CzZcfTxActive = false;
    return;
  }
  rf.seek(offset);
  uint8_t chunk[CZoneDataBlockChunkMax];
  const size_t n = rf.read(chunk, CZoneDataBlockChunkMax);
  rf.close();
  if (n == 0) { CzZcfTxActive = false; return; }

  tN2kMsg out;
  out.SetPGN(130816L);
  out.Priority = 7;
  out.Destination = 255; // broadcast
  out.Add2ByteUInt(CzTxHeader);
  out.Add2ByteUInt(blockIdx);
  out.AddByte(0x00);
  out.AddByte(CzModuleAddress);
  for (int i = 0; i < (int)(CZoneDataBlockHeaderLen - 4) - 2; i++) out.AddByte(0xFF);
  for (size_t i = 0; i < n; i++) out.AddByte(chunk[i]);
  NMEA2000.SendMsg(out);
  CzZcfTxLastMs = millis();
  if (N2kDebug) {
    Serial.printf("CZone 130816 tx: block=%u bytes=%u%s\n",
                  (unsigned)blockIdx, (unsigned)n,
                  (n < CZoneDataBlockChunkMax) ? " [last]" : "");
  }
}

static void SendCZoneConfigAs130816_CZ() {
  if (!CzAllowOutboundConfigTx) {
    CzZcfTxActive = false;
    CzZcfTxRetries = 0;
    return;
  }
  const unsigned long now = millis();
  if ((long)(now - CzLastConfigPushMs) < (long)CzConfigPushCooldownMs) {
    return;
  }
  CzLastConfigPushMs = now;
  if (!LittleFS.exists(CzZcfFinalPath)) {
    if (N2kDebug) Serial.println("CZone 130816 tx: no /config.zcf");
    return;
  }
  if (CzZcfTxActive) {
    if (N2kDebug) Serial.println("CZone 130816 tx: reset active transfer");
  }
  CzZcfTxActive = true;
  CzZcfTxBlock = 0;
  CzZcfTxRetries = 0;
  if (N2kDebug) Serial.println("CZone 130816 tx: start block 0");
  CzSendOneBlock(0);
}
#endif // ENABLE_CZONE
