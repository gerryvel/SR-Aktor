/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/**
 * @file sr-aktor.ino
 * @author Gerry Sebb
 * @brief Binary Switch NMEA2000
 * @version 1.0
 * @date 2026-06-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include "configuration.h"
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <ESP_WiFi.h>
#include <ESPAsyncWebServer.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <ESPmDNS.h>
#include <arpa/inet.h>
#include "BoardInfo.h"
#include "czone.h"
#include "helper.h"
#include "web.h"
#include "LEDindicator.h"
#include "init.h"


/******************************************* Setup *******************************************************/

// Circular buffer for raw N2k messages (store small history)
N2kRawEntry N2kRawBuf[N2KRAW_MAX];
uint8_t N2kRawHead = 0;
// Proprietary mapping storage (in RAM, persisted via writeConfig when changed)
ProprietaryMapping PropMappings[MAX_PROP_MAPPINGS];
uint8_t PropMappingCount = 0;

DetectedEvent DetectedEvents[MAX_DETECTED_EVENTS];
uint8_t DetectedEventCount = 0;

// Push an incoming raw N2k message into the circular buffer for web debug
void PushRawN2k(const tN2kMsg &msg) {
  N2kRawHead = (N2kRawHead + 1) % N2KRAW_MAX;
  N2kRawEntry &e = N2kRawBuf[N2kRawHead];
  e.pgn = msg.PGN;
  e.src = msg.Source;
  e.dst = msg.Destination;
  e.len = msg.DataLen;
  for (uint8_t i = 0; i < 8; i++) e.data[i] = (i < e.len) ? msg.Data[i] : 0xFF;
}

// Handler for incoming NMEA2000 messages. Registered in setup().
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  if (N2kDebug) {
    Serial.print("Received PGN: "); Serial.println(N2kMsg.PGN);
    N2kMsg.Print(&Serial);
  }
  // Push raw message into ring buffer for web debug and save last received PGN for web inspection
  PushRawN2k(N2kMsg);
  LastReceivedN2kPGN = N2kMsg.PGN;
  LastReceivedN2kText = String("PGN:") + String(N2kMsg.PGN) + String(" Src:") + String(N2kMsg.Source) + String(" Dst:") + String(N2kMsg.Destination);

  if (CZone_HandleMsg(N2kMsg)) {
    return;
  }

#if ENABLE_CZONE
  // In CZone mode, relay control must come from the CZone parser only.
  // Processing standard 127502/126208 commands in parallel can create
  // competing command loops on some MFDs (observed as repeated toggling).
  if (N2kMsg.PGN == 127502L || N2kMsg.PGN == 126208L) {
    if (N2kDebug) {
      Serial.printf("Ignore standard control PGN %lu while CZone is active\n", N2kMsg.PGN);
    }
    return;
  }
#endif

  // Direct Switch Bank Control (PGN 127502)
  if (N2kMsg.PGN == 127502L) {
    unsigned char TargetBankInstance = 0;
    tN2kBinaryStatus BankStatus = 0;
    if (ParseN2kPGN127502(N2kMsg, TargetBankInstance, BankStatus)) {
    // Accept both instance 0 and 1 for compatibility with different MFD mappings.
    // or accept if broadcast and AcceptBroadcastCommands is enabled
  // Some devices use Destination==255 for global broadcast - accept both 0 and 255
  bool isBroadcast = (N2kMsg.Destination == 0 || N2kMsg.Destination == 255);
  if (TargetBankInstance == 0 || TargetBankInstance == 1 || (isBroadcast && AcceptBroadcastCommands)) {
        tN2kOnOff s1 = N2kGetStatusOnBinaryStatus(BankStatus, 1);
        tN2kOnOff s2 = N2kGetStatusOnBinaryStatus(BankStatus, 2);
      tN2kOnOff s3 = N2kGetStatusOnBinaryStatus(BankStatus, N2K_THIRD_SWITCH_ITEM);
      tN2kOnOff s3Legacy = N2kGetStatusOnBinaryStatus(BankStatus, 3);

        bool on1 = (s1 == N2kOnOff_On);
        bool on2 = (s2 == N2kOnOff_On);
      bool on3 = (s3 == N2kOnOff_On) || (s3Legacy == N2kOnOff_On);

        SetRelayLogical(0, on1);
        SetRelayLogical(1, on2);
        SetRelayLogical(2, on3);
        // Update status vars and send an immediate status report
        Rel1Status = GetRelayLogicalState(0);
        Rel2Status = GetRelayLogicalState(1);
        Rel3Status = GetRelayLogicalState(2);
        SendN2kSwitchBankStatus(Rel1Status, Rel2Status, Rel3Status);
      }
    }
    return;
  }

  // Group Function (PGN 126208) - handle Command for PGN 127502 and 127501
  // (Manche MFDs/Maretron-kompatible Stacks kommandieren die Status-PGN 127501
  //  direkt statt der Control-PGN 127502 - siehe NMEA Diskussion dazu.)
  if (N2kMsg.PGN == 126208L) {
    tN2kGroupFunctionCode gf;
    unsigned long PGNForGroupFunction;
    if (tN2kGroupFunctionHandler::Parse(N2kMsg, gf, PGNForGroupFunction)) {
      if ((PGNForGroupFunction == 127502L || PGNForGroupFunction == 127501L) && gf == N2kgfc_Command) {
        uint8_t PrioritySetting = 0;
        uint8_t NumberOfParameterPairs = 0;
        if (tN2kGroupFunctionHandler::ParseCommandParams(N2kMsg, PrioritySetting, NumberOfParameterPairs)) {
          int Index = 0;
          if (tN2kGroupFunctionHandler::StartParseCommandPairParameters(N2kMsg, Index)) {
            // Allow broadcast commands when enabled - some send Dst==0, others Dst==255
            bool isBroadcast = (N2kMsg.Destination == 0 || N2kMsg.Destination == 255);
            if (N2kMsg.Destination != NodeAddress && !(isBroadcast && AcceptBroadcastCommands)) {
              // Not for us
              return;
            }
            for (uint8_t p = 0; p < NumberOfParameterPairs; p++) {
              uint8_t FieldNo = N2kMsg.GetByte(Index);
              uint8_t FieldValue = N2kMsg.GetByte(Index);
              // Field 1 = Bank Instance (skip), Felder 2..4 = Switch 1..3 Status in beiden PGNs
              if (FieldNo >= 2 && FieldNo <= 4) {
                bool on = (FieldValue != 0);
                SetRelayLogical(FieldNo - 2, on);
              } else if (FieldNo >= 1 && FieldNo <= 3) {
                // Fallback fuer Stacks, die wie urspruenglich ab Feld 1 zaehlen
                bool on = (FieldValue != 0);
                SetRelayLogical(FieldNo - 1, on);
              }
            }
            // Update status vars and send an immediate status report
            Rel1Status = GetRelayLogicalState(0);
            Rel2Status = GetRelayLogicalState(1);
            Rel3Status = GetRelayLogicalState(2);
            SendN2kSwitchBankStatus(Rel1Status, Rel2Status, Rel3Status);
            if (N2kDebug) {
              Serial.printf("GroupFunction Command auf PGN %lu verarbeitet\n", PGNForGroupFunction);
            }
          }
        }
      }
    }
    return;
  }
}

void setup() {

  initSerial();
  delay(2000);
  initLittleFS();

	/**
	 * @brief file exists, reading and loading config file
	 * 
	 */
  readConfig("/config.json");
    IP = inet_addr(tAP_Config.wAP_IP);
    AP_SSID = tAP_Config.wAP_SSID;
    AP_PASSWORD = tAP_Config.wAP_Password;
    sRelay1Name = tAP_Config.wRelay1Name;
    sRelay2Name = tAP_Config.wRelay2Name;
    sRelay3Name = tAP_Config.wRelay3Name;
    Serial.println("\nConfigdata : AP IP: " + IP.toString() + ", AP SSID: " + AP_SSID + " , Passwort: " + AP_PASSWORD + " , Relais1: " + sRelay1Name + " , Relais2: " + sRelay2Name + " , Relais3: " + sRelay3Name + " read from file");

  // IO Inits
  LEDInit();
  initRelays();

  // Boardinfo	11
  /**
   * @brief 
   * Read Boardinfo for output 
   */
    sBoardInfo = boardInfo.ShowChipIDtoString();

	//Wifi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPdisconnect();
  if(WiFi.softAP(AP_SSID, AP_PASSWORD, channel, hide_SSID, max_connection)){
    WiFi.softAPConfig(IP, Gateway, NMask);
    Serial.println("\nAccesspoint " + String(AP_SSID) + " running");
    Serial.println("\nSet IP " + IP.toString() + ", Gateway: " + Gateway.toString() + ", NetMask: " + NMask.toString() + " ready");
    LEDon(LED(Green));
    delay(1000);
    LEDoff(LED(Green));
  } else {
      Serial.println(F("Starting AP failed."));
      LEDon(LED(Red));  
      delay(1000); 
      ESP.restart();
  }

  WiFi.setHostname(HostName);
  Serial.println("Set Hostname " + String(WiFi.getHostname()) + " done\n");

  delay(1000);
  WiFiDiag();

	if (!MDNS.begin(AP_SSID)) {
		Serial.println(F("Error setting up MDNS responder!"));
		while (1) {
			delay(1000);
		}
	}
  Serial.println("mDNS responder started\n");

// Start TCP (HTTP) server
	server.begin();
	Serial.println("TCP server started\n");

// Add service to MDNS-SD
	MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);

// Webconfig laden
  initWebsite();
  
// Init NMEA2000
  initNMEA2000();
  CZone_Init();

  // Register incoming NMEA2000 message handler so we can react to remote commands
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

/**
 * @brief OTA
 * 
 */
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
 
  printf("Setup end\n");
}

bool IsTimeToUpdate(unsigned long NextUpdate) {
  return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0) {
  return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
  while ( NextUpdate < millis() ) NextUpdate += Period;
}

/************************ n2k Datenfunktionen ***************************/

/**
 * @brief Send PGN127501
 * 
 * @param Switchbank 
 */
void SendN2kSwitchBankStatus(bool Status1, bool Status2, bool Status3) {
  static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, SwitchStatusSendOffset);
  tN2kMsg N2kMsg;
  tN2kBinaryStatus bankStatus;

  if ( IsTimeToUpdate(SlowDataUpdated) ) {
    SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);
    N2kResetBinaryStatus(bankStatus);

    // Read actual logical relay states at send time to avoid stale values
    Status1 = GetRelayLogicalState(0);
    Status2 = GetRelayLogicalState(1);
    Status3 = GetRelayLogicalState(2);
    Serial.printf("R1 Status     : %s \n", Status1 ? "On" : "Off");
    Serial.printf("R2 Status     : %s \n", Status2 ? "On" : "Off");
    Serial.printf("R3 Status     : %s \n", Status3 ? "On" : "Off");

    const bool report1 = InvertN2kStatus ? !Status1 : Status1;
    const bool report2 = InvertN2kStatus ? !Status2 : Status2;
    const bool report3 = InvertN2kStatus ? !Status3 : Status3;

    // Build full bank status and keep one deliberate empty slot for Vulcan compatibility.
    // If relay 3 is mapped to slot 4, keep slot 3 empty instead.
    const uint8_t placeholderSlot = (N2K_THIRD_SWITCH_ITEM == 4) ? 3 : 4;
    N2kSetStatusBinaryOnStatus(bankStatus, report1 ? N2kOnOff_On : N2kOnOff_Off, 1);
    N2kSetStatusBinaryOnStatus(bankStatus, report2 ? N2kOnOff_On : N2kOnOff_Off, 2);
    N2kSetStatusBinaryOnStatus(bankStatus, report3 ? N2kOnOff_On : N2kOnOff_Off, N2K_THIRD_SWITCH_ITEM);
    N2kSetStatusBinaryOnStatus(bankStatus, N2kOnOff_Unavailable, placeholderSlot);
    SetN2kPGN127501(N2kMsg, 0, bankStatus);

        // Diagnostic: print message bytes only when N2kDebug is enabled.
    if (N2kDebug) {
      N2kMsg.Print(&Serial);
    }

    // Send only when we updated the message
    NMEA2000.SendMsg(N2kMsg);
  }
}


void SetN2kPGN126208(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus) {
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority=3;
	BankStatus = (BankStatus << 8) | DeviceBankInstance;
	N2kMsg.AddUInt64(BankStatus);
}  

//*****************************************************************************
inline void SetN2kSwitchBankCommand(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus) {
  SetN2kPGN127502(N2kMsg,DeviceBankInstance,BankStatus);
}

//*****************************************************************************
void SetSwitch(unsigned char DeviceBankInstance, uint8_t SwitchIndex, bool ItemStatus) {
  tN2kBinaryStatus BankStatus;
  tN2kMsg N2kMsg;

  N2kResetBinaryStatus(BankStatus);
  N2kSetStatusBinaryOnStatus(BankStatus,ItemStatus?N2kOnOff_On:N2kOnOff_Off,SwitchIndex);
  SetN2kSwitchBankCommand(N2kMsg,DeviceBankInstance,BankStatus);
  NMEA2000.SendMsg(N2kMsg);
}

#define PRIORITY_DO_NOT_CHANGE    0x08
#define RESERVED_4_BITS_SET_TO_1  0x0f

void SetN2kGroupFunctionCommand126208(tN2kMsg &N2kMsg, unsigned char DestinationId,
                     unsigned char FieldNoOfParam, unsigned char FieldValue) {
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority    = 3;
    N2kMsg.Destination = DestinationId;
    N2kMsg.AddByte(N2kgfc_Command);     // field 1
    N2kMsg.Add3ByteInt(126208L);        // field 2 : Commanded PGN
                                        // field 3 : Priority Setting
                                        // field 4 : NMEA Reserved
    N2kMsg.AddByte(PRIORITY_DO_NOT_CHANGE | RESERVED_4_BITS_SET_TO_1<<4);
    N2kMsg.AddByte(1);                  // field 5 : Number of Pairs .. to follow
    N2kMsg.AddByte(FieldNoOfParam);     // field 6 : Field No of commanded param
    N2kMsg.AddByte(FieldValue);         // field 7 : Value of commanded param
}

void sendSwitchCommand(unsigned char DestinationId, unsigned char FieldNoOfParam, unsigned char FieldValue)
{
    tN2kMsg N2kMsg;
    SetN2kGroupFunctionCommand126208(N2kMsg, DestinationId, FieldNoOfParam, FieldValue);
    NMEA2000.SendMsg(N2kMsg);
}
  
void CheckSourceAddressChange() {
  int SourceAddress = NMEA2000.GetN2kSource();

  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory 
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}
/************************************ Loop ***********************************/
void loop() {
  SendN2kSwitchBankStatus(Rel1Status, Rel2Status, Rel3Status);

  NMEA2000.ParseMessages();
  CZone_Loop();
  CheckSourceAddressChange();

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) {
    Serial.read();
  }


// OTA	
	ArduinoOTA.handle();

/**
 * @brief Actual Website Data
 * 
 */
  
    sCL_Status = sWifiStatus(WiFi.status());
    sAP_Station = WiFi.softAPgetStationNum();
    freeHeapSpace();

  Rel1Status = GetRelayLogicalState(0);
  Rel2Status = GetRelayLogicalState(1);
  Rel3Status = GetRelayLogicalState(2);
    
/**
 * @brief Construct a new if object
 * Reboot from Website
*/
  if (IsRebootRequired) {
      Serial.println("Rebooting ESP32: "); 
      delay(1000); // give time for reboot page to load
      ESP.restart();
      }

	// LED Indicator
    LoopIndicator();

}