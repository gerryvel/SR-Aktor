# 1 "C:\\Users\\GERRYA~1\\AppData\\Local\\Temp\\tmp1ekh4hww"
#include <Arduino.h>
# 1 "C:/Users/gerryadmin/Documents/SR Aktor/src/sr-aktor.ino"
# 26 "C:/Users/gerryadmin/Documents/SR Aktor/src/sr-aktor.ino"
#include <Arduino.h>
#include "configuration.h"
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <ESP_WiFi.h>
#include <ESPAsyncWebServer.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <ESPmDNS.h>
#include <arpa/inet.h>
#include "BoardInfo.h"
#include "czone.h"
#include "helper.h"
#include "web.h"
#include "LEDindicator.h"
#include "init.h"





N2kRawEntry N2kRawBuf[N2KRAW_MAX];
uint8_t N2kRawHead = 0;

ProprietaryMapping PropMappings[MAX_PROP_MAPPINGS];
uint8_t PropMappingCount = 0;

DetectedEvent DetectedEvents[MAX_DETECTED_EVENTS];
uint8_t DetectedEventCount = 0;
void PushRawN2k(const tN2kMsg &msg);
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void setup();
bool IsTimeToUpdate(unsigned long NextUpdate);
void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period);
void SendN2kSwitchBankStatus(bool Status1, bool Status2, bool Status3);
void SetN2kPGN126208(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus);
inline void SetN2kSwitchBankCommand(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus);
void SetSwitch(unsigned char DeviceBankInstance, uint8_t SwitchIndex, bool ItemStatus);
void SetN2kGroupFunctionCommand126208(tN2kMsg &N2kMsg, unsigned char DestinationId,
                     unsigned char FieldNoOfParam, unsigned char FieldValue);
void sendSwitchCommand(unsigned char DestinationId, unsigned char FieldNoOfParam, unsigned char FieldValue);
void CheckSourceAddressChange();
void loop();
#line 57 "C:/Users/gerryadmin/Documents/SR Aktor/src/sr-aktor.ino"
void PushRawN2k(const tN2kMsg &msg) {
  N2kRawHead = (N2kRawHead + 1) % N2KRAW_MAX;
  N2kRawEntry &e = N2kRawBuf[N2kRawHead];
  e.pgn = msg.PGN;
  e.src = msg.Source;
  e.dst = msg.Destination;
  e.len = msg.DataLen;
  for (uint8_t i = 0; i < 8; i++) e.data[i] = (i < e.len) ? msg.Data[i] : 0xFF;
}


void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  if (N2kDebug) {
    Serial.print("Received PGN: "); Serial.println(N2kMsg.PGN);
    N2kMsg.Print(&Serial);
  }

  PushRawN2k(N2kMsg);
  LastReceivedN2kPGN = N2kMsg.PGN;
  LastReceivedN2kText = String("PGN:") + String(N2kMsg.PGN) + String(" Src:") + String(N2kMsg.Source) + String(" Dst:") + String(N2kMsg.Destination);

  if (CZone_HandleMsg(N2kMsg)) {
    return;
  }


  if (N2kMsg.PGN == 127502L) {
    unsigned char TargetBankInstance = 0;
    tN2kBinaryStatus BankStatus = 0;
    if (ParseN2kPGN127502(N2kMsg, TargetBankInstance, BankStatus)) {



  bool isBroadcast = (N2kMsg.Destination == 0 || N2kMsg.Destination == 255);
  if (TargetBankInstance == 0 || TargetBankInstance == 1 || (isBroadcast && AcceptBroadcastCommands)) {
        tN2kOnOff s1 = N2kGetStatusOnBinaryStatus(BankStatus, 1);
        tN2kOnOff s2 = N2kGetStatusOnBinaryStatus(BankStatus, 2);
      tN2kOnOff s3 = N2kGetStatusOnBinaryStatus(BankStatus, N2K_THIRD_SWITCH_ITEM);

        bool on1 = (s1 == N2kOnOff_On);
        bool on2 = (s2 == N2kOnOff_On);
      bool on3 = (s3 == N2kOnOff_On);

        digitalWrite(Relais[0], on1 ? HIGH : LOW);
        digitalWrite(Relais[1], on2 ? HIGH : LOW);
        digitalWrite(Relais[2], on3 ? HIGH : LOW);

        Rel1Status = GetRelayLogicalState(0);
        Rel2Status = GetRelayLogicalState(1);
        Rel3Status = GetRelayLogicalState(2);
        SendN2kSwitchBankStatus(Rel1Status, Rel2Status, Rel3Status);
      }
    }
    return;
  }




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

            bool isBroadcast = (N2kMsg.Destination == 0 || N2kMsg.Destination == 255);
            if (N2kMsg.Destination != NodeAddress && !(isBroadcast && AcceptBroadcastCommands)) {

              return;
            }
            for (uint8_t p = 0; p < NumberOfParameterPairs; p++) {
              uint8_t FieldNo = N2kMsg.GetByte(Index);
              uint8_t FieldValue = N2kMsg.GetByte(Index);

              if (FieldNo >= 2 && FieldNo <= 4) {
                bool on = (FieldValue != 0);
                digitalWrite(Relais[FieldNo - 2], on ? HIGH : LOW);
              } else if (FieldNo >= 1 && FieldNo <= 3) {

                bool on = (FieldValue != 0);
                digitalWrite(Relais[FieldNo - 1], on ? HIGH : LOW);
              }
            }

            Rel1Status = digitalRead(Relais[0]);
            Rel2Status = digitalRead(Relais[1]);
            Rel3Status = digitalRead(Relais[2]);
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





  readConfig("/config.json");
    IP = inet_addr(tAP_Config.wAP_IP);
    AP_SSID = tAP_Config.wAP_SSID;
    AP_PASSWORD = tAP_Config.wAP_Password;
    sRelay1Name = tAP_Config.wRelay1Name;
    sRelay2Name = tAP_Config.wRelay2Name;
    sRelay3Name = tAP_Config.wRelay3Name;
    Serial.println("\nConfigdata : AP IP: " + IP.toString() + ", AP SSID: " + AP_SSID + " , Passwort: " + AP_PASSWORD + " , Relais1: " + sRelay1Name + " , Relais2: " + sRelay2Name + " , Relais3: " + sRelay3Name + " read from file");


  LEDInit();
  initRelays();






    sBoardInfo = boardInfo.ShowChipIDtoString();


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


 server.begin();
 Serial.println("TCP server started\n");


 MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);


  initWebsite();


  initNMEA2000();
  CZone_Init();


  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);





  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else
        type = "filesystem";


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
# 293 "C:/Users/gerryadmin/Documents/SR Aktor/src/sr-aktor.ino"
void SendN2kSwitchBankStatus(bool Status1, bool Status2, bool Status3) {
  static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, SwitchStatusSendOffset);
  tN2kMsg N2kMsg;
  tN2kBinaryStatus bankStatus;

  if ( IsTimeToUpdate(SlowDataUpdated) ) {
    SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);
    N2kResetBinaryStatus(bankStatus);


    Status1 = GetRelayLogicalState(0);
    Status2 = GetRelayLogicalState(1);
    Status3 = GetRelayLogicalState(2);
    Serial.printf("R1 Status     : %s \n", Status1 ? "On" : "Off");
    Serial.printf("R2 Status     : %s \n", Status2 ? "On" : "Off");
    Serial.printf("R3 Status     : %s \n", Status3 ? "On" : "Off");

    const bool report1 = InvertN2kStatus ? !Status1 : Status1;
    const bool report2 = InvertN2kStatus ? !Status2 : Status2;
    const bool report3 = InvertN2kStatus ? !Status3 : Status3;



    N2kSetStatusBinaryOnStatus(bankStatus, report1 ? N2kOnOff_On : N2kOnOff_Off, 1);
    N2kSetStatusBinaryOnStatus(bankStatus, report2 ? N2kOnOff_On : N2kOnOff_Off, 2);
    N2kSetStatusBinaryOnStatus(bankStatus, report3 ? N2kOnOff_On : N2kOnOff_Off, N2K_THIRD_SWITCH_ITEM);
    N2kSetStatusBinaryOnStatus(bankStatus, N2kOnOff_Unavailable, 4);
    SetN2kPGN127501(N2kMsg, 0, bankStatus);


    if (N2kDebug) {
      N2kMsg.Print(&Serial);
    }


    NMEA2000.SendMsg(N2kMsg);
  }
}


void SetN2kPGN126208(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus) {
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority=3;
 BankStatus = (BankStatus << 8) | DeviceBankInstance;
 N2kMsg.AddUInt64(BankStatus);
}


inline void SetN2kSwitchBankCommand(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus) {
  SetN2kPGN127502(N2kMsg,DeviceBankInstance,BankStatus);
}


void SetSwitch(unsigned char DeviceBankInstance, uint8_t SwitchIndex, bool ItemStatus) {
  tN2kBinaryStatus BankStatus;
  tN2kMsg N2kMsg;

  N2kResetBinaryStatus(BankStatus);
  N2kSetStatusBinaryOnStatus(BankStatus,ItemStatus?N2kOnOff_On:N2kOnOff_Off,SwitchIndex);
  SetN2kSwitchBankCommand(N2kMsg,DeviceBankInstance,BankStatus);
  NMEA2000.SendMsg(N2kMsg);
}

#define PRIORITY_DO_NOT_CHANGE 0x08
#define RESERVED_4_BITS_SET_TO_1 0x0f

void SetN2kGroupFunctionCommand126208(tN2kMsg &N2kMsg, unsigned char DestinationId,
                     unsigned char FieldNoOfParam, unsigned char FieldValue) {
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority = 3;
    N2kMsg.Destination = DestinationId;
    N2kMsg.AddByte(N2kgfc_Command);
    N2kMsg.Add3ByteInt(126208L);


    N2kMsg.AddByte(PRIORITY_DO_NOT_CHANGE | RESERVED_4_BITS_SET_TO_1<<4);
    N2kMsg.AddByte(1);
    N2kMsg.AddByte(FieldNoOfParam);
    N2kMsg.AddByte(FieldValue);
}

void sendSwitchCommand(unsigned char DestinationId, unsigned char FieldNoOfParam, unsigned char FieldValue)
{
    tN2kMsg N2kMsg;
    SetN2kGroupFunctionCommand126208(N2kMsg, DestinationId, FieldNoOfParam, FieldValue);
    NMEA2000.SendMsg(N2kMsg);
}

void CheckSourceAddressChange() {
  int SourceAddress = NMEA2000.GetN2kSource();

  if (SourceAddress != NodeAddress) {
    NodeAddress = SourceAddress;
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}

void loop() {
  static unsigned long lastSwitchControlAnnounce = 0;

  SendN2kSwitchBankStatus(Rel1Status, Rel2Status, Rel3Status);



  if (millis() - lastSwitchControlAnnounce >= 1000UL) {
    tN2kBinaryStatus controlBankStatus;
    tN2kMsg controlMsg;

    lastSwitchControlAnnounce = millis();

    Rel1Status = GetRelayLogicalState(0);
    Rel2Status = GetRelayLogicalState(1);
    Rel3Status = GetRelayLogicalState(2);

    N2kResetBinaryStatus(controlBankStatus);
    N2kSetStatusBinaryOnStatus(controlBankStatus, Rel1Status ? N2kOnOff_On : N2kOnOff_Off, 1);
    N2kSetStatusBinaryOnStatus(controlBankStatus, Rel2Status ? N2kOnOff_On : N2kOnOff_Off, 2);
    N2kSetStatusBinaryOnStatus(controlBankStatus, Rel3Status ? N2kOnOff_On : N2kOnOff_Off, N2K_THIRD_SWITCH_ITEM);

    N2kSetStatusBinaryOnStatus(controlBankStatus, N2kOnOff_Unavailable, 4);

    SetN2kSwitchBankCommand(controlMsg, 0, controlBankStatus);
    NMEA2000.SendMsg(controlMsg);
  }

  NMEA2000.ParseMessages();
  CZone_Loop();
  CheckSourceAddressChange();


  if ( Serial.available() ) {
    Serial.read();
  }



 ArduinoOTA.handle();






    sCL_Status = sWifiStatus(WiFi.status());
    sAP_Station = WiFi.softAPgetStationNum();
    freeHeapSpace();

  Rel1Status = GetRelayLogicalState(0);
  Rel2Status = GetRelayLogicalState(1);
  Rel3Status = GetRelayLogicalState(2);





  if (IsRebootRequired) {
      Serial.println("Rebooting ESP32: ");
      delay(1000);
      ESP.restart();
      }


    LoopIndicator();

}