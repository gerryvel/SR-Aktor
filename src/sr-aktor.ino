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
 * @brief Binary switch NMEA2000
 * @version 1.0
 * @date 2025-05-18
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
#include "helper.h"
#include "web.h"
#include "LEDindicator.h"
#include "init.h"

#define ENABLE_DEBUG_LOG 0 // Debug log


void debug_log(char* str) {
#if ENABLE_DEBUG_LOG == 1
  Serial.println(str);
#endif
}


/******************************************* Setup *******************************************************/
void setup() {

  initSerial();
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

  // Boardinfo	
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
    Serial.println("\nSet IP " + IP.toString() + " ,Gateway: " + Gateway.toString() + " ,NetMask: " + NMask.toString() + " ready");
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
  
// Init and start NMEA2000
  initNMEA2000();

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
 * @brief Send PGN127502
 * 
 * @param Switchbank 
 */

void SetSwitch(unsigned char DeviceBankInstance, uint8_t SwitchIndex, bool ItemStatus) {
  tN2kBinaryStatus BankStatus;
  tN2kMsg N2kMsg;

  N2kResetBinaryStatus(BankStatus);
  N2kSetStatusBinaryOnStatus(BankStatus,ItemStatus?N2kOnOff_On:N2kOnOff_Off,SwitchIndex);
  SetN2kSwitchBankCommand(N2kMsg,DeviceBankInstance,BankStatus);
  NMEA2000.SendMsg(N2kMsg);
}


inline void SetN2kSwitchBankCommand(tN2kMsg &N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus) {
  SetN2kPGN127502(N2kMsg,DeviceBankInstance,BankStatus);
}

void SendSwitchControl(unsigned char DeviceBankInstance){
  tN2kBinaryStatus BankStatus;
  static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, SwitchControlSendOffset);
  tN2kMsg N2kMsg;

  SetN2kPGN127502(N2kMsg,DeviceBankInstance,BankStatus);  //Note that B&G may use 126208 for commanding switches.
  NMEA2000.SendMsg(N2kMsg);
}

void SendN2kSwitchBankStatus(bool Status1, bool Status2, bool Status3) {
  static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, SwitchStatusSendOffset);
  tN2kMsg N2kMsg;

  if ( IsTimeToUpdate(SlowDataUpdated) ) {
    SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

    Serial.printf("R1 Status     : %s \n", Status1 ? "On" : "Off");
    Serial.printf("R2 Status     : %s \n", Status2 ? "On" : "Off");
    Serial.printf("R3 Status     : %s \n", Status3 ? "On" : "Off");

{
  tN2kBinaryStatus BankStatus;
  N2kResetBinaryStatus(BankStatus);
  N2kSetStatusBinaryOnStatus(BankStatus, Status1 ? N2kOnOff_On : N2kOnOff_Off, 0);
  N2kSetStatusBinaryOnStatus(BankStatus, Status2 ? N2kOnOff_On : N2kOnOff_Off, 1);
  N2kSetStatusBinaryOnStatus(BankStatus, Status3 ? N2kOnOff_On : N2kOnOff_Off, 2);
  SetN2kPGN127501(N2kMsg, 0, BankStatus);
}
  }
     NMEA2000.SendMsg(N2kMsg);
  }


/************************************ Loop ***********************************/
void loop() {

  SendSwitchControl(0);
  SendN2kSwitchBankStatus(Rel1Status, Rel2Status, Rel3Status);

  NMEA2000.ParseMessages();
  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }

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

    Rel1Status = digitalRead(Relais[0]);
    Rel2Status = digitalRead(Relais[1]);
    Rel3Status = digitalRead(Relais[2]);
    
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