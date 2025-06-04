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

#define ENABLE_DEBUG_LOG 0 // Debug log


/**
 *  Set the information for other bus devices, which PGN messages we support
 */ 
const unsigned long TransmitMessages[] PROGMEM = {127501L, // Binary Status Report
                                                  127502L, // Binary Switch Status
                                                  126208L, // B&G may use 126208 for commanding switches
                                                  0
                                                 };



void debug_log(char* str) {
#if ENABLE_DEBUG_LOG == 1
  Serial.println(str);
#endif
}


/******************************************* Setup *******************************************************/
void setup() {

  // Init USB serial port
  Serial.begin(115200);

  Serial.printf("Binary Switch setup %s start\n", Version);

  /**
   * @brief Filesystem prepare for Webfiles
   * 
   */
	if (!LittleFS.begin(true)) {
		Serial.println("An Error has occurred while mounting LittleFS");
		return;
	}
	Serial.println("\nBytes LittleFS used:" + String(LittleFS.usedBytes()));

	File root = LittleFS.open("/");
  listDir(LittleFS, "/", 3);

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

  // LED
  LEDInit();

  // IOinit
  pinMode(Relais[0], OUTPUT);
  pinMode(Relais[1], OUTPUT);
  pinMode(Relais[2], OUTPUT);
  digitalWrite(Relais[0], 0);
  digitalWrite(Relais[1], 0);
  digitalWrite(Relais[2], 0);

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
      Serial.println("Starting AP failed.");
      LEDon(LED(Red));  
      delay(1000); 
      ESP.restart();
  }

  WiFi.setHostname(HostName);
  Serial.println("Set Hostname " + String(WiFi.getHostname()) + " done\n");

  delay(1000);
  WiFiDiag();

	if (!MDNS.begin(AP_SSID)) {
		Serial.println("Error setting up MDNS responder!");
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
  website();
  

// Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

/**
 * @brief Set NMEA2000 product information
 * 
 */
  NMEA2000.SetProductInformation("BA01.2504", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Binary Actor Module",  // Manufacturer's Model ID
                                 "0.9.0.0 (2025-04-20)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2024-04-20)" // Manufacturer's Model version
                                );
// Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number.
                                130, // Device function=Binary Event Monitor. See codes on See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                30, // Device class=Inter/Intranetwork Device. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

// If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 33);  // Read stored last NodeAddress, default 33
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  Serial.println("Opening NMEA2000");
  if (NMEA2000.Open())
    Serial.println(" NMEA2000 Initialized"\n);
  else
    Serial.println(" NMEA2000 Initialized failed"\n);

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