#ifndef _INIT_H_
#define _INIT_H_

/**
 * @file init.h
 * @author Gerry Sebb
 * @brief Initalisierungfunktionen und Setup
 * @version 1.0
 * @date 2025-06-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include "configuration.h"

// Init USB serial port
void initSerial() {
  Serial.begin(115200);
  Serial.printf("Binary Switch setup %s start\n", Version);
}

  /**
   * @brief Filesystem prepare for Webfiles
   * 
   */
void initLittleFS() {  
	if (!LittleFS.begin(true)) {
		Serial.println(F("An Error has occurred while mounting LittleFS"));
		return;
	}
	Serial.println("\nBytes LittleFS used:" + String(LittleFS.usedBytes()));

	File root = LittleFS.open("/");
  listDir(LittleFS, "/", 3);
}


void initRelays() {
  for (int i = 0; i < 3; i++) {
    pinMode(Relais[i], OUTPUT);
    digitalWrite(Relais[i], 0);
  }
}

String relayState(int numRelay){
  if(RELAY_NO){
    if(digitalRead(Relais[numRelay-1])){
      return "";
    }
    else {
      return "checked";
    }
  }
  else {
    if(digitalRead(Relais[numRelay-1])){
      return "checked";
    }
    else {
      return "";
    }
  }
  return "";
}

void initNMEA2000() {
// Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

/**
 * @brief Set NMEA2000 product information
 * Set the information for other bus devices, which PGN messages we support
 */
  const unsigned long TransmitMessages[] PROGMEM = {127501L, // Binary Status Report
                                                    127502L, // Binary Switch Status
                                                    126208L, // B&G may use 126208 for commanding switches
                                                   0
                                                   };


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

  Serial.println(F("Opening NMEA2000"));
  if (NMEA2000.Open())
    Serial.println(F(" NMEA2000 Initialized\n"));
  else
    Serial.println(F(" NMEA2000 Initialized failed\n"));

  }

#endif