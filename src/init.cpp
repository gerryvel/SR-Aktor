#include <Arduino.h>
#include "init.h"
#include "configuration.h"
#include <LittleFS.h>
#include "helper.h" // provides listDir and filesystem helpers
// Use NMEA2000 core header and extern instance to avoid multiple-definition
#include "NMEA2000.h"
extern tNMEA2000 &NMEA2000;

void initSerial() {
  Serial.begin(115200);
  Serial.printf("BinarySwitch setup %s start\n", VersionSoftware);
}

void initLittleFS() {
  if (!LittleFS.begin(false)) {
    Serial.println(F("An Error has occurred while mounting LittleFS"));
    return;
  }
  Serial.println("\nBytes LittleFS used:" + String(LittleFS.usedBytes()));
  listDir(LittleFS, "/", 3);
}


void initRelays() {
  for (int i = 0; i < 3; i++) {
    pinMode(Relais[i], OUTPUT);
    digitalWrite(Relais[i], LOW); // Relais off beim Start
    if (N2kDebug) Serial.printf("initRelays: pin %d set LOW\n", Relais[i]);
  }
    Rel1Status = GetRelayLogicalState(0);
    Rel2Status = GetRelayLogicalState(1);
    Rel3Status = GetRelayLogicalState(2);
}

String relayState(int numRelay){
  bool logicalOn = false;
  if (numRelay >= 1 && numRelay <= NUM_RELAYS) {
    int pinVal = digitalRead(Relais[numRelay-1]);
    // Hardware uses LOW = relay OFF, HIGH = relay ON
    logicalOn = (pinVal == HIGH);
  }
  return logicalOn ? String("checked") : String("");
}

void SetRelayLogical(int relayIndex, bool on) {
  if (relayIndex < 0 || relayIndex >= NUM_RELAYS) return;
  // Hardware uses LOW = relay OFF, HIGH = relay ON
  // Ensure we write HIGH for ON, LOW for OFF regardless of RELAY_NO setting
  digitalWrite(Relais[relayIndex], on ? HIGH : LOW);
  if (N2kDebug) Serial.printf("SetRelayLogical: relay=%d pin=%d -> %s\n", relayIndex+1, Relais[relayIndex], on?"ON":"OFF");
}

bool GetRelayLogicalState(int relayIndex) {
  if (relayIndex < 0 || relayIndex >= NUM_RELAYS) return false;
  int v = digitalRead(Relais[relayIndex]);
  // Hardware uses LOW = relay OFF, HIGH = relay ON
  return (v == HIGH);
}

void initNMEA2000() {
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  // Build a device unique id from efuse MAC
  uint64_t mac = ESP.getEfuseMac(); // 48-bit MAC
  // compress into a 32-bit id similar to previous behaviour
  uint32_t id_local = (uint32_t)(mac & 0xFFFFFFFF);

  const unsigned long TransmitMessages[] PROGMEM = {127501L, 127502L, 126208L, 130817L, 65283L, 65284L, 65290L, 0};
  const unsigned long ReciveMessages[] PROGMEM = {127501L, 127502L, 126208L, 65280L, 65284L, 65288L, 65290L, 0};

  NMEA2000.SetProductInformation("SR03.2510", 100, "SR Switch Module", VersionSoftware, VersionHardware);
  NMEA2000.SetDeviceInformation(id_local, 140, 30, 717);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);

  preferences.begin("nvs", false);
  NodeAddress = preferences.getInt("LastNodeAddress", 33);
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReciveMessages);

  Serial.println(F("Opening NMEA2000"));
  if (NMEA2000.Open())
    Serial.println(F(" NMEA2000 Initialized\n"));
  else
    Serial.println(F(" NMEA2000 Initialized failed\n"));
}
