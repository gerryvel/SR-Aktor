#ifndef __configuration__H__
#define __configuration__H__

/**
 * @file configuration.h
 * @author Gerry Sebb
 * @brief Konfiguration für GPIO und Variable
 * @version 2.3
 * @date 2025-01-06
 */

// Core Arduino includes (provides String, IPAddress, uint8_t, etc.)
#include <Arduino.h>
#include <Preferences.h>

// Versionierung
#define VersionSoftware "1.3.1.0 (2026-06-26)"  // Version Software
#define VersionHardware "1.1.0.0 (2025-08-07)"  // Version Hardware

/**
 * @brief Config NMEA2000
 */
#define ESP32_CAN_TX_PIN GPIO_NUM_4  // Set CAN TX port to 4 
#define ESP32_CAN_RX_PIN GPIO_NUM_5  // Set CAN RX port to 5
#define N2K_SOURCE 15

// --- Test-Schalter: CZone Protokoll komplett aktivieren/deaktivieren ---
// 1 = CZone Pakete werden gesendet/verarbeitet (Originalverhalten)
// 0 = Nur Standard-NMEA2000 PGNs (127501/127502/126208), kein CZone-Traffic.
//     Zum Testen, ob das MFD auch ohne CZone eine Switching-Seite anlegt.
#ifndef ENABLE_CZONE
#define ENABLE_CZONE 1
#endif

// General timing / offsets
#define SwitchControlSendOffset 0
#define SwitchStatusSendOffset 40
#define RPMSendOffset 80
#define BatteryDCSendOffset 120
#define BatteryDCStatusSendOffset 160
#define SlowDataUpdatePeriod 1000  // Time between CAN Messages sent
// NMEA2000 item index for the third relay on PGN127501/127502.
// Standard item index for the third relay on PGN127501/127502.
#define N2K_THIRD_SWITCH_ITEM 3

// Configuration Website
#define PAGE_REFRESH 10 // x Sec.
#define WEB_TITEL "SR03 Module"
extern String sHeapspace;

// Configuration mit Webinterface
struct Web_Config
{
	char wAP_IP[20];
	char wAP_SSID[64];
	char wAP_Password[12];
	char wRelay1Name[12];
	char wRelay2Name[12];
	char wRelay3Name[12];
	char wADC2_Cal[6];
};
extern Web_Config tAP_Config;

// Configuration AP
#define HostName "BinarySwitch"
#define CHANNEL_DEFAULT 10
#define HIDE_SSID_DEFAULT false
#define MAX_CONNECTIONS_DEFAULT 2

// Variables for WIFI-AP
extern IPAddress IP;
extern IPAddress Gateway;
extern IPAddress NMask;
extern const char* AP_SSID;
extern const char* AP_PASSWORD;
extern IPAddress AP_IP;
extern IPAddress CL_IP;
extern IPAddress SELF_IP;
extern String sAP_Station;

// Node address and persistent preferences
extern int NodeAddress;
extern Preferences preferences;

// AP runtime settings (provide variables for legacy code using 'channel', 'hide_SSID', 'max_connection')
extern int channel;
extern bool hide_SSID;
extern int max_connection;

// --- Definitions for internal helper types used by N2k debugging and mappings ---
// Keep these lightweight so they can be used from multiple compilation units.
#ifndef N2KRAW_MAX
#define N2KRAW_MAX 16
#endif

#ifndef MAX_PROP_MAPPINGS
#define MAX_PROP_MAPPINGS 32
#endif

#ifndef MAX_DETECTED_EVENTS
#define MAX_DETECTED_EVENTS 32
#endif

// Raw N2k message entry used for web debug buffer
struct N2kRawEntry {
	uint32_t pgn;
	uint8_t src;
	uint8_t dst;
	uint8_t len;
	uint8_t data[8];
};

// Simple mapping definition: pgn, byte offset, expected value, relay index (1..), action
struct ProprietaryMapping {
	uint32_t pgn;
	uint8_t offset;
	uint8_t value;
	uint8_t relay;
	uint8_t action;
};

// Detected event cached for edge-detection
struct DetectedEvent {
	uint32_t pgn;
	unsigned long lastSeen;
	uint8_t count;
	uint8_t len;
	uint8_t data[32];
};

// Configuration Client (Network Data Windsensor)
#define CL_SSID "NoWa"
#define CL_PASSWORD "12345678"
extern int iSTA_on;
extern int bConnect_CL;
extern bool bClientConnected;

// Calibration data variable definition for ADC1 and ADC2 Input
extern double ADC_Calibration_Value1;
extern double ADC_Calibration_Value2;

// Configuration Sensors I2C
#define I2C_SDA 21
#define I2C_SCL 22
#define SEALEVELPRESSURE_HPA (1013.25)
extern float fbmp_temperature;
extern float fbmp_pressure;
extern float fbmp_altitude;
extern String sI2C_Status;
extern bool bI2C_Status;

// Global Data Sonar
const int iMaxSonar = 35;
extern int iDistance;

// Global Data Motor data
extern float FuelLevel;
extern float FuelLevelMax;
extern float CoolantTemp;
extern float MotorTemp;
extern float EngineRPM;
extern float BordSpannung;
extern bool EngineOn;
extern String motorErrorReported;
extern String coolantErrorReported;
extern unsigned long Counter;
enum EngineStatus { Off = 0, On = 1 };
#define RPM_Calibration_Value 4.0
#define Eingine_RPM_Pin 19

// Global Data Battery
extern int Bat1Capacity;
extern int Bat2Capacity;
extern int SoCError;
extern float BatSoC;

// Data wire for temperature (Dallas DS18B20)
#define ONE_WIRE_BUS 14
extern String sOneWire_Status;

// Variables Website
extern float fDrehzahl;
extern float fGaugeDrehzahl;
extern float fBordSpannung;
extern float fCoolantTemp;
extern float fMotorTemp;
extern float fCoolantOffset;
extern float fMotorOffset;
extern String sSTBB;
extern String sOrient;

// Definition NMEA0183 MWV
extern double dMWV_WindDirectionT;
extern double dMWV_WindSpeedM;
extern double dVWR_WindDirectionM;
extern double dVWR_WindAngle;
extern double dVWR_WindSpeedkn;
extern double dVWR_WindSpeedms;

// Variable NMEA 0183 Stream
extern const char *udpAddress;
extern const int udpPort;

// SR-Board
#define NUM_RELAYS 3
#define RELAY_NO 1
extern int Relais[NUM_RELAYS];
extern int SwitchSet;
extern bool Rel1Status;
extern bool Rel2Status;
extern bool Rel3Status;
// Per-relay flags: allow MFD control for each relay (default true)
extern bool RelayAllowMFD[NUM_RELAYS];
extern String sRelay1Name;
extern String sRelay2Name;
extern String sRelay3Name;

// NMEA2000 / debug globals
extern bool N2kDebug;
extern bool InvertN2kStatus;
extern bool AcceptBroadcastCommands;
extern unsigned long LastReceivedN2kPGN;
extern String LastReceivedN2kText;

// Boot protection timer (ms) - set during setup to block mappings until baseline seeded
extern unsigned long BootBlockUntil;

// N2k raw buffer and mappings - ensure types N2kRawEntry, ProprietaryMapping, DetectedEvent
// are defined before including this header
extern N2kRawEntry N2kRawBuf[];
extern uint8_t N2kRawHead;

extern ProprietaryMapping PropMappings[];
extern uint8_t PropMappingCount;

extern DetectedEvent DetectedEvents[];
extern uint8_t DetectedEventCount;


#endif // __configuration__H__
