#include "configuration.h"

// Node address and persistent preferences
int NodeAddress = 33;
Preferences preferences;
uint8_t chipid[6] = {0};
uint32_t id = 0;
int i = 0;

String sHeapspace = "";
Web_Config tAP_Config;

IPAddress IP = IPAddress(192, 168, 15, 25);
IPAddress Gateway = IPAddress(192, 168, 15, 25);
IPAddress NMask = IPAddress(255, 255, 255, 0);
const char* AP_SSID = "BinarySwitch";
const char* AP_PASSWORD  = "12345678";
IPAddress AP_IP;
IPAddress CL_IP;
IPAddress SELF_IP;
String sAP_Station = "";

int iSTA_on = 0;
int bConnect_CL = 0;
bool bClientConnected = false;

// AP runtime settings (legacy names)
int channel = CHANNEL_DEFAULT;
bool hide_SSID = HIDE_SSID_DEFAULT;
int max_connection = MAX_CONNECTIONS_DEFAULT;

double ADC_Calibration_Value1 = 170.0;
double ADC_Calibration_Value2 = 19.0;

float fbmp_temperature = 0;
float fbmp_pressure = 0;
float fbmp_altitude = 0;
String sI2C_Status = "";
bool bI2C_Status = false;

int iDistance = 0;

float FuelLevel = 0;
float FuelLevelMax = 30;
float CoolantTemp = 0;
float MotorTemp = 0;
float EngineRPM = 0;
float BordSpannung = 0;
bool EngineOn = false;
String motorErrorReported = "Aus";
String coolantErrorReported = "Aus";
unsigned long Counter = 0;

int Bat1Capacity = 55;
int Bat2Capacity = 90;
int SoCError = 0;
float BatSoC = 0;

String sOneWire_Status = "";

float fDrehzahl = 0;
float fGaugeDrehzahl = 0;
float fBordSpannung = 0;
float fCoolantTemp = 0;
float fMotorTemp = 0;
float fCoolantOffset = 0;
float fMotorOffset = 0;
String sSTBB = "";
String sOrient = "";

double dMWV_WindDirectionT = 0;
double dMWV_WindSpeedM = 0;
double dVWR_WindDirectionM = 0;
double dVWR_WindAngle = 0;
double dVWR_WindSpeedkn = 0;
double dVWR_WindSpeedms = 0;

const char *udpAddress = "192.168.30.255";
const int udpPort = 4444;

int Relais[NUM_RELAYS] = {25, 26, 27};
int SwitchSet = 0;
bool Rel1Status = false;
bool Rel2Status = false;
bool Rel3Status = false;
// By default, allow MFD commands to control all relays
bool RelayAllowMFD[NUM_RELAYS] = { true, true, true };
String sRelay1Name = "";
String sRelay2Name = "";
String sRelay3Name = "";
// Default: ensure relays powered-off at start to avoid accidental activation
bool ForceRelaysOffAtBoot = true;
// Default: report relay states without inversion.
bool InvertN2kStatus = false;

bool N2kDebug = false;
bool AcceptBroadcastCommands = true;
unsigned long LastReceivedN2kPGN = 0;
String LastReceivedN2kText = "";
unsigned long BootBlockUntil = 0;


