/**
 * @file web.h
 * @author Gerry Sebb
 * @brief Webseite
 * Variablen lesen und schreiben, Webseiten erstellen
 * @version 0.1
 * @date 2025-01-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "helper.h"
#include "init.h"
#include "configuration.h"
#include "boardinfo.h"
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <Arduino.h>

// Set web server port number to 80
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket server on port 81

// Forward declaration for helper in sr-aktor.ino to send test commands
void sendSwitchCommand(unsigned char DestinationId, unsigned char FieldNoOfParam, unsigned char FieldValue);

// Info Board for HTML-Output
String sBoardInfo;
BoardInfo boardInfo;
bool IsRebootRequired = false;

String processor(const String& var)
{
    if (var == "CONFIGPLACEHOLDER")
    {
        String buttons = "";
        buttons += "<form onsubmit = \"return formToJson(this);\">";
        buttons += "<p class=\"CInput\"><label>SSID </label><input type = \"text\" name = \"SSID\" value=\"";
        buttons += tAP_Config.wAP_SSID;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>IP </label><input type = \"text\" name = \"IP\" value=\"";
        buttons += tAP_Config.wAP_IP;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>Password </label><input type = \"text\" name = \"Password\" value=\"";
        buttons += tAP_Config.wAP_Password;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>Relais 1 Name </label><input type = \"text\" name = \"Relay1Name\" value=\"";
        buttons += tAP_Config.wRelay1Name;
        buttons += "\"/> </p>";
        buttons += "<p class=\"CInput\"><label>Relais 2 Name </label><input type = \"text\" name = \"Relay2Name\" value=\"";
        buttons += tAP_Config.wRelay2Name;
        buttons += "\"/> </p>";
        buttons += "<p class=\"CInput\"><label>Relais 3 Name </label><input type = \"text\" name = \"Relay3Name\" value=\"";
        buttons += tAP_Config.wRelay3Name;
        buttons += "\"/> </p>";
        // NMEA2000 Debug checkbox
        buttons += "<p class=\"CInput\"><label>NMEA2000 Debug </label>";
        buttons += "<input type=\"checkbox\" name=\"n2kDebug\" ";
         if (N2kDebug) buttons += "checked";
        buttons += "/> </p>";
        // Accept broadcast commands checkbox
        buttons += "<p class=\"CInput\"><label>Accept Broadcast Commands </label>";
        buttons += "<input type=\"checkbox\" name=\"acceptBroadcastCommands\" ";
         if (AcceptBroadcastCommands) buttons += "checked";
        buttons += "/> </p>";
        buttons += "<p class=\"button\"><input type=\"submit\" value=\"Speichern\"></p>";
        buttons += "</form>";
        return buttons;
    }
    return String();
}

// Placeholder with button section 
String SwitchRelais(const String& var){
  Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons ="";
    for(int i=1; i<=NUM_RELAYS; i++){
      String relayStateValue = relayState(i);
      buttons+= "<h4>Relais " + String(i) + "</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"" + String(i) + "\" "+ relayStateValue +"><span class=\"slider\"></span></label>";
    }
    return buttons;
  }
  return String();
}


//Variables for website
String sCL_Status = sWifiStatus(WiFi.status());

String replaceVariable(const String& var)
{
    if (var == "sRelay1Name") return sRelay1Name;
  	if (var == "sRelay2Name") return sRelay2Name;
    if (var == "sRelay3Name") return sRelay3Name;
    if (var == "sK1_Status") return String(Rel1Status);
    if (var == "sK2_Status") return String(Rel2Status);
    if (var == "sK3_Status") return String(Rel3Status);
    if (var == "sBoardInfo") return sBoardInfo;
    if (var == "sHeapspace") return sHeapspace;
    if (var == "sFS_USpace") return String(LittleFS.usedBytes());
    if (var == "sFS_TSpace") return String(LittleFS.totalBytes());
    if (var == "sAP_IP") return WiFi.softAPIP().toString();
    if (var == "sAP_Clients") return String(sAP_Station);
    if (var == "sCL_Addr") return WiFi.localIP().toString();
    if (var == "sCL_Status") return String(sCL_Status);
    if (var == "sVersion") return VersionSoftware;
    if (var == "CONFIGPLACEHOLDER") return processor(var);
    if (var == "BUTTONPLACEHOLDER") return SwitchRelais(var);
    return "NoVariable";
}


void initWebsite() {
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/favicon.ico", "image/x-icon");
    });
    server.on("/logo80.jpg", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/logo80.jpg", "image/jpg");
    });
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", String(), false, replaceVariable);
    });
    server.on("/system.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/system.html", String(), false, replaceVariable);
    });
    server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/settings.html", String(), false, replaceVariable);
    });
    server.on("/werte.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/werte.html", String(), false, replaceVariable);
    });
    server.on("/ueber.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/ueber.html", String(), false, replaceVariable);
    });
    server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(LittleFS, "/reboot.html", String(), false, processor);
        IsRebootRequired = true;
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
    });
    server.on("/settings.html", HTTP_POST, [](AsyncWebServerRequest *request){
       // Wird nicht genutzt, da Body-Handler verwendet wird
        }, 
        NULL, 
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String json = "";
        if (index == 0) json = ""; // Nur beim ersten Chunk leeren
        for (size_t i = 0; i < len; i++) {
            json += (char)data[i];
        }
        if (index + len == total) { // Letzter Chunk
            Serial.println("Empfangenes JSON (Body):");
            Serial.println(json);
            // Parse JSON so we can apply some settings immediately (no reboot needed)
            String jsonToSave = json; // default
            {
                StaticJsonDocument<1024> doc;
                DeserializationError err = deserializeJson(doc, json);
                if (!err) {
                    Serial.println("Parsed JSON:");
                    serializeJsonPretty(doc, Serial);
                    // Apply n2kDebug if provided
                    if (doc["n2kDebug"].is<bool>()) {
                        N2kDebug = doc["n2kDebug"].as<bool>();
                        Serial.printf("Applied n2kDebug = %s\n", N2kDebug?"true":"false");
                    } else if (doc["n2kDebug"].is<int>()) {
                        N2kDebug = (doc["n2kDebug"].as<int>() != 0);
                        Serial.printf("Applied n2kDebug = %s\n", N2kDebug?"true":"false");
                    }
                    // Apply acceptBroadcastCommands if provided so change is immediate
                    if (doc["acceptBroadcastCommands"].is<bool>()) {
                        AcceptBroadcastCommands = doc["acceptBroadcastCommands"].as<bool>();
                        Serial.printf("Applied acceptBroadcastCommands = %s\n", AcceptBroadcastCommands?"true":"false");
                    } else if (doc["acceptBroadcastCommands"].is<int>()) {
                        AcceptBroadcastCommands = (doc["acceptBroadcastCommands"].as<int>() != 0);
                        Serial.printf("Applied acceptBroadcastCommands = %s\n", AcceptBroadcastCommands?"true":"false");
                    }
                    // Ensure the saved JSON contains n2kDebug (so config persists the flag)
                    if (!doc["n2kDebug"].is<bool>() && !doc["n2kDebug"].is<int>()) {
                        doc["n2kDebug"] = N2kDebug;
                    }
                    // Ensure saved JSON contains acceptBroadcastCommands so it persists
                    if (!doc["acceptBroadcastCommands"].is<bool>() && !doc["acceptBroadcastCommands"].is<int>()) {
                        doc["acceptBroadcastCommands"] = AcceptBroadcastCommands;
                    }
                    // Serialize back to string for saving
                    jsonToSave = "";
                    serializeJson(doc, jsonToSave);
                } else {
                    Serial.print(F("deserializeJson() failed in settings handler: "));
                    Serial.println(err.f_str());
                }
            }

            if (writeConfig(jsonToSave)) {
                // Print saved config to Serial for verification
                File cfg = LittleFS.open("/config.json", "r");
                if (cfg) {
                    Serial.println("Saved config.json:");
                    while (cfg.available()) Serial.write(cfg.read());
                    cfg.close();
                }
                request->send(200, "text/plain", "Daten gespeichert");
            } else {
                request->send(500, "text/plain", "Fehler beim Speichern!");
            }
        }
    }
);
  
// Send a GET request to <ESP_IP>/update?relay=<inputMessage>&state=<inputMessage2>
    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
      if (request->hasParam("relay") && request->hasParam("state")){
        String relayIdStr = request->getParam("relay")->value();
        String relayStateStr = request->getParam("state")->value();

        int relayIndex = relayIdStr.toInt()-1;
        int state = relayStateStr.toInt();

        if (relayIndex >= 0 && relayIndex < NUM_RELAYS) {
            // Use logical setter to respect polarity
            SetRelayLogical(relayIndex, state != 0);
            Serial.print("Set relay logical state via web: ");
        } else {
            Serial.println("Ungültiger Relais-Index");
        }

        Serial.println("Relais: " + relayIdStr + " Zustand: " + relayStateStr);
        request->send(200, "text/plain", "OK");
      } else {
        request->send(400, "text/plain", "Fehlende Parameter");
    }
  });

        // Quick test endpoints for N2kDebug
        server.on("/n2kdebug", HTTP_GET, [](AsyncWebServerRequest *request) {
            if (request->hasParam("on")) {
                String v = request->getParam("on")->value();
                if (v == "1" || v.equalsIgnoreCase("true")) N2kDebug = true;
                else N2kDebug = false;
                Serial.printf("/n2kdebug set to %s\n", N2kDebug?"true":"false");
                request->send(200, "text/plain", N2kDebug?"1":"0");
            } else {
                request->send(200, "text/plain", N2kDebug?"1":"0");
            }
        });
        // Return last received N2k PGN and brief text
        server.on("/n2klast", HTTP_GET, [](AsyncWebServerRequest *request) {
            String out = "{";
            out += "\"lastPGN\": " + String(LastReceivedN2kPGN) + ",";
            out += "\"text\": \"" + LastReceivedN2kText + "\"";
            out += "}";
            request->send(200, "application/json", out);
        });
        // Return a small raw buffer of last N NMEA2000 messages (hex)
        server.on("/n2kraw", HTTP_GET, [](AsyncWebServerRequest *request) {
            String out = "[";
            for (uint8_t i = 0; i < N2KRAW_MAX; i++) {
                uint8_t idx = (N2kRawHead + i) % N2KRAW_MAX;
                N2kRawEntry &e = N2kRawBuf[idx];
                if (e.pgn == 0) continue; // empty slot
                if (out.length() > 1) out += ",";
                out += "{";
                out += "\"pgn\": " + String(e.pgn) + ",";
                out += "\"src\": " + String(e.src) + ",";
                out += "\"dst\": " + String(e.dst) + ",";
                out += "\"len\": " + String(e.len) + ",";
                out += "\"data\": \"";
                for (uint8_t b = 0; b < e.len; b++) {
                    char buf[4];
                    sprintf(buf, "%02X", e.data[b]);
                    out += String(buf);
                    if (b < e.len-1) out += " ";
                }
                out += "\"}";
            }
            out += "]";
            request->send(200, "application/json", out);
        });

        // Test endpoint: send a PGN126208 Group Function Command that targets PGN127502
        // Usage: /test126208?dst=<node|255>&field=<1..3>&val=<0|1>
        server.on("/test126208", HTTP_GET, [] (AsyncWebServerRequest *request) {
            unsigned int dst = NodeAddress;
            unsigned int field = 1;
            unsigned int val = 1;
            if (request->hasParam("dst")) dst = request->getParam("dst")->value().toInt();
            if (request->hasParam("field")) field = request->getParam("field")->value().toInt();
            if (request->hasParam("val")) val = request->getParam("val")->value().toInt();
            if (field < 1 || field > 3) {
                request->send(400, "text/plain", "field must be 1..3");
                return;
            }
            if (val > 1) val = 1;
            Serial.printf("Test126208: sending to dst=%u field=%u val=%u\n", dst, field, val);
            sendSwitchCommand((unsigned char)dst, (unsigned char)field, (unsigned char)val);
            request->send(200, "text/plain", "sent");
        });
}
