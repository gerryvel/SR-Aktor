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
#include "configuration.h"
#include "boardinfo.h"
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <Arduino.h>

// Set web server port number to 80
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket server on port 81

// Info Board for HTML-Output
String sBoardInfo;
BoardInfo boardInfo;
bool IsRebootRequired = false;

String processor(const String& var)
{
    if (var == "CONFIGPLACEHOLDER")
    {
        String buttons = "";
        buttons += "<form onSubmit = \"event.preventDefault(); formToJson(this);\">";
        buttons += "<p class=\"CInput\"><label>SSID </label><input type = \"text\" name = \"SSID\" value=\"";
        buttons += tAP_Config.wAP_SSID;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>IP </label><input type = \"text\" name = \"IP\" value=\"";
        buttons += tAP_Config.wAP_IP;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>Password </label><input type = \"text\" name = \"Password\" value=\"";
        buttons += tAP_Config.wAP_Password;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>Relais 1 Name </label><input type = \"text\" name = \"sRelay1Name\" value=\"";
        buttons += tAP_Config.wRelay1Name;
        buttons += "\"/> </p>";
        buttons += "<p class=\"CInput\"><label>Relais 2 Name </label><input type = \"text\" name = \"sRelay2Name\" value=\"";
        buttons += tAP_Config.wRelay2Name;
        buttons += "\"/> </p>";
        buttons += "<p class=\"CInput\"><label>Relais 3 Name </label><input type = \"text\" name = \"sRelay3Name\" value=\"";
        buttons += tAP_Config.wRelay3Name;
        buttons += "\"/> </p>";
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
    if (var == "sVersion") return Version;
    if (var == "CONFIGPLACEHOLDER") return processor(var);
    if (var == "BUTTONPLACEHOLDER") return SwitchRelais(var);
    return "NoVariable";
}


void website() {
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
    server.on("/gauge.min.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/gauge.min.js");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
    });
    server.on("/settings.html", HTTP_POST, [](AsyncWebServerRequest *request)
    {
        int count = request->params();
        Serial.printf("Anzahl: %i\n", count);
        for (int i = 0; i < count; i++)
        {
            AsyncWebParameter* p = request->getParam(i);
            Serial.print("PWerte von der Internet - Seite: ");
            Serial.print("Param name: ");
            Serial.println(p->name());
            Serial.print("Param value: ");
            Serial.println(p->value());
            Serial.println("------");
            // p->value in die config schreiben
            writeConfig(p->name(), p->value());
        }
        request->send(200, "text/plain", "Daten gespeichert");
    });

// Send a GET request to <ESP_IP>/update?relay=<inputMessage>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    String inputMessage2;
    String inputParam2;
    // GET input1 value on <ESP_IP>/update?relay=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1) & request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      inputParam2 = PARAM_INPUT_2;
      if(RELAY_NO){
        Serial.print("NO ");
        digitalWrite(Relais[inputMessage.toInt()]-1, !inputMessage2.toInt());
      }
      else{
        Serial.print("NC ");
        digitalWrite(Relais[inputMessage.toInt()-1], inputMessage2.toInt());
      }
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage + inputMessage2);
    request->send(200, "text/plain", "OK");
  });
}

