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
        buttons += "<p class=\"CInput\"><label>Oil Offset </label><input type = \"text\" name = \"MotorOffset\" value=\"";
        buttons += tAP_Config.wMotor_Offset;
        buttons += "\"/> &deg;C</p>";
        buttons += "<p class=\"CInput\"><label>K&uuml;hlwasser Offset </label><input type = \"text\" name = \"CoolantOffset\" value=\"";
        buttons += tAP_Config.wCoolant_Offset;
        buttons += "\"/> &deg;C</p>";
        buttons += "<p class=\"CInput\"><label>max. F&uuml;llstand </label><input type = \"text\" name = \"Fuellstandmax\" value=\"";
        buttons += tAP_Config.wFuellstandmax;
        buttons += "\"/> l</p>";
        buttons += "<p class=\"CInput\"><label>ADC1 Kalibrierung </label><input type = \"text\" name = \"ADC1_Cal\" value=\"";
        buttons += tAP_Config.wADC1_Cal;
        buttons += "\"/></p>";
        buttons += "<p class=\"CInput\"><label>ADC2 Kalibrierung </label><input type = \"text\" name = \"ADC2_Cal\" value=\"";
        buttons += tAP_Config.wADC2_Cal;
        buttons += "\"/></p>";
        buttons += "<p class=\"button\"><input type=\"submit\" value=\"Speichern\"></p>";
        buttons += "</form>";
        return buttons;
    }
    return String();
}

//Variables for website
String sCL_Status = sWifiStatus(WiFi.status());

String replaceVariable(const String& var)
{
    if (var == "sDrehzahl") return String(fDrehzahl, 1);
    if (var == "sFuellstand") return String(FuelLevel, 1);
    if (var == "sFuellstandmax") return String(FuelLevelMax, 1);
    if (var == "sBordspannung") return String(fBordSpannung, 1);
    if (var == "sCoolantTemp") return String(fCoolantTemp, 1);
    if (var == "sMotorTemp") return String(fMotorTemp, 1);
    if (var == "sCoolantOffset") return String(fCoolantOffset);
    if (var == "sMotorOffset") return String(fMotorOffset);
    if (var == "sMotorError") return String(motorErrorReported);
    if (var == "sCoolantError") return String(coolantErrorReported);
    if (var == "sBoardInfo") return sBoardInfo;
    if (var == "sADC1_Cal") return String(ADC_Calibration_Value1);
    if (var == "sADC2_Cal") return String(ADC_Calibration_Value2);
    if (var == "sHeapspace") return sHeapspace;
    if (var == "sFS_USpace") return String(LittleFS.usedBytes());
    if (var == "sFS_TSpace") return String(LittleFS.totalBytes());
    if (var == "sAP_IP") return WiFi.softAPIP().toString();
    if (var == "sAP_Clients") return String(sAP_Station);
    if (var == "sCL_Addr") return WiFi.localIP().toString();
    if (var == "sCL_Status") return String(sCL_Status);
    if (var == "sOneWire_Status") return String(sOneWire_Status);
    if (var == "sVersion") return Version;
    if (var == "sCounter") return String(Counter);
    if (var == "CONFIGPLACEHOLDER") return processor(var);
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
}

