#include <FS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

// Wifi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

// HTTP requests
#include <ESP8266HTTPClient.h>

// OTA updates
#include <ESP8266httpUpdate.h>
// Blynk
#include <BlynkSimpleEsp8266.h>

// JSON
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson


// GPIO Defines
#define I2C_SDA 0 // D3
#define I2C_SCL 2 // D4

// Humidity/Temperature SI7021
#include <SI7021.h>
SI7021 si7021;

// #define SW_SERIAL_RX 12 // D6
// #define SW_SERIAL_TX 15 // D8

#include <Wire.h>

// Pressure and Temperature
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

// Use i2c OLED Lib
#include "OLED.h"

// Handy timers
#include <SimpleTimer.h>

#include <SoftwareSerial.h>

// SoftwareSerial swSerial(SW_SERIAL_RX, SW_SERIAL_TX);

// CO2 SERIAL
#define DEBUG_SERIAL Serial1
#define SENSOR_SERIAL Serial

byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
unsigned char response[7];

// Pressure, temperature and humidity sensor
Adafruit_BMP085 bme;

OLED lcd(I2C_SDA, I2C_SCL);

// Blynk token
char blynk_token[33] {"Blynk token"};
const char blynk_domain[] {"ezagro.kumekay.com"};
const uint16_t blynk_port {8442};

// Device Id
char device_id[17] = "Device ID";
char fw_ver[17] = "v0.0.4";

// Handy timer
SimpleTimer timer;

// Setup Wifi connection
WiFiManager wifiManager;

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying the need to save config
void saveConfigCallback() {
        DEBUG_SERIAL.println("Should save config");
        shouldSaveConfig = true;
}

void factoryReset() {
        wifiManager.resetSettings();
        SPIFFS.format();
        ESP.reset();
}


char *c_str(String s) {
  char *cstr = new char[s.length() + 1];
  return strcpy(cstr, s.c_str());
}

void printString(String str,  uint8_t r=0, uint8_t c=0) {
  char *line { c_str(String(str))};
  lcd.print(line, r, c);
  DEBUG_SERIAL.println(line);
}

String readCO2() {
  // CO2
  String co2 {"--"};
  bool header_found {false};
  char tries {0};

  SENSOR_SERIAL.write(cmd, 9);
  memset(response, 0, 7);

  // Looking for packet start
  while(SENSOR_SERIAL.available() && (!header_found)) {
          if(SENSOR_SERIAL.read() == 0xff ) {
                  if(SENSOR_SERIAL.read() == 0x86 ) header_found = true;
          }
  }

  if (header_found) {
          SENSOR_SERIAL.readBytes(response, 7);

          byte crc = 0x86;
          for (char i = 0; i < 6; i++) {
                  crc+=response[i];
          }
          crc = 0xff - crc;
          crc++;

          if ( !(response[6] == crc) ) {
                  DEBUG_SERIAL.println("CO2: CRC error: " + String(crc) + " / "+ String(response[6]));
          } else {
                  unsigned int responseHigh = (unsigned int) response[0];
                  unsigned int responseLow = (unsigned int) response[1];
                  unsigned int ppm = (256*responseHigh) + responseLow;
                  co2 = String(ppm);
                  DEBUG_SERIAL.println("CO2:" + String(co2));
          }
  } else {
          DEBUG_SERIAL.println("CO2: Header not found");
  }

  return co2;
}

void sendMeasurements() {
        lcd.clear();
        printString(String(device_id) + " " + String(fw_ver));
        // H/T
        float tf = si7021.getCelsiusHundredths() / 100.0 ;
        int hi = si7021.getHumidityPercent();

        auto h = String(hi);
        auto t = String(tf, 0);

        if (isnan(hi) || isnan(tf) ) {
                h = "--";
                t = "--";
        }

        Blynk.virtualWrite(V1, t);
        Blynk.virtualWrite(V2, h);

        printString("H: " + h + "% T: " + t + "C", 1);

        // P/T2
        auto t2 = String(bme.readTemperature(), 0);
        auto p = String(bme.readPressure());

        printString("P: " + p + "Pa T: " + t2 + "C", 2);

        Blynk.virtualWrite(V3, t2);
        Blynk.virtualWrite(V4, p);


        String co2 = readCO2();
        printString("CO2: " + co2 + "ppm", 3);

        Blynk.virtualWrite(V5, co2);
}

bool loadConfig() {
        File configFile = SPIFFS.open("/config.json", "r");
        if (!configFile) {
                DEBUG_SERIAL.println("Failed to open config file");
                return false;
        }

        size_t size = configFile.size();
        if (size > 1024) {
                DEBUG_SERIAL.println("Config file size is too large");
                return false;
        }

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        // We don't use String here because ArduinoJson library requires the input
        // buffer to be mutable. If you don't use ArduinoJson, you may as well
        // use configFile.readString instead.
        configFile.readBytes(buf.get(), size);

        StaticJsonBuffer<200> jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());

        if (!json.success()) {
                DEBUG_SERIAL.println("Failed to parse config file");
                return false;
        }

        // Save parameters
        strcpy(device_id, json["device_id"]);
        strcpy(blynk_token, json["blynk_token"]);
}

void setupWiFi() {
        //set config save notify callback
        wifiManager.setSaveConfigCallback(saveConfigCallback);

        // Custom parameters
        WiFiManagerParameter custom_device_id("device_id", "Device ID", device_id, 16);
        WiFiManagerParameter custom_blynk_token("blynk", "Blynk token", blynk_token, 34);
        wifiManager.addParameter(&custom_blynk_token);
        wifiManager.addParameter(&custom_device_id);

        lcd.clear();

        // wifiManager.setTimeout(180);

        String ssid { "ku_" +  String(ESP.getChipId())};
        String pass {"ku_pass_" + String(ESP.getFlashChipId()) };

        printString("Connect to WiFi:",1);
        printString("Network: " + ssid,2);
        printString("Password: "+ pass,3);
        printString("And open browser:", 4);
        printString("http://192.168.4.1", 5);
        printString("to set up your device", 6);

        if (!wifiManager.autoConnect(ssid.c_str(), pass.c_str())) {
                DEBUG_SERIAL.println("failed to connect and hit timeout");
                delay(3000);
                //reset and try again, or maybe put it to deep sleep
                ESP.reset();
                delay(5000);
        }

        //save the custom parameters to FS
        if (shouldSaveConfig) {
                DEBUG_SERIAL.println("saving config");
                DynamicJsonBuffer jsonBuffer;
                JsonObject &json = jsonBuffer.createObject();
                json["device_id"] = custom_device_id.getValue();
                json["blynk_token"] = custom_blynk_token.getValue();

                File configFile = SPIFFS.open("/config.json", "w");
                if (!configFile) {
                        DEBUG_SERIAL.println("failed to open config file for writing");
                }

                json.printTo(DEBUG_SERIAL);
                json.printTo(configFile);
                configFile.close();
                //end save
        }

        //if you get here you have connected to the WiFi
        DEBUG_SERIAL.println("WiFi connected");

        DEBUG_SERIAL.print("IP address: ");
        DEBUG_SERIAL.println(WiFi.localIP());
}

void wifiModeCallback(WiFiManager *myWiFiManager) {
        DEBUG_SERIAL.println("Entered config mode");
        DEBUG_SERIAL.println(WiFi.softAPIP());
}

// Virtual pin update FW
BLYNK_WRITE(V22) {
        if (param.asInt() == 1) {
                DEBUG_SERIAL.println("Got a FW update request");

                char full_version[34] {""};
                strcat(full_version, device_id);
                strcat(full_version, "::");
                strcat(full_version, fw_ver);

                t_httpUpdate_return ret = ESPhttpUpdate.update("http://firmware.ezagro.kumekay.com/update", full_version);
                switch (ret) {
                case HTTP_UPDATE_FAILED:
                        DEBUG_SERIAL.println("[update] Update failed.");
                        break;
                case HTTP_UPDATE_NO_UPDATES:
                        DEBUG_SERIAL.println("[update] Update no Update.");
                        break;
                case HTTP_UPDATE_OK:
                        DEBUG_SERIAL.println("[update] Update ok.");
                        break;
                }

        }
}

// Virtual pin reset settings
BLYNK_WRITE(V23) {
        factoryReset();
}

void setup() {
        // Init serial ports
        DEBUG_SERIAL.begin(115200);

        SENSOR_SERIAL.begin(9600);

        // Init I2C interface
        Wire.begin(I2C_SDA, I2C_SCL);

        // Init Humidity/Temperature sensor
        si7021.begin(I2C_SDA, I2C_SCL);

        // Init Pressure/Temperature sensor
        if (!bme.begin()) {
                DEBUG_SERIAL.println("Could not find a valid BMP085 sensor, check wiring!");
        }

        // Init LCD display
        lcd.begin();

        // Init filesystem
        if (!SPIFFS.begin()) {
                DEBUG_SERIAL.println("Failed to mount file system");
                ESP.reset();
        }

        // Setup WiFi
        setupWiFi();

        // Load config
        if (!loadConfig()) {
                DEBUG_SERIAL.println("Failed to load config");
                factoryReset();
        } else {
                DEBUG_SERIAL.println("Config loaded");
        }

        // Start blynk
        Blynk.config(blynk_token, blynk_domain, blynk_port);

        // Setup a function to be called every 10 second
        timer.setInterval(10000L, sendMeasurements);

        sendMeasurements();
}

void loop() {
        Blynk.run();
        timer.run();
}
