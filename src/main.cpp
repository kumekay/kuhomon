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

// Use I2C
#define I2C_SDA 2
#define I2C_SCL 14

#include <Wire.h>

// Pressure and Temperature
#include <Adafruit_BMP085.h>

// Use TFT_ILI9163C display library
#include <TFT_ILI9163C.h>

// Temperature and Humidity
#include "DHT.h" // https://github.com/adafruit/DHT-sensor-library

// Handy timers
#include <SimpleTimer.h>

// dht sensor
#define DHTPIN 12
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE, 15);

// Pressure and temperature sensor
Adafruit_BMP085 bmp;

// Serial management
#define DATA_SERIAL Serial
#define SENSOR_SERIAL Serial

// lcd
#define __CS  16  //(D0)
#define __DC  5   //(D1)
#define __RST 4   //(D2)

/*
 SCLK:D5
 MOSI:D7
*/

TFT_ILI9163C lcd = TFT_ILI9163C(__CS, __DC, __RST);


// Blynk token
char blynk_token[33]{"Blynk token"};
const char blynk_domain[]{"ezagro.kumekay.com"};
const uint16_t blynk_port{8442};

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
    DATA_SERIAL.println("Should save config");
    shouldSaveConfig = true;
}

void factoryReset() {
    wifiManager.resetSettings();
    SPIFFS.format();
    ESP.reset();
}

void sendMeasurements() {
    auto h = String(dht.readHumidity(), 0);
    auto t = String(dht.readTemperature(), 0);

    auto t2 = String(bmp.readTemperature(), 0);
    auto p = String(bmp.readPressure(), 0);

//    auto co2 = String(readCO2Level(), 0);

    // Send data to Blynk
    Blynk.virtualWrite(V1, h);
    Blynk.virtualWrite(V2, t);
//    Blynk.virtualWrite(V3, t2);
//    Blynk.virtualWrite(V4, p);
//    Blynk.virtualWrite(V5, co2);

//    // Clear LCD
//    lcd.clear();
//
//    // add logo
    lcd.setCursor(2, 2);
    lcd.print(device_id);
    lcd.print(" ");
    lcd.println(fw_ver);

    // Print data
    const char rows {2};
    String row[rows] {"H: " + h + "% T: " + t + "C",
    "P: " + p + "Pa T: " + t2 + "C"};
    for (char i=0; i < rows; i++) {
        lcd.println(row[i]);
        DATA_SERIAL.println(row[i]);
    }
}

//void showData() {
//
//}

// Source http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19/mh-z19-co2-manual(ver1_0).pdf
bool getCheckSum(char *packet) {
    char checksum{0xFF};
    for (char i = 1; i < 8; i++)
        checksum -= packet[i];

    checksum++;

    return packet[8] == checksum;
}

int readCO2Level() {
    const char request[9]{0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    char response[9];

    SENSOR_SERIAL.write(request, 9);
    SENSOR_SERIAL.readBytes(response, 9);

    if (getCheckSum(response)) {
        int ppm = (256 * response[2]) + response[3];
        DATA_SERIAL.println("CO2 Level: " + String(ppm) + "ppm");
        return ppm;
    } else {
        DATA_SERIAL.println("CO2 Sensor CRC error!");
        return -1;
    }
}

bool loadConfig() {
    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        DATA_SERIAL.println("Failed to open config file");
        return false;
    }

    size_t size = configFile.size();
    if (size > 1024) {
        DATA_SERIAL.println("Config file size is too large");
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
        DATA_SERIAL.println("Failed to parse config file");
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

    // lcd.clear();

    // add logo
    lcd.setCursor(0, 0);
    lcd.print("Connect to WiFi:");
    lcd.setCursor(0, 1);
    lcd.print("EZagro ezsecret");

    // wifiManager.setTimeout(180);
    if (!wifiManager.autoConnect("EZagro", "ezsecret")) {
        DATA_SERIAL.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
    }

    //save the custom parameters to FS
    if (shouldSaveConfig) {
        DATA_SERIAL.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.createObject();
        json["device_id"] = custom_device_id.getValue();
        json["blynk_token"] = custom_blynk_token.getValue();

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            DATA_SERIAL.println("failed to open config file for writing");
        }

        json.printTo(DATA_SERIAL);
        json.printTo(configFile);
        configFile.close();
        //end save
    }

    //if you get here you have connected to the WiFi
    DATA_SERIAL.println("WiFi connected");

    DATA_SERIAL.print("IP address: ");
    DATA_SERIAL.println(WiFi.localIP());
}

void wifiModeCallback(WiFiManager *myWiFiManager) {
    DATA_SERIAL.println("Entered config mode");
    DATA_SERIAL.println(WiFi.softAPIP());
}

// Virtual pin update FW
BLYNK_WRITE(V22) {
    if (param.asInt() == 1) {
        DATA_SERIAL.println("Got a FW update request");

        char full_version[34]{""};
        strcat(full_version, device_id);
        strcat(full_version, "::");
        strcat(full_version, fw_ver);

        t_httpUpdate_return ret = ESPhttpUpdate.update("http://firmware.ezagro.kumekay.com/update", full_version);
        switch (ret) {
            case HTTP_UPDATE_FAILED:
                DATA_SERIAL.println("[update] Update failed.");
                break;
            case HTTP_UPDATE_NO_UPDATES:
                DATA_SERIAL.println("[update] Update no Update.");
                break;
            case HTTP_UPDATE_OK:
                DATA_SERIAL.println("[update] Update ok.");
                break;
        }

    }
}

// Virtual pin reset settings
BLYNK_WRITE(V23) {
    factoryReset();
}

void setup() {
    // Init serial port
    DATA_SERIAL.begin(115200);

    // Init I2C interface
    Wire.begin(I2C_SDA, I2C_SCL);

    // Init Humidity/Temperature sensor
    dht.begin();

    // Init Pressure/Temperature sensor
    if (!bmp.begin()) {
        DATA_SERIAL.println("Could not find a valid BMP085 sensor, check wiring!");
    }

    // Init LCD display
    lcd.begin();


    // Init filesystem
    if (!SPIFFS.begin()) {
        DATA_SERIAL.println("Failed to mount file system");
        ESP.reset();
    }

    // Setup WiFi
    setupWiFi();

    // Load config
    if (!loadConfig()) {
        DATA_SERIAL.println("Failed to load config");
        factoryReset();
    } else {
        DATA_SERIAL.println("Config loaded");
    }

    // Start blynk
    Blynk.config(blynk_token, blynk_domain, blynk_port);

    // Setup a function to be called every 5 second
    timer.setInterval(10000L, sendMeasurements);
}

void loop() {
    Blynk.run();
    timer.run();
}
