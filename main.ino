#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "ScioSense_ENS160.h"
#include "Adafruit_AHTX0.h"

// --- I2cInterface class ---
class I2cInterface {
public:
  void begin(TwoWire &wire, uint8_t address) {
    _wire = &wire;
    _address = address;
  }

  bool writeRegister(uint8_t reg, uint8_t *data, size_t len) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    for (size_t i = 0; i < len; i++) _wire->write(data[i]);
    return (_wire->endTransmission() == 0);
  }

  bool readRegister(uint8_t reg, uint8_t *data, size_t len) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom(_address, len) != len) return false;
    for (size_t i = 0; i < len; i++) data[i] = _wire->read();
    return true;
  }

private:
  TwoWire *_wire;
  uint8_t _address;
};


// --- Configuration ---
const char* WIFI_SSID     = "YourWiFiSSID";
const char* WIFI_PASSWORD = "YourWiFiPassword";
const char* SERVER_HOST   = "192.168.1.100";   // Your Flask server IP
const int   SERVER_PORT   = 5000;
const long READ_INTERVAL = 10000;

#define I2C_SDA 8
#define I2C_SCL 9

// --- Globals ---
I2cInterface i2c;
ScioSense_ENS160 ens160;
Adafruit_AHTX0 aht;
unsigned long lastReadTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting ENS160 + AHT21 reader...");

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected, IP: " + WiFi.localIP().toString());

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // Initialize ENS160 (try 0x53 then 0x52)
  i2c.begin(Wire, 0x53);
  if (!ens160.begin(&i2c)) {
    i2c.begin(Wire, 0x52);
    if (!ens160.begin(&i2c)) {
      Serial.println("ENS160 not found! Check wiring.");
    } else {
      Serial.println("ENS160 initialized at 0x52.");
      ens160.startStandardMeasure();
    }
  } else {
    Serial.println("ENS160 initialized at 0x53.");
    ens160.startStandardMeasure();
  }

  // Initialize AHT21
  if (!aht.begin()) {
    Serial.println("AHT21 not found! Check wiring.");
  } else {
    Serial.println("AHT21 initialized.");
  }
}

void sendToFlask(uint16_t co2, uint16_t tvoc, float temp, float hum) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "http://" + String(SERVER_HOST) + ":" + String(SERVER_PORT) + "/api/indoor_sensors";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> doc;
  doc["co2"] = co2;
  doc["tvoc"] = tvoc;
  doc["temp_aht21"] = temp;
  doc["hum_aht21"] = hum;
  doc["temp_bmp280"] = 0.0;
  doc["pressure_bmp280"] = 0.0;

  String jsonString;
  serializeJson(doc, jsonString);
  int httpCode = http.POST(jsonString);
  if (httpCode == 200) {
    Serial.println("Data sent successfully.");
  } else {
    Serial.printf("HTTP POST failed, code: %d\n", httpCode);
  }
  http.end();
}

void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    // Read ENS160 (following original logic)
    uint16_t eco2 = 0, tvoc = 0;
    if (ens160.update() == ScioSense::ENS16x::Result::Ok) {
      if (hasFlag(ens160.getDeviceStatus(), ScioSense::ENS16x::DeviceStatus::NewData)) {
        tvoc = ens160.getTvoc();
        eco2 = ens160.getEco2();
        Serial.printf("ENS160: eCO₂=%d ppm, TVOC=%d ppb\n", eco2, tvoc);
      } else {
        Serial.println("ENS160: No new data.");
      }
    } else {
      Serial.println("ENS160 update failed.");
    }

    // Read AHT21
    float temperature = 0.0, humidity = 0.0;
    sensors_event_t humidity_event, temp_event;
    if (aht.getEvent(&humidity_event, &temp_event)) {
      temperature = temp_event.temperature;
      humidity = humidity_event.relative_humidity;
      Serial.printf("AHT21: T=%.2f °C, H=%.1f %%\n", temperature, humidity);
    } else {
      Serial.println("AHT21 read failed.");
    }

    sendToFlask(eco2, tvoc, temperature, humidity);
  }
  delay(100);
}
