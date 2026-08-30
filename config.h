#ifndef CONFIG_H
#define CONFIG_H

#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

// Network
const char* ssid     = "";
const char* password = "";
const char* host     = "";
const int   port     = 5000;
const char* path     = "/api/sensors";

// Calibration
constexpr float TEMP_CALIB   = -4.0;
constexpr float PRESS_CALIB  = 15.0;
constexpr float HUM_CALIB    = 0.0;
constexpr float WIND_CALIB   = 1.1;
constexpr float PM25_CALIB   = 1.0;
constexpr float PM10_CALIB   = 1.0;

// Pins
constexpr int PMS_RX_PIN     = 20;
constexpr int SDA_PIN        = 6;
constexpr int SCL_PIN        = 7;
constexpr int GREEN_LED      = 2;
constexpr int IR_PIN         = 8;

// Timing
constexpr unsigned long SENSOR_READ_INTERVAL = 3000;
constexpr unsigned long SEND_INTERVAL        = 30000;
constexpr unsigned long WIFI_RETRY_INTERVAL  = 10000;
constexpr unsigned long DEBOUNCE_US          = 15000;

// Buffer
constexpr int BUFFER_SIZE = 15;
int bufferIndex = 0;
int bufferCount = 0;

// sensors
extern HardwareSerial pms;
extern Adafruit_BMP280 bmp;
extern Adafruit_AHTX0 aht;

bool bmp_ok = false;
bool aht_ok = false;

volatile unsigned long irPulseCount = 0;
volatile unsigned long lastIrTime = 0;
float windKmh = 0.0;
unsigned long lastWindCalc = 0;

#endif
