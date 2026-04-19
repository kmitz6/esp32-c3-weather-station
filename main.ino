#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

// ============================================================
// Network credentials
// ============================================================
const char* ssid     = "";
const char* password = "";
const char* host     = "";
const int   port     = 5000;
const char* path     = "/api/sensors";

// ============================================================
// CALIBRATION OFFSETS (additive corrections)
// ============================================================
// These values are added to the raw aggregated values
// before sending to the server.
// Example: if temperature reads 4°C too high, set TEMP_CALIB = -4.0
float TEMP_CALIB     = -4.0;   // subtract 4°C
float PRESS_CALIB    = 15.0;   // add 15 hPa
float HUM_CALIB      = 0.0;    // no change
float WIND_CALIB     = 0.0;    // no change
int   PM25_CALIB     = 0;      // no change
int   PM10_CALIB     = 0;      // no change
// ============================================================

// PMS5003
HardwareSerial pms(1);
const int rxPin = 20;

// I2C sensors
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
const int sdaPin = 6;
const int sclPin = 7;
bool bmp_ok = false;
bool aht_ok = false;

// LED
const int GREEN_LED = 2;

// Anemometer
const int IR_PIN = 8;
volatile unsigned long irPulseCount = 0;
volatile unsigned long lastIrTime = 0;
const unsigned long DEBOUNCE_US = 5000;
float windKmh = 0.0;
unsigned long lastWindCalc = 0;
int pulsesPerMin = 0;

// Rolling buffer for last 15 readings
const int BUFFER_SIZE = 15;
float tempBuffer[BUFFER_SIZE];
float pressBuffer[BUFFER_SIZE];
float humBuffer[BUFFER_SIZE];
float windBuffer[BUFFER_SIZE];
int pm25Buffer[BUFFER_SIZE];
int pm10Buffer[BUFFER_SIZE];

int bufferIndex = 0;
int bufferCount = 0;

// Current raw instantaneous values
int pm25 = 0, pm10 = 0;
float temperature = 0.0f;
float pressure = 0.0f;
float humidity = 0.0f;
bool sensorSeen = false;

// Timing
unsigned long lastSend = 0;
unsigned long lastValid = 0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 2000;  // 2 seconds
const unsigned long SEND_INTERVAL = 10000;        // 10 seconds

// Recovery
unsigned long lastAnySensorOk = 0;
bool sensorsNeedReinit = false;

// ------------------------------------------------------------------
// Interrupt for wind sensor
// ------------------------------------------------------------------
void IRAM_ATTR irISR() {
  unsigned long now = micros();
  if (now - lastIrTime > DEBOUNCE_US) {
    irPulseCount++;
    lastIrTime = now;
  }
}

// ------------------------------------------------------------------
// Wind speed calculation (raw)
// ------------------------------------------------------------------
void updatePulsesPerMin() {
  unsigned long now = millis();
  unsigned long dt = now - lastWindCalc;
  if (dt >= 3000) {
    float pulsesPerSec = (float)irPulseCount / (dt / 1000.0);
    pulsesPerMin = (int)(pulsesPerSec * 60.0);
    windKmh = pulsesPerMin * 0.04;
    Serial.print("RAW: Wind - pulses in last ");
    Serial.print(dt);
    Serial.print(" ms: ");
    Serial.print(irPulseCount);
    Serial.print(" -> ");
    Serial.print(pulsesPerMin);
    Serial.print(" pulses/min (~");
    Serial.print(windKmh);
    Serial.println(" kmh)");
    
    irPulseCount = 0;
    lastWindCalc = now;
  }
}

// ------------------------------------------------------------------
// PMS5003 reader (raw)
// ------------------------------------------------------------------
bool readPMS() {
  while (pms.available()) {
    if (pms.peek() == 0x42) break;
    pms.read();
    delay(1);
  }
  if (pms.available() < 32) return false;
  uint8_t buf[32];
  if (pms.readBytes(buf, 32) != 32) return false;
  if (buf[0] != 0x42 || buf[1] != 0x4D) return false;
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += buf[i];
  uint16_t chk = (buf[30] << 8) | buf[31];
  if (sum != chk) return false;
  pm25 = (buf[4] << 8) | buf[5];
  pm10 = (buf[6] << 8) | buf[7];
  Serial.print("RAW: Air quality - PM2.5: "); Serial.print(pm25);
  Serial.print(", PM10: "); Serial.println(pm10);
  return true;
}

// ------------------------------------------------------------------
// Read environment (raw, without calibration)
// ------------------------------------------------------------------
bool readEnvironment() {
  bool any_success = false;
  if (bmp_ok) {
    float t = bmp.readTemperature();
    float p = bmp.readPressure() / 100.0F;
    if (!isnan(t) && !isnan(p)) {
      temperature = t;
      pressure = p;
      any_success = true;
      Serial.print("RAW: BMP280 - Temp: "); Serial.print(temperature);
      Serial.print(" °C, Pressure: "); Serial.print(pressure); Serial.println(" hPa");
    } else {
      Serial.println("WARN: BMP280 read failed");
    }
  }
  if (aht_ok) {
    sensors_event_t humidity_event, temp_event;
    if (aht.getEvent(&humidity_event, &temp_event)) {
      humidity = humidity_event.relative_humidity;
      any_success = true;
      Serial.print("RAW: AHT20 - Humidity: "); Serial.print(humidity); Serial.println(" %");
    } else {
      Serial.println("WARN: AHT20 read failed");
    }
  }
  return any_success;
}

// ------------------------------------------------------------------
// Store raw readings into rolling buffer
// ------------------------------------------------------------------
void storeToBuffer() {
  tempBuffer[bufferIndex] = temperature;
  pressBuffer[bufferIndex] = pressure;
  humBuffer[bufferIndex] = humidity;
  windBuffer[bufferIndex] = windKmh;
  pm25Buffer[bufferIndex] = pm25;
  pm10Buffer[bufferIndex] = pm10;
  
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) bufferCount++;
}

// ------------------------------------------------------------------
// Compute aggregates from raw buffer
// ------------------------------------------------------------------
void computeRawAggregates(float &avgTemp, float &avgPress, float &avgHum,
                          float &maxWind, int &maxPM25, int &maxPM10) {
  if (bufferCount == 0) {
    avgTemp = avgPress = avgHum = 0.0f;
    maxWind = 0.0f;
    maxPM25 = maxPM10 = 0;
    return;
  }
  float sumTemp = 0, sumPress = 0, sumHum = 0;
  float maxW = -1e6;
  int maxP25 = -1, maxP10 = -1;
  for (int i = 0; i < bufferCount; i++) {
    sumTemp += tempBuffer[i];
    sumPress += pressBuffer[i];
    sumHum += humBuffer[i];
    if (windBuffer[i] > maxW) maxW = windBuffer[i];
    if (pm25Buffer[i] > maxP25) maxP25 = pm25Buffer[i];
    if (pm10Buffer[i] > maxP10) maxP10 = pm10Buffer[i];
  }
  avgTemp = sumTemp / bufferCount;
  avgPress = sumPress / bufferCount;
  avgHum = sumHum / bufferCount;
  maxWind = maxW;
  maxPM25 = maxP25;
  maxPM10 = maxP10;
}

// ------------------------------------------------------------------
// Print raw aggregates to serial
// ------------------------------------------------------------------
void printAggregates() {
  float avgT, avgP, avgH, maxW;
  int maxP25, maxP10;
  computeRawAggregates(avgT, avgP, avgH, maxW, maxP25, maxP10);
  Serial.println("--- Aggregates (raw, last " + String(bufferCount) + " readings) ---");
  Serial.print("Avg Temp: "); Serial.print(avgT); Serial.print(" °C, ");
  Serial.print("Avg Press: "); Serial.print(avgP); Serial.print(" hPa, ");
  Serial.print("Avg Hum: "); Serial.print(avgH); Serial.println(" %");
  Serial.print("Max Wind: "); Serial.print(maxW); Serial.print(" km/h, ");
  Serial.print("Max PM2.5: "); Serial.print(maxP25); Serial.print(", ");
  Serial.print("Max PM10: "); Serial.println(maxP10);
  Serial.println("----------------------------------------");
}

// ------------------------------------------------------------------
// Reinit sensors
// ------------------------------------------------------------------
void reinitSensors() {
  Serial.println("REINIT: Restarting sensors.");
  Wire.end(); delay(100);
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);
  bmp_ok = false;
  if (bmp.begin(0x76)) bmp_ok = true;
  else if (bmp.begin(0x77)) bmp_ok = true;
  else Serial.println("REINIT: BMP280 not found");
  aht_ok = aht.begin();
  if (aht_ok) Serial.println("REINIT: AHT20 OK");
  else Serial.println("REINIT: AHT20 not found");
  pms.end(); delay(100);
  pms.begin(9600, SERIAL_8N1, rxPin);
  sensorsNeedReinit = false;
}

// ------------------------------------------------------------------
// WiFi reconnect
// ------------------------------------------------------------------
void WiFiReconnect() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("WIFI: Reconnecting...");
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nINFO: WiFi reconnected");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nERR: WiFi reconnect timeout");
  }
}

// ------------------------------------------------------------------
// Send data to server (applies calibration offsets to the aggregates)
// ------------------------------------------------------------------
void sendData(float rawAvgTemp, float rawAvgPress, float rawAvgHum,
              float rawMaxWind, int rawMaxPM25, int rawMaxPM10) {
  // Apply calibration offsets
  float calibTemp = rawAvgTemp + TEMP_CALIB;
  float calibPress = rawAvgPress + PRESS_CALIB;
  float calibHum = rawAvgHum + HUM_CALIB;
  float calibWind = rawMaxWind + WIND_CALIB;
  int calibPM25 = rawMaxPM25 + PM25_CALIB;
  int calibPM10 = rawMaxPM10 + PM10_CALIB;

  // Ensure no negative values for PM (optional)
  if (calibPM25 < 0) calibPM25 = 0;
  if (calibPM10 < 0) calibPM10 = 0;
  if (calibWind < 0) calibWind = 0;

  Serial.println("--- Sending to server (calibrated) ---");
  Serial.print("Calibrated Temp: "); Serial.print(calibTemp);
  Serial.print(" °C, Press: "); Serial.print(calibPress);
  Serial.print(" hPa, Hum: "); Serial.print(calibHum);
  Serial.print(" %, Wind: "); Serial.print(calibWind);
  Serial.print(" km/h, PM2.5: "); Serial.print(calibPM25);
  Serial.print(", PM10: "); Serial.println(calibPM10);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR: WiFi not connected");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH); delay(250);
      digitalWrite(GREEN_LED, LOW); delay(250);
    }
    return;
  }
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("ERR: Server connection failed");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH); delay(250);
      digitalWrite(GREEN_LED, LOW); delay(250);
    }
    return;
  }
  char payload[512];
  snprintf(payload, sizeof(payload),
           "{\"pm25\":%d,\"pm10\":%d,\"temp\":%.1f,\"pressure\":%.1f,\"humidity\":%.1f,\"wind_kmh\":%.1f,\"ts\":%lu}",
           calibPM25, calibPM10, calibTemp, calibPress, calibHum, calibWind, millis());
  char httpBuffer[512];
  int len = snprintf(httpBuffer, sizeof(httpBuffer),
                     "POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s",
                     path, host, strlen(payload), payload);
  client.write((uint8_t*)httpBuffer, len);
  unsigned long timeout = millis();
  bool gotHTTP200 = false;
  while (millis() - timeout < 5000) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line.startsWith("HTTP/1.1 200") || line.startsWith("HTTP/1.0 200")) {
        gotHTTP200 = true;
      }
    }
  }
  client.stop();
  if (gotHTTP200) {
    Serial.println("INFO: Data sent successfully");
    digitalWrite(GREEN_LED, HIGH); delay(750); digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("ERR: HTTP Error!");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH); delay(250);
      digitalWrite(GREEN_LED, LOW); delay(250);
    }
  }
}

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  Serial.println("INFO: Booting weather station");

  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irISR, FALLING);
  lastWindCalc = millis();

  pms.begin(9600, SERIAL_8N1, rxPin);
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);
  if (bmp.begin(0x76)) bmp_ok = true;
  else if (bmp.begin(0x77)) bmp_ok = true;
  else Serial.println("ERR: BMP280 not detected");
  aht_ok = aht.begin();
  if (aht_ok) Serial.println("INFO: AHT20 found");
  else Serial.println("ERR: AHT20 not detected");

  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.print("INFO: Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nINFO: WiFi connected");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nERR: WiFi timeout");
  }

  lastValid = millis();
  lastAnySensorOk = millis();
  lastSensorRead = millis();
  lastSend = millis();
}

// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {
  updatePulsesPerMin();

  // Read sensors every 2 seconds
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    bool pms_ok = readPMS();
    bool env_ok = readEnvironment();
    if (pms_ok || env_ok) {
      sensorSeen = true;
      lastValid = millis();
      lastAnySensorOk = millis();
      sensorsNeedReinit = false;
      storeToBuffer();                // store raw readings
      printAggregates();              // print raw aggregates
    } else {
      if (millis() - lastValid > 30000) sensorSeen = false;
      if (millis() - lastAnySensorOk > 60000 && !sensorsNeedReinit) {
        sensorsNeedReinit = true;
        Serial.println("ERR: 60s with no data, reinit sensors");
      }
    }
  }

  if (sensorsNeedReinit) {
    reinitSensors();
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) WiFiReconnect();

  // Send data every 10 seconds using raw aggregates + calibration
  if (millis() - lastSend >= SEND_INTERVAL) {
    if (bufferCount > 0) {
      float avgTemp, avgPress, avgHum, maxWind;
      int maxPM25, maxPM10;
      computeRawAggregates(avgTemp, avgPress, avgHum, maxWind, maxPM25, maxPM10);
      sendData(avgTemp, avgPress, avgHum, maxWind, maxPM25, maxPM10);
    } else {
      Serial.println("WARN: No readings in buffer, sending zeros.");
      sendData(0.0, 0.0, 0.0, 0.0, 0, 0);
    }
    lastSend = millis();
  }
}
