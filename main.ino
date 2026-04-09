#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

// Network credentials
const char* ssid     = "";
const char* password = "";
const char* host     = "";
const int   port     = 5000;
const char* path     = "/api/sensors";

// PMS5003 air quality sensor
HardwareSerial pms(1);
const int rxPin = 20;  // PMS TX -> ESP RX

// I2C sensors (BMP280 + AHT20)
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
const int sdaPin = 6;
const int sclPin = 7;
bool bmp_ok = false;
bool aht_ok = false;

// transmit LED
const int GREEN_LED = 2;

// anemometer
const int IR_PIN = 8;
volatile unsigned long irPulseCount = 0;
volatile unsigned long lastIrTime = 0;
const unsigned long DEBOUNCE_US = 5000;
float windKmh = 0.0;

unsigned long lastWindCalc = 0;
int pulsesPerMin = 0;  

// data variables (keep last good values)
int pm25 = 0, pm10 = 0;
float temperature = 0.0f;
float pressure = 0.0f;
float humidity = 0.0f;
bool sensorSeen = false;
unsigned long lastSend = 0;
unsigned long lastValid = 0;

// non‑blocking sensor read timer
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 2000;

// for recovery after long sensor loss
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
// Wind speed calculation
// ------------------------------------------------------------------
void updatePulsesPerMin() {
  unsigned long now = millis();
  unsigned long dt = now - lastWindCalc;
  if (dt >= 3000) {
    float pulsesPerSec = (float)irPulseCount / (dt / 1000.0);
    pulsesPerMin = (int)(pulsesPerSec * 60.0);
    windKmh = pulsesPerMin * 0.01;   // conversion factor
    Serial.print("DATA: Wind - pulses in last ");
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
// PMS5003 reader (unchanged, works)
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

  Serial.print("DATA: Air quality - PM2.5: "); Serial.print(pm25);
  Serial.print(", PM10 : "); Serial.println(pm10);
  return true;
}

// ------------------------------------------------------------------
// Read BMP280 (temperature & pressure) + AHT20 (humidity)
// ------------------------------------------------------------------
bool readEnvironment() {
  bool any_success = false;

  // ----- BMP280 -----
  if (bmp_ok) {
    float t = bmp.readTemperature();
    float p = bmp.readPressure() / 100.0F;
    if (!isnan(t) && !isnan(p)) {
      temperature = t;
      pressure = p;
      any_success = true;
      Serial.print("DATA: BMP280 - Temp: "); Serial.print(temperature);
      Serial.print(" °C, Pressure: "); Serial.print(pressure); Serial.println(" hPa");
    } else {
      Serial.println("WARN: BMP280 read failed (invalid values)");
    }
  } else {
    Serial.println("WARN: BMP280 not initialised, skipping read");
  }

  // ----- AHT20 -----
  if (aht_ok) {
    sensors_event_t humidity_event, temp_event;
    if (aht.getEvent(&humidity_event, &temp_event)) {
      humidity = humidity_event.relative_humidity;
      // optionally use temp_event.temperature if BMP280 is absent
      any_success = true;
      Serial.print("DATA: AHT20 - Humidity: "); Serial.print(humidity); Serial.println(" %");
    } else {
      Serial.println("WARN: AHT20 read failed");
    }
  } else {
    Serial.println("WARN: AHT20 not initialised, skipping read");
  }

  return any_success;
}

// ------------------------------------------------------------------
// Re‑initialise all sensors (I2C + PMS)
// ------------------------------------------------------------------
void reinitSensors() {
  Serial.println("REINIT: Restarting sensors.");
  
  // Reset I2C
  Wire.end();
  delay(100);
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);  // safe speed
  
  // Re‑init BMP280 (try both addresses)
  bmp_ok = false;
  if (bmp.begin(0x76)) {
    bmp_ok = true;
    Serial.println("REINIT: BMP280 found at 0x76");
  } else if (bmp.begin(0x77)) {
    bmp_ok = true;
    Serial.println("REINIT: BMP280 found at 0x77");
  } else {
    Serial.println("REINIT: BMP280 not found");
  }
  
  // Re‑init AHT20
  aht_ok = aht.begin();
  if (aht_ok) Serial.println("REINIT: AHT20 OK");
  else Serial.println("REINIT: AHT20 not found");
  
  // Re‑init PMS UART
  pms.end();
  delay(100);
  pms.begin(9600, SERIAL_8N1, rxPin);
  Serial.println("REINIT: PMS5003 reinit attempt");
  
  sensorsNeedReinit = false;
}

// ------------------------------------------------------------------
// WiFi reconnect with full reset
// ------------------------------------------------------------------
void WiFiReconnect() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("WIFI: Connection lost, attempting full reconnect...");
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
// HTTP POST (now includes humidity)
// ------------------------------------------------------------------
void sendData(int pm25, int pm10, float temp, float pres, float hum, int pulses, float wind) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR: WiFi not connected");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH);
      delay(250);
      digitalWrite(GREEN_LED, LOW);
      delay(250);
    }
    return;
  }

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("ERR: Server connection failed");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH);
      delay(250);
      digitalWrite(GREEN_LED, LOW);
      delay(250);
    }
    return;
  }

  char payload[512];
  snprintf(payload, sizeof(payload),
           "{\"pm25\":%d,\"pm10\":%d,\"temp\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f,\"pulses_per_min\":%d,\"wind_kmh\":%.2f,\"ts\":%lu}",
           pm25, pm10, temp, pres, hum, pulses, wind, millis());

  char httpBuffer[512];
  int len = snprintf(httpBuffer, sizeof(httpBuffer),
                     "POST %s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Content-Type: application/json\r\n"
                     "Content-Length: %d\r\n"
                     "\r\n"
                     "%s",
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
    digitalWrite(GREEN_LED, HIGH);
    delay(750);
    digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("ERR: HTTP Error!");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH);
      delay(250);
      digitalWrite(GREEN_LED, LOW);
      delay(250);
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

  // Wind sensor interrupt
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irISR, FALLING);
  lastWindCalc = millis();
  Serial.println("INFO: Wind sensor ready");

  // PMS5003
  pms.begin(9600, SERIAL_8N1, rxPin);
  Serial.println("INFO: PMS5003 initialised");

  // I2C sensors
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);
  
  // Try both BMP280 addresses
  if (bmp.begin(0x76)) {
    bmp_ok = true;
    Serial.println("INFO: BMP280 found at 0x76");
  } else if (bmp.begin(0x77)) {
    bmp_ok = true;
    Serial.println("INFO: BMP280 found at 0x77");
  } else {
    Serial.println("ERR: BMP280 not detected");
  }
  
  aht_ok = aht.begin();
  if (aht_ok) Serial.println("INFO: AHT20 found");
  else Serial.println("ERR: AHT20 not detected");

  // WiFi
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.print("INFO: Connecting to ");
  Serial.print(ssid);
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
    bool env_ok = readEnvironment();  // BMP + AHT

    if (pms_ok || env_ok) {
      sensorSeen = true;
      lastValid = millis();
      lastAnySensorOk = millis();
      sensorsNeedReinit = false;
    } else {
      if (millis() - lastValid > 30000) {
        sensorSeen = false;
        Serial.println("ERR: No sensor data for 30s");
      }
      if (millis() - lastAnySensorOk > 60000 && !sensorsNeedReinit) {
        sensorsNeedReinit = true;
        Serial.println("ERR: 60s with no data, will reinit sensors");
      }
    }
  }

  // Reinit if needed
  if (sensorsNeedReinit) {
    reinitSensors();
    delay(500);
  }

  // WiFi recovery
  if (WiFi.status() != WL_CONNECTED) {
    WiFiReconnect();
  }

  // Send data every 10 seconds
  if (millis() - lastSend >= 10000) {
    if (sensorSeen) {
      sendData(pm25, pm10, temperature, pressure, humidity, pulsesPerMin, windKmh);
    } else {
      sendData(0, 0, 0.0, 0.0, 0.0, 0, 0.0);
    }
    lastSend = millis();
  }
}
