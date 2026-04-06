#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Network creds
const char* ssid     = "";
const char* password = "";
const char* host     = "";
const int   port     = 5000;
const char* path     = "/api/sensors";

// PMS5003 air quality sensor
HardwareSerial pms(1);
const int rxPin = 20;  // PMS TX -> ESP RX

// BMP280 temperature and pressure sensor
Adafruit_BMP280 bmp;
const int sdaPin = 6;
const int sclPin = 7;

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

// data variables
int pm25 = 0, pm10 = 0;
float temperature = 0.0f, pressure = 0.0f;
bool sensorSeen = false;
unsigned long lastSend = 0;
unsigned long lastValid = 0;
char httpBuffer[512];

// non-blocking sensor read timer
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 2000;

// for recovery after long sensor loss
unsigned long lastAnySensorOk = 0;
bool sensorsNeedReinit = false;

// interrupt routine for the ir transoptor
void IRAM_ATTR irISR() {
  unsigned long now = micros();
  if (now - lastIrTime > DEBOUNCE_US) {
    irPulseCount++;
    lastIrTime = now;
  }
}

// wind speed calculation
void updatePulsesPerMin() {
  unsigned long now = millis();
  unsigned long dt = now - lastWindCalc;
  if (dt >= 1000) {
    float pulsesPerSec = (float)irPulseCount / (dt / 1000.0);
    pulsesPerMin = (int)(pulsesPerSec * 60.0);
    windKmh = pulsesPerMin * 0.01;   // conversion factor
    Serial.print("DATA: Wind - pulses in last ");
    Serial.print(dt);
    Serial.print(" ms: ");
    Serial.print(irPulseCount);
    Serial.print(" -> ");
    Serial.print(pulsesPerMin);
    Serial.print(" pulses/min ");
    Serial.print("(~");
    Serial.print(windKmh);
    Serial.println(" kmh)");
    
    irPulseCount = 0;
    lastWindCalc = now;
  }
}

void WiFiConnnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.println("INFO: WiFi lost, attempting full reconnect...");

  // Full WiFi stack reset - this is the key fix for the "cannot set config" error
  WiFi.disconnect(true, true);   // disconnect + erase old config
  delay(100);

  WiFi.mode(WIFI_OFF);           // completely power down the radio
  delay(200);

  WiFi.mode(WIFI_STA);           // restart in station mode
  delay(100);

  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {  // 25s timeout
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nINFO: WiFi reconnected successfully");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nERR: WiFi reconnect timeout");
  }
}

// HTTP POST
void sendData(int pm25, int pm10, float temp, float pres, int pulsesPerMin, float windKmh) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR: WiFi connection failed");
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

  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"pm25\":%d,\"pm10\":%d,\"temp\":%.2f,\"pressure\":%.2f,\"pulses_per_min\":%d,\"wind_kmh\":%.2f,\"ts\":%lu}",
           pm25, pm10, temp, pres, pulsesPerMin, windKmh, millis());

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

  // increased timeout to 5 seconds
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
    Serial.println("INFO: Data sent successfully (HTTP 200)");
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

// PMS5003 data read
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

// BMP280 READ
bool readBMP() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;

  if (isnan(temperature) || isnan(pressure)) return false;

  Serial.print("DATA: BMP280 - Temp: "); Serial.print(temperature);
  Serial.print(", Pressure: "); Serial.println(pressure);

  return true;
}

// SENSOR REINIT
void reinitSensors() {
  Serial.println("INFO: Re-initializing sensors...");
  // Reinit I2C
  Wire.end();
  delay(100);
  Wire.begin(sdaPin, sclPin);
  if (!bmp.begin(0x76)) {
    Serial.println("ERR: BMP280 reinit failed");
  } else {
    Serial.println("INFO: BMP280 reinit OK");
  }
  // Reinit UART for PMS
  pms.end();
  delay(100);
  pms.begin(9600, SERIAL_8N1, rxPin);
  Serial.println("INFO: PMS5003 reinit attempted");
  sensorsNeedReinit = false;
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);

  Serial.println("INFO: Booting weather station");

  // IR sensor with debounce
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irISR, FALLING);
  lastWindCalc = millis();
  Serial.println("INFO: Transoptical sensor initiated - windspeed");
  delay(1000);

  pms.begin(9600, SERIAL_8N1, rxPin);
  Serial.println("INFO: PMS5003 initialized");
  delay(1000);

  Wire.begin(sdaPin, sclPin);
  if (!bmp.begin(0x76)) Serial.println("ERR: Temp/pressure sensor no signal");
  else Serial.println("INFO: BMP280 Temp/pressure sensor initiated");

  Serial.print("INFO: Connecting WiFi");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\nWiFi connected");
  else Serial.println("ERR: WiFi timeout");
  delay(2000);

  lastValid = millis();
  lastAnySensorOk = millis();
  lastSensorRead = millis();
}

// LOOP
void loop() {
  // 1. Update wind speed (always, non-blocking)
  updatePulsesPerMin();

  // 2. Read sensors every SENSOR_READ_INTERVAL ms (non-blocking)
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();

    bool pms_ok = readPMS();
    bool bmp_ok = readBMP();

    if (pms_ok || bmp_ok) {
      sensorSeen = true;
      lastValid = millis();
      lastAnySensorOk = millis();
      sensorsNeedReinit = false;  // at least one sensor works
    } else {
      if (millis() - lastValid > 30000) {
        sensorSeen = false;
        Serial.println("ERR: No sensor data detected.");
      }
      // If no sensor data for 60 seconds, try to reinit
      if (millis() - lastAnySensorOk > 60000 && !sensorsNeedReinit) {
        sensorsNeedReinit = true;
        Serial.println("ERR: Both sensors failed for 60s, will reinit");
      }
    }
  }

  // 3. Reinit sensors if needed (only once, then cleared on success)
  if (sensorsNeedReinit) {
    reinitSensors();
    delay(500);
  }

  // 4. Check WiFi and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    WiFiConnect();
  }

  // 5. Send data every 10 seconds
  if (millis() - lastSend >= 10000) {
    if (sensorSeen) {
      sendData(pm25, pm10, temperature, pressure, pulsesPerMin, windKmh);
    } else {
      sendData(0, 0, 0.0, 0.0, 0, 0.0);
    }
    lastSend = millis();
  }
}
