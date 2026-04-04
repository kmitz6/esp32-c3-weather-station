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
HardwareSerial pms(0);
const int rxPin = 20;  // PMS TX -> ESP RX

// BMP280 temperature and pressure sensor
Adafruit_BMP280 bmp;
const int sdaPin = 6;
const int sclPin = 7;

// transmit LED
const int GREEN_LED = 2;  // blinks on HTTP 200 OK

// anemometer
const int IR_PIN = 8;
volatile int irPulseCount = 0;
volatile unsigned long lastIrTime = 0;
const unsigned long DEBOUNCE_US = 5000;  // to be adjusted if sensor returns more than one pulse per window or if loses a beat
float windKmh = 0.0;

unsigned long lastWindCalc = 0;
int pulsesPerMin = 0;  

// data variables
int pm25 = 0, pm10 = 0;
float temperature = 0.0f, pressure = 0.0f;
bool sensorSeen = false;
unsigned long lastSend = 0;
unsigned long lastValid = 0;
char httpBuffer[300];

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

// ---------------- HTTP ----------------
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

  
  char payload[180];  // larger buffer
  snprintf(payload, sizeof(payload),
           "{\"pm25\":%d,\"pm10\":%d,\"temp\":%.2f,\"pressure\":%.2f,\"pulses_per_min\":%d,\"wind_kmh\":%.2f,\"ts\":%lu}",
           pm25, pm10, temp, pres, pulsesPerMin, windKmh, millis());
// due to some data conversion errors I rolled back to manually writing a call
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

  while (millis() - timeout < 2000) {
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

// ---------------- PMS READ ----------------
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

// ---------------- BMP READ ----------------
bool readBMP() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;

  if (isnan(temperature) || isnan(pressure)) return false;

  Serial.print("DATA: BMP280 - Temp: "); Serial.print(temperature);
  Serial.print(", Pressure: "); Serial.println(pressure);

  return true;
}

// ---------------- SETUP ----------------
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
}

// ---------------- LOOP ----------------
void loop() {
  Serial.println("INFO: loop() start");

  updatePulsesPerMin();   // calculate pulses per minute every N sec

  bool pms_ok = readPMS();
  bool bmp_ok = readBMP();

  if (pms_ok || bmp_ok) {
    sensorSeen = true;
    lastValid = millis();
  } else {
    if (millis() - lastValid > 30000) {
      sensorSeen = false;
      Serial.println("ERR: No sensor data detected.");
      delay(2000);
    }
  }

  if (millis() - lastSend >= 10000) {
    if (sensorSeen) {
      sendData(pm25, pm10, temperature, pressure, pulsesPerMin, windKmh);
    } else {
      sendData(0, 0, 0.0, 0.0, 0, 0.0);
    }
    lastSend = millis();
  }

  delay(2000);
}
