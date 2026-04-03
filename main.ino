#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// ---------------- WIFI ----------------
const char* ssid     = "";
const char* password = "";
const char* host     = "";
const int   port     = 5000;
const char* path     = "/api/sensors";

// ---------------- PMS5003 ----------------
HardwareSerial pms(0);
const int rxPin = 20;  // PMS TX -> ESP RX
const int txPin = 21;  // PMS RX -> ESP TX

// ---------------- BMP280 ----------------
Adafruit_BMP280 bmp;
const int sdaPin = 6;
const int sclPin = 7;

// ---------------- LEDs ----------------
const int GREEN_LED = 2;  // blinks on HTTP 200 OK
const int RED_LED   = 10; // stays ON on error

// ---------------- IR SWITCH (Wind Speed) with debounce ----------------
const int IR_PIN = 8;
volatile int irPulseCount = 0;
volatile unsigned long lastIrTime = 0;
const unsigned long DEBOUNCE_US = 5000;   // 5 ms debounce (adjust if needed)

unsigned long lastWindCalc = 0;      // last time wind speed was calculated
int pulsesPerMin = 0;                // computed pulses per minute

// ---------------- DATA ----------------
int pm25 = 0, pm10 = 0;
float temperature = 0.0f, pressure = 0.0f;
bool sensorSeen = false;
unsigned long lastSend = 0;
unsigned long lastValid = 0;
char httpBuffer[300];

// ---------------- IR INTERRUPT SERVICE ROUTINE (debounced) ----------------
void IRAM_ATTR irISR() {
  unsigned long now = micros();
  if (now - lastIrTime > DEBOUNCE_US) {
    irPulseCount++;
    lastIrTime = now;
  }
}

// ---------------- PULSES PER MINUTE CALCULATION ----------------
void updatePulsesPerMin() {
  unsigned long now = millis();
  unsigned long dt = now - lastWindCalc;
  if (dt >= 3000) {   // calculate every 3 seconds
    float pulsesPerSec = (float)irPulseCount / (dt / 1000.0);
    pulsesPerMin = (int)(pulsesPerSec * 60.0);
    
    Serial.print("[Wind] Pulses in last ");
    Serial.print(dt);
    Serial.print(" ms: ");
    Serial.print(irPulseCount);
    Serial.print(" -> ");
    Serial.print(pulsesPerMin);
    Serial.println(" pulses/min");
    
    irPulseCount = 0;
    lastWindCalc = now;
  }
}

// ---------------- HTTP ----------------
void sendData(int pm25, int pm10, float temp, float pres, int pulsesPerMin) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping POST");
    digitalWrite(RED_LED, HIGH);
    return;
  }

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("HTTP connect failed");
    digitalWrite(RED_LED, HIGH);
    return;
  }

  char payload[160];
  snprintf(payload, sizeof(payload),
           "{\"pm25\":%d,\"pm10\":%d,\"temp\":%.2f,\"pressure\":%.2f,\"pulses_per_min\":%d,\"ts\":%lu}",
           pm25, pm10, temp, pres, pulsesPerMin, millis());

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
      Serial.println(line);
      if (line.startsWith("HTTP/1.1 200") || line.startsWith("HTTP/1.0 200")) {
        gotHTTP200 = true;
      }
    }
  }

  client.stop();

  if (gotHTTP200) {
    Serial.println("Data sent successfully (HTTP 200)");
    digitalWrite(GREEN_LED, HIGH);
    delay(200);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
  } else {
    Serial.println("HTTP Error!");
    digitalWrite(RED_LED, HIGH);
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

  Serial.print("PM2.5: "); Serial.println(pm25);
  Serial.print("PM10 : "); Serial.println(pm10);

  return true;
}

// ---------------- BMP READ ----------------
bool readBMP() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;

  if (isnan(temperature) || isnan(pressure)) return false;

  Serial.print("Temp: "); Serial.println(temperature);
  Serial.print("Pressure: "); Serial.println(pressure);

  return true;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  Serial.println("=== ESP32-C3 Boot ===");

  // IR sensor with debounce
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irISR, FALLING);
  Serial.print("IR sensor on GPIO ");
  Serial.print(IR_PIN);
  Serial.println(" configured with falling edge interrupt and 5ms debounce");
  lastWindCalc = millis();

  pms.begin(9600, SERIAL_8N1, rxPin, txPin);
  Serial.println("PMS UART started");

  Wire.begin(sdaPin, sclPin);
  if (!bmp.begin(0x76)) Serial.println("BMP280 not found!");
  else Serial.println("BMP280 OK");

  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\nWiFi connected");
  else Serial.println("\nWiFi timeout");

  lastValid = millis();
}

// ---------------- LOOP ----------------
void loop() {
  Serial.println("\n--- Loop Start ---");

  updatePulsesPerMin();   // calculate pulses per minute every 3 sec

  bool pms_ok = readPMS();
  bool bmp_ok = readBMP();

  if (pms_ok || bmp_ok) {
    sensorSeen = true;
    lastValid = millis();
  } else {
    if (millis() - lastValid > 30000) {
      sensorSeen = false;
      Serial.println("No sensor data detected");
    }
  }

  if (millis() - lastSend >= 10000) {
    if (sensorSeen) {
      sendData(pm25, pm10, temperature, pressure, pulsesPerMin);
    } else {
      sendData(0, 0, 0.0, 0.0, pulsesPerMin);
    }
    lastSend = millis();
  }

  delay(2000);
}
