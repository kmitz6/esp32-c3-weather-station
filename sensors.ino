#include "config.h"

HardwareSerial pms(1);
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;

bool bmp_ok = false;
bool aht_ok = false;

volatile unsigned long irPulseCount = 0;
volatile unsigned long lastIrTime = 0;
float windKmh = 0.0;
unsigned long lastWindCalc = 0;

void IRAM_ATTR irISR() {
  unsigned long now = micros();
  if (now - lastIrTime > DEBOUNCE_US) {
    irPulseCount++;
    lastIrTime = now;
  }
}

void updatePulsesPerMin() {
  unsigned long now = millis();
  unsigned long dt = now - lastWindCalc;
  if (dt >= 3000) {
    float pulsesPerSec = (float)irPulseCount / (dt / 1000.0);
    int pulsesPerMin = (int)(pulsesPerSec * 60.0);
    windKmh = pulsesPerMin * 0.012; // 0.012 is an arbitrary value based on another calibrated sensor readings
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

bool readPMS(int &pm25, int &pm10) {
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
  Serial.print("RAW: Air quality - PM2.5: ");
  Serial.print(pm25);
  Serial.print(", PM10: ");
  Serial.println(pm10);
  return true;
}

bool readEnvironment(float &temp, float &pres, float &hum) {
  bool any = false;
  if (bmp_ok) {
    float t = bmp.readTemperature();
    float p = bmp.readPressure() / 100.0F;
    if (!isnan(t) && !isnan(p)) {
      temp = t;
      pres = p;
      any = true;
      Serial.print("RAW: BMP280 - Temp: ");
      Serial.print(temp);
      Serial.print(" °C, Pressure: ");
      Serial.print(pres);
      Serial.println(" hPa");
    } else {
      Serial.println("WARN: BMP280 read failed");
    }
  }
  if (aht_ok) {
    sensors_event_t humidity_event, temp_event;
    if (aht.getEvent(&humidity_event, &temp_event)) {
      hum = humidity_event.relative_humidity;
      any = true;
      Serial.print("RAW: AHT20 - Humidity: ");
      Serial.print(hum);
      Serial.println(" %");
    } else {
      Serial.println("WARN: AHT20 read failed");
    }
  }
  return any;
}

void reinitSensors() {
  Serial.println("REINIT: Restarting sensors.");
  Wire.end(); delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  bmp_ok = false;
  if (bmp.begin(0x76)) bmp_ok = true;
  else if (bmp.begin(0x77)) bmp_ok = true;
  else Serial.println("REINIT: BMP280 not found");
  aht_ok = aht.begin();
  if (aht_ok) Serial.println("REINIT: AHT20 OK");
  else Serial.println("REINIT: AHT20 not found");
  pms.end(); delay(100);
  pms.begin(9600, SERIAL_8N1, PMS_RX_PIN);
}
