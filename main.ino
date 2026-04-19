#include "config.h"

bool sensorSeen = false;
unsigned long lastSend = 0;
unsigned long lastValid = 0;
unsigned long lastSensorRead = 0;
unsigned long lastAnySensorOk = 0;
unsigned long lastWifiAttempt = 0;
bool sensorsNeedReinit = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  Serial.println("INFO: Booting weather station");

  pinMode(IR_PIN, INPUT_PULLUP);
  gpio_set_pin_filter(IR_PIN, 1023);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irISR, FALLING);
  lastWindCalc = millis();

  pms.begin(9600, SERIAL_8N1, PMS_RX_PIN);
  Wire.begin(SDA_PIN, SCL_PIN);
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

void loop() {
  updatePulsesPerMin();

  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();

    int raw_pm25 = -1, raw_pm10 = -1;
    float raw_temp = NAN, raw_press = NAN, raw_hum = NAN;

    bool pms_ok = readPMS(raw_pm25, raw_pm10);
    bool env_ok = readEnvironment(raw_temp, raw_press, raw_hum);

    if (pms_ok || env_ok) {
      sensorSeen = true;
      lastValid = millis();
      lastAnySensorOk = millis();
      sensorsNeedReinit = false;
      storeToBuffer(raw_temp, raw_press, raw_hum, windKmh,
                    pms_ok ? raw_pm25 : -1, pms_ok ? raw_pm10 : -1);
      printAggregates();
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
    clearBuffer();
    delay(500);
    sensorsNeedReinit = false;
  }

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiAttempt >= WIFI_RETRY_INTERVAL) {
    lastWifiAttempt = millis();
    WiFiReconnect();
  }

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
