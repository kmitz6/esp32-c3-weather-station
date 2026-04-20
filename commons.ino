#include "config.h"

float tempBuffer[BUFFER_SIZE];
float pressBuffer[BUFFER_SIZE];
float humBuffer[BUFFER_SIZE];
float windBuffer[BUFFER_SIZE];
int pm25Buffer[BUFFER_SIZE];
int pm10Buffer[BUFFER_SIZE];


void storeToBuffer(float temp, float press, float hum, float wind, int pm25, int pm10) {
  tempBuffer[bufferIndex] = temp;
  pressBuffer[bufferIndex] = press;
  humBuffer[bufferIndex] = hum;
  windBuffer[bufferIndex] = wind;
  pm25Buffer[bufferIndex] = pm25;
  pm10Buffer[bufferIndex] = pm10;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) bufferCount++;
}

void clearBuffer() {
  bufferIndex = 0;
  bufferCount = 0;
}

void computeRawAggregates(float &avgTemp, float &avgPress, float &avgHum,
                          float &maxWind, int &maxPM25, int &maxPM10) {
  if (bufferCount == 0) {
    avgTemp = avgPress = avgHum = 0.0f;
    maxWind = 0.0f;
    maxPM25 = maxPM10 = 0;
    return;
  }
  float sumTemp = 0, sumPress = 0, sumHum = 0;
  int tempCount = 0, pressCount = 0, humCount = 0;
  float maxW = -1e6;
  int maxP25 = -1, maxP10 = -1;
  for (int i = 0; i < bufferCount; i++) {
    if (!isnan(tempBuffer[i])) { sumTemp += tempBuffer[i]; tempCount++; }
    if (!isnan(pressBuffer[i])) { sumPress += pressBuffer[i]; pressCount++; }
    if (!isnan(humBuffer[i])) { sumHum += humBuffer[i]; humCount++; }
    if (windBuffer[i] > maxW) maxW = windBuffer[i];
    if (pm25Buffer[i] > maxP25) maxP25 = pm25Buffer[i];
    if (pm10Buffer[i] > maxP10) maxP10 = pm10Buffer[i];
  }
  avgTemp = (tempCount > 0) ? sumTemp / tempCount : 0.0f;
  avgPress = (pressCount > 0) ? sumPress / pressCount : 0.0f;
  avgHum = (humCount > 0) ? sumHum / humCount : 0.0f;
  maxWind = maxW;
  maxPM25 = maxP25;
  maxPM10 = maxP10;
}

void printAggregates() {
  float avgT, avgP, avgH, maxW;
  int maxP25, maxP10;
  computeRawAggregates(avgT, avgP, avgH, maxW, maxP25, maxP10);
  Serial.print("--- Aggregates (raw, last ");
  Serial.print(bufferCount);
  Serial.println(" readings) ---");
  Serial.print("Avg Temp: "); Serial.print(avgT); Serial.print(" °C, ");
  Serial.print("Avg Press: "); Serial.print(avgP); Serial.print(" hPa, ");
  Serial.print("Avg Hum: "); Serial.print(avgH); Serial.println(" %");
  Serial.print("Max Wind: "); Serial.print(maxW); Serial.print(" km/h, ");
  Serial.print("Max PM2.5: "); Serial.print(maxP25); Serial.print(", ");
  Serial.print("Max PM10: "); Serial.println(maxP10);
  Serial.println("----------------------------------------");
}

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

void sendData(float rawAvgTemp, float rawAvgPress, float rawAvgHum,
              float rawMaxWind, int rawMaxPM25, int rawMaxPM10) {
  float calibTemp = rawAvgTemp + TEMP_CALIB;
  float calibPress = rawAvgPress + PRESS_CALIB;
  float calibHum = rawAvgHum + HUM_CALIB;
  float calibWind = rawMaxWind * WIND_CALIB;
  int calibPM25 = rawMaxPM25 * PM25_CALIB;
  int calibPM10 = rawMaxPM10 * PM10_CALIB;

  Serial.println("--- Sending to server ---");
  Serial.print("Calibrated Temp: "); Serial.print(calibTemp);
  Serial.print(" °C, Press: "); Serial.print(calibPress);
  Serial.print(" hPa, Hum: "); Serial.print(calibHum);
  Serial.print(" %, Wind: "); Serial.print(calibWind);
  Serial.print(" km/h, PM2.5: "); Serial.print(calibPM25);
  Serial.print(", PM10: "); Serial.println(calibPM10);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR: WiFi not connected");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH); delay(100);
      digitalWrite(GREEN_LED, LOW); delay(100);
    }
    return;
  }
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("ERR: Server connection failed");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH); delay(100);
      digitalWrite(GREEN_LED, LOW); delay(100);
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
    digitalWrite(GREEN_LED, HIGH); delay(100); digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("ERR: HTTP Error!");
    for(int i = 0; i < 3; i++) {
      digitalWrite(GREEN_LED, HIGH); delay(100);
      digitalWrite(GREEN_LED, LOW); delay(100);
    }
  }
}
