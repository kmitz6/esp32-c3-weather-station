# ESP32 Weather Station with PMS5003, BMP280, AHT20, and Anemometer

3D model available @ cults:



This project implements a weather data logger for an ESP32 board. It reads data from:

- **PMS5003** laser particle counter (PM2.5, PM10)
- **BMP280** temperature and pressure sensor
- **AHT20** humidity sensor
- **Anemometer** (IR interrupt‑based, e.g. a cup anemometer with a reed switch or hall sensor)

Readings are taken every 3 seconds, stored in a rolling 15‑sample buffer, and every 30 seconds the **average** of temperature, pressure, humidity together with the **maximum** of wind speed, PM2.5 and PM10 over the buffer are calculated. After applying user‑defined calibration offsets, the data is sent to a remote server via HTTP POST.

The firmware includes automatic WiFi reconnection and sensor re‑initialisation if no valid data is received for 60 seconds.

---

## Hardware Requirements

- ESP32 development board (e.g. ESP32‑WROOM‑32)
- PMS5003 particulate matter sensor
- BMP280 pressure/temperature sensor (I²C)
- AHT20 humidity/temperature sensor (I²C)
- Anemometer (pulse output, normally high, falling edge interrupt)
- Green LED (optional, for status indication)
- Appropriate power supply (5V for ESP32, PMS5003)

### Pin Connections

| Component      | ESP32 Pin  | Notes                                      |
|----------------|------------|--------------------------------------------|
| PMS5003 TX     | 20 (RX1)   | Use UART1 (Serial1)                        |
| I²C SDA        | 6          | Connect to BMP280 & AHT20 SDA              |
| I²C SCL        | 7          | Connect to BMP280 & AHT20 SCL              |
| Anemometer OUT | 8          | Internal pull‑up, falling edge interrupt   |
| Green LED      | 2          | Active high, indicates data send / errors  |

> **Note:** The code assumes the PMS5003 is connected to `HardwareSerial(1)` with RX on pin 20.  
> I²C address for BMP280 is automatically detected (0x76 or 0x77).  
> The anemometer should produce a low pulse (e.g. 5 ms) per revolution – the debounce time is set to 5000 µs.

---

## Software Dependencies

Install the following libraries via the Arduino Library Manager or manually:

- [`WiFi.h`](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi) – built‑in with ESP32 core
- [`Wire.h`](https://github.com/espressif/arduino-esp32/tree/master/libraries/Wire) – built‑in
- [`Adafruit_BMP280`](https://github.com/adafruit/Adafruit_BMP280_Library)
- [`Adafruit_AHTX0`](https://github.com/adafruit/Adafruit_AHTX0)

The PMS5003 is read directly via serial – no external library is required.

---

## Configuration

Edit the following defines at the top of the sketch **before uploading**:

### WiFi & Server
```cpp
const char* ssid     = "your_wifi_ssid";
const char* password = "your_wifi_password";
const char* host     = "your.server.com";   // IP address or domain name
const int   port     = 5000;                // Server port
const char* path     = "/api/sensors";      // HTTP endpoint
```

### Calibration Offsets
```cpp
float TEMP_CALIB     = -4.0;   // added to raw average temperature (°C)
float PRESS_CALIB    = 15.0;   // added to raw average pressure (hPa)
float HUM_CALIB      = 0.0;    // added to raw average humidity (%)
float WIND_CALIB     = 1.0;    // multiplied with max wind speed
float PM25_CALIB     = 1.0;    // multiplied with max PM2.5
float PM10_CALIB     = 1.0;    // multiplied with max PM10
```

### Timing (optional)
```cpp
const unsigned long SENSOR_READ_INTERVAL = 3000;   // milliseconds between sensor reads
const unsigned long SEND_INTERVAL        = 30000;  // milliseconds between HTTP POSTs
```

### Buffer Size
```cpp
const int BUFFER_SIZE = 15;   // number of readings used for each aggregate
```

---

## How It Works

1. **Initialisation (setup)**  
   - Serial, I²C, LED, anemometer interrupt and PMS5003 serial port are initialised.  
   - BMP280 and AHT20 are probed.  
   - WiFi connection is established (with a 30 s timeout).

2. **Main Loop**  
   - Every 3 seconds:  
     - Read PMS5003 (checksum‑validated).  
     - Read BMP280 and AHT20 (if present).  
     - If **any** sensor provides valid data, the values are stored into the rolling buffer.  
     - The rolling buffer keeps the last 15 readings (or fewer during startup).  
   - The anemometer pulse count is integrated continuously; wind speed (km/h) is calculated every 3 seconds using the formula:  
     `wind_kmh = (pulses_per_minute) * 0.04`.  
     *This factor assumes 1 pulse/min = 0.04 km/h – adjust according to your anemometer’s specification.*

3. **Data Aggregation & Transmission**  
   - Every 30 seconds the sketch computes:  
     - **Average** of temperature, pressure, humidity over the buffer.  
     - **Maximum** of wind speed, PM2.5, PM10 over the buffer.  
   - Calibration offsets are applied to these raw aggregates.  
   - A JSON payload is built and sent via HTTP POST to `http://host:port/path`.  
   - The green LED blinks once for success, three times for failure.

4. **Fault Recovery**  
   - If no sensor provides valid data for 60 seconds, the I²C bus and PMS5003 are re‑initialised.  
   - WiFi is automatically reconnected if the connection is lost.

---

## Serial Output

The firmware prints detailed information to the Serial Monitor (115200 baud):

- Raw instantaneous readings (every 3 s)
- Buffer aggregates (every 3 s)
- Calibrated data sent to the server (every 30 s)
- Warnings and errors (e.g. sensor read failures, WiFi problems)

Example:
```
RAW: Air quality - PM2.5: 12, PM10: 18
RAW: BMP280 - Temp: 22.45 °C, Pressure: 1013.20 hPa
RAW: AHT20 - Humidity: 54.20 %
--- Aggregates (raw, last 12 readings) ---
Avg Temp: 22.51 °C, Avg Press: 1013.15 hPa, Avg Hum: 54.10 %
Max Wind: 2.34 km/h, Max PM2.5: 15, Max PM10: 22
----------------------------------------
--- Sending to server ---
Calibrated Temp: 18.51 °C, Press: 1028.15 hPa, Hum: 54.10 %, Wind: 2.34 km/h, PM2.5: 15, PM10: 22
INFO: Data sent successfully
```

---

## LED Behaviour

| LED Pattern                 | Meaning                              |
|-----------------------------|--------------------------------------|
| Short blink (750 ms) once   | Data successfully sent to server     |
| Three fast blinks (250 ms)  | WiFi not connected or server error   |
| (No blink)                  | Normal operation                     |

---

## Notes & Customisation

### Wind Speed Calibration
The factor `0.04` in `updatePulsesPerMin()` assumes that one pulse per minute corresponds to 0.04 km/h.  
If your anemometer has a different specification (e.g. 1 pulse/min = 0.1 m/s), change the formula accordingly.  
You can also use `WIND_CALIB` as a global multiplier.

### I²C Address Conflicts
The code tries BMP280 at 0x76 first, then 0x77. If you use a different sensor (e.g. BME280), the library remains compatible but humidity will come from the AHT20 only.

### PMS5003
The PMS5003 is read at 9600 baud. The code skips any bytes until it finds the start character `0x42`. A full 32‑byte frame is read and checksum‑validated.

### Data Payload Format
The HTTP POST body is a JSON object:
```json
{
  "pm25": 15,
  "pm10": 22,
  "temp": 18.5,
  "pressure": 1028.2,
  "humidity": 54.1,
  "wind_kmh": 2.34,
  "ts": 123456789
}
```
`ts` is the uptime in milliseconds (can be ignored or replaced with a real timestamp).

---

## Troubleshooting

- **No I²C sensors detected** – Check wiring, pull‑up resistors (typically 4.7 kΩ on SDA/SCL). Try scanning I²C addresses with a simple sketch.
- **PMS5003 not sending data** – Ensure the sensor is powered (5V) and the TX pin is connected to ESP32 RX (pin 20). Ground must be common.
- **Anemometer gives erratic readings** – Adjust `DEBOUNCE_US` (currently 5000 µs) to match your sensor’s pulse width.  
- **WiFi does not reconnect** – The code forces a full disconnection and re‑initialisation of the WiFi stack. If problems persist, check your router’s client limit or signal strength.
- **Server returns error** – Verify that the server endpoint expects exactly the JSON format shown above. The code checks for `HTTP/1.1 200` or `HTTP/1.0 200`; any other response is treated as an error.

---

## License

no license
