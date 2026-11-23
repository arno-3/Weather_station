#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>
#include <HTTPClient.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <ArduinoOTA.h>

// ---------- WiFi ----------
const char* ssid = "dlink-6D5C";
const char* password = "cuwqs53732";

WebServer server(80);

// ---------- I2C ----------
#define I2C_ADDR 0x08

// Tester pin
const int ledPin = 14;

// ---------- Wind sensor agreement thresholds ----------
const float WIND_SPEED_TOLERANCE = 2.0;   // m/s
const float WIND_DIR_TOLERANCE   = 30.0;  // degrees

// ---------- Weather Underground ----------
const char* WU_STATION_ID  = "IPOTCH10";
const char* WU_STATION_KEY = "fHQKUbHD";
const unsigned long wuUploadInterval = 60000;
unsigned long lastWuUpload = 0;

// ---------- Daily rain reset ----------
const byte CMD_RESET_RAIN = 0xAA; // must match the Nano's value exactly
char lastRainResetDate[11] = "";  // "YYYY-MM-DD"

// ---------- Rain (WU only, not exposed on /weather) ----------
const float RAIN_MM_TO_IN = 0.0393700787f;
volatile float g_rainDailyMm = 0;

float rainBuckets[60] = {0};     // per-minute rain (mm) for the rolling hour
uint8_t rainBucketIdx = 0;
float lastRainDailyMm = 0;
bool haveRainBaseline = false;

// ---------- Live sensor values (shared with web handler) ----------
volatile float g_temperature = 0;
volatile float g_humidity = 0;
volatile float g_pressure = 0;
volatile float g_radiation = 0;

volatile float g_windSpeedVaisala = 0;
volatile float g_windDirVaisala = 0;
volatile float g_windSpeedSonic = 0;
volatile float g_windDirSonic = 0;

volatile float g_windSpeed = 0;
volatile float g_windDirection = 0;
volatile bool g_windSensorsAgree = false;

volatile bool g_dataValid = false;

unsigned long lastRead = 0;
const unsigned long readInterval = 5000;

// ---------- Weather Underground minute averaging ----------
const uint16_t MIN_WU_SAMPLES = 6; // Require at least 6 valid readings per field

double wuTempSum = 0;
double wuHumiditySum = 0;
double wuPressureSum = 0;
double wuRadiationSum = 0;
double wuWindSpeedSum = 0;

double wuWindDirSinSum = 0;
double wuWindDirCosSum = 0;

uint16_t wuTempCount = 0;
uint16_t wuHumidityCount = 0;
uint16_t wuPressureCount = 0;
uint16_t wuRadiationCount = 0;
uint16_t wuWindSpeedCount = 0;
uint16_t wuWindDirCount = 0;

// Last accepted readings, used to detect sudden spikes
bool wuHaveBaseline = false;
float wuLastTemp = 0;
float wuLastHumidity = 0;
float wuLastPressure = 0;
float wuLastRadiation = 0;
float wuLastWindSpeed = 0;

// ---------- Health / recovery ----------
const uint32_t WDT_TIMEOUT_MS = 30000;
const unsigned long WIFI_DEAD_RESTART_MS = 5UL * 60UL * 1000UL; // restart if offline this long
const time_t MIN_VALID_TIME = 8 * 3600 * 2;

int nanoFailCount = 0;
unsigned long lastGoodRead = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastDiag = 0;
unsigned long wifiDownSince = 0;
const unsigned long NO_REQUEST_RESTART_MS = 5UL * 60UL * 1000UL; // restart if /weather is not requested for this long
unsigned long lastWeatherRequest = 0;

// Must match the Nano exactly (9 floats = 36 bytes)
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float radiation;
  float windSpeedVaisala;
  float windDirVaisala;
  float windSpeedSonic;
  float windDirSonic;
  float rainDailyMm;
};

bool readSensorDataFromNano(SensorData &out) {
  uint8_t bytesReceived = Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)sizeof(SensorData));
  if (bytesReceived != sizeof(SensorData)) {
    while (Wire.available()) Wire.read(); // drain any partial data
    return false;
  }
  Wire.readBytes((byte*)&out, sizeof(out));
  return true;
}

void restartI2C() {
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setTimeOut(100);
}

float circularDiff(float a, float b) {
  float diff = a - b;
  while (diff > 180)  diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

float circularAverage(float a, float b) {
  float aRad = a * (float)M_PI / 180.0f;
  float bRad = b * (float)M_PI / 180.0f;
  float x = (cosf(aRad) + cosf(bRad)) / 2.0f;
  float y = (sinf(aRad) + sinf(bRad)) / 2.0f;
  float avgRad = atan2f(y, x);
  float avgDeg = avgRad * 180.0f / (float)M_PI;
  if (avgDeg < 0) avgDeg += 360.0f;
  return avgDeg;
}

void validateWindData(float speedVaisala, float dirVaisala, float speedSonic, float dirSonic,
                       float &outSpeed, float &outDir, bool &outAgree) {
  float speedDiff = fabsf(speedVaisala - speedSonic);
  float dirDiff = fabsf(circularDiff(dirVaisala, dirSonic));

  if (speedDiff <= WIND_SPEED_TOLERANCE && dirDiff <= WIND_DIR_TOLERANCE) {
    outSpeed = (speedVaisala + speedSonic) / 2.0f;
    outDir = circularAverage(dirVaisala, dirSonic);
    outAgree = true;
  } else {
    outSpeed = speedSonic;
    outDir = dirSonic;
    outAgree = false;
  }
}

// Reject non-finite, physically implausible or sudden spike readings.
// Jump limits are per reading, with readings normally 5 seconds apart.
bool acceptWUValue(float value,
                   float previous,
                   bool havePrevious,
                   float minimum,
                   float maximum,
                   float maxJump) {
  if (!isfinite(value)) return false;

  if (value < minimum || value > maximum) return false;

  if (havePrevious && fabsf(value - previous) > maxJump) {
    Serial.printf(
      "WU rejected spike: %.2f (previous %.2f, jump %.2f)\n",
      value, previous, fabsf(value - previous)
    );
    return false;
  }

  return true;
}

void accumulateWUSample() {
  // Temperature: degrees C
  if (acceptWUValue(g_temperature, wuLastTemp, wuHaveBaseline,
                    -60.0f, 70.0f, 10.0f)) {
    wuTempSum += g_temperature;
    wuTempCount++;
    wuLastTemp = g_temperature;
  }

  // Relative humidity: %
  if (acceptWUValue(g_humidity, wuLastHumidity, wuHaveBaseline,
                    0.0f, 100.0f, 25.0f)) {
    wuHumiditySum += g_humidity;
    wuHumidityCount++;
    wuLastHumidity = g_humidity;
  }

  // Atmospheric pressure: hPa
  if (acceptWUValue(g_pressure, wuLastPressure, wuHaveBaseline,
                    800.0f, 1100.0f, 50.0f)) {
    wuPressureSum += g_pressure;
    wuPressureCount++;
    wuLastPressure = g_pressure;
  }

  // Solar radiation: W/m^2
  if (acceptWUValue(g_radiation, wuLastRadiation, wuHaveBaseline,
                    0.0f, 1600.0f, 900.0f)) {
    wuRadiationSum += g_radiation;
    wuRadiationCount++;
    wuLastRadiation = g_radiation;
  }

  // Wind speed: m/s
  if (acceptWUValue(g_windSpeed, wuLastWindSpeed, wuHaveBaseline,
                    0.0f, 75.0f, 25.0f)) {
    wuWindSpeedSum += g_windSpeed;
    wuWindSpeedCount++;
    wuLastWindSpeed = g_windSpeed;
  }

  // Wind direction: degrees. Use a circular average later.
  if (isfinite(g_windDirection) &&
      g_windDirection >= 0.0f &&
      g_windDirection <= 360.0f) {
    float dirRad = g_windDirection * (float)M_PI / 180.0f;

    wuWindDirSinSum += sinf(dirRad);
    wuWindDirCosSum += cosf(dirRad);
    wuWindDirCount++;
  }

  // Baselines are available after the first accepted sample.
  wuHaveBaseline = true;
}

void resetWUAverages() {
  wuTempSum = 0;
  wuHumiditySum = 0;
  wuPressureSum = 0;
  wuRadiationSum = 0;
  wuWindSpeedSum = 0;

  wuWindDirSinSum = 0;
  wuWindDirCosSum = 0;

  wuTempCount = 0;
  wuHumidityCount = 0;
  wuPressureCount = 0;
  wuRadiationCount = 0;
  wuWindSpeedCount = 0;
  wuWindDirCount = 0;
}

// Called once per WU upload (every minute).
void updateRainForWU(float &rainHourIn, float &rainDailyIn) {
  float cur = g_rainDailyMm;
  float delta = 0;

  if (haveRainBaseline) {
    // If the total dropped (midnight reset or Nano reboot), the new total is all new rain
    delta = (cur >= lastRainDailyMm) ? (cur - lastRainDailyMm) : cur;
  }
  haveRainBaseline = true;
  lastRainDailyMm = cur;

  rainBuckets[rainBucketIdx] = delta;
  rainBucketIdx = (rainBucketIdx + 1) % 60;

  float hourMm = 0;
  for (int i = 0; i < 60; i++) hourMm += rainBuckets[i];

  rainHourIn  = hourMm * RAIN_MM_TO_IN;
  rainDailyIn = cur * RAIN_MM_TO_IN;
}

// Sends the reset command to the Nano once per local day.
void checkDailyRainReset() {
  time_t now = time(nullptr);
  if (now < MIN_VALID_TIME) return;   // wait for NTP

  struct tm t;
  localtime_r(&now, &t);
  char today[11];
  strftime(today, sizeof(today), "%Y-%m-%d", &t);

  if (strcmp(today, lastRainResetDate) == 0) return;

  // On first boot just record the date, so a mid-day ESP32 restart doesn't wipe today's rain
  if (lastRainResetDate[0] != '\0') {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(CMD_RESET_RAIN);
    if (Wire.endTransmission() != 0) {
      Serial.println("Rain reset failed, will retry");
      return;
    }
    Serial.println("Daily rain reset sent to Nano");
  }
  strncpy(lastRainResetDate, today, sizeof(lastRainResetDate));
  lastRainResetDate[sizeof(lastRainResetDate) - 1] = '\0';
}

void uploadToWeatherUnderground() {
  // Snapshot this minute's data and start a fresh collection window.
  const double tempSum = wuTempSum;
  const double humiditySum = wuHumiditySum;
  const double pressureSum = wuPressureSum;
  const double radiationSum = wuRadiationSum;
  const double windSpeedSum = wuWindSpeedSum;
  const double windDirSinSum = wuWindDirSinSum;
  const double windDirCosSum = wuWindDirCosSum;

  const uint16_t tempCount = wuTempCount;
  const uint16_t humidityCount = wuHumidityCount;
  const uint16_t pressureCount = wuPressureCount;
  const uint16_t radiationCount = wuRadiationCount;
  const uint16_t windSpeedCount = wuWindSpeedCount;
  const uint16_t windDirCount = wuWindDirCount;

  resetWUAverages();

  // Rain: update the rolling hour every minute, even if the upload is skipped
  float rainHourIn = 0, rainDailyIn = 0;
  bool haveRain = g_dataValid;
  if (haveRain) updateRainForWU(rainHourIn, rainDailyIn);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WU upload skipped: WiFi disconnected");
    return;
  }

  // Require enough valid samples for at least one field.
  bool haveTemp = tempCount >= MIN_WU_SAMPLES;
  bool haveHumidity = humidityCount >= MIN_WU_SAMPLES;
  bool havePressure = pressureCount >= MIN_WU_SAMPLES;
  bool haveRadiation = radiationCount >= MIN_WU_SAMPLES;
  bool haveWindSpeed = windSpeedCount >= MIN_WU_SAMPLES;
  bool haveWindDir = windDirCount >= MIN_WU_SAMPLES;

  if (!haveTemp && !haveHumidity && !havePressure &&
      !haveRadiation && !haveWindSpeed && !haveWindDir) {
    Serial.println("WU upload skipped: insufficient valid samples");
    return;
  }

  // Calculate averages from accepted readings only.
  float avgTempC = haveTemp ? tempSum / tempCount : 0;
  float avgHumidity = haveHumidity ? humiditySum / humidityCount : 0;
  float avgPressure = havePressure ? pressureSum / pressureCount : 0;
  float avgRadiation = haveRadiation ? radiationSum / radiationCount : 0;
  float avgWindSpeed = haveWindSpeed ? windSpeedSum / windSpeedCount : 0;

  float avgWindDir = 0;
  if (haveWindDir) {
    avgWindDir = atan2f(
      (float)windDirSinSum,
      (float)windDirCosSum
    ) * 180.0f / (float)M_PI;

    if (avgWindDir < 0) avgWindDir += 360.0f;
  }

  // Convert averaged values to Weather Underground units.
  float tempF = (avgTempC) * 9.0f / 5.0f + 32.0f;
  float pressureInHg = avgPressure * 0.0295299830714f;
  float windMph = avgWindSpeed * 2.23694f;

  String url =
    "http://rtupdate.wunderground.com/weatherstation/updateweatherstation.php";

  url += "?ID=" + String(WU_STATION_ID);
  url += "&PASSWORD=" + String(WU_STATION_KEY);
  url += "&dateutc=now";

  // Only send fields with enough valid samples.
  if (haveTemp)
    url += "&tempf=" + String(tempF, 1);

  if (haveHumidity)
    url += "&humidity=" + String(avgHumidity, 1);

  if (havePressure)
    url += "&baromin=" + String(pressureInHg, 2);

  if (haveWindDir)
    url += "&winddir=" + String((int)roundf(avgWindDir) % 360);

  if (haveWindSpeed)
    url += "&windspeedmph=" + String(windMph, 1);

  if (haveRadiation)
    url += "&solarradiation=" + String(avgRadiation, 1);

  if (haveRain) {
    url += "&rainin=" + String(rainHourIn, 3);
    url += "&dailyrainin=" + String(rainDailyIn, 3);
  }

  url += "&action=updateraw";
  url += "&realtime=1";
  url += "&rtfreq=60";

  Serial.println("---- WU minute averages ----");

  if (haveTemp)
    Serial.printf("Average temperature: %.2f C\n", avgTempC);

  if (haveHumidity)
    Serial.printf("Average humidity: %.2f %%\n", avgHumidity);

  if (havePressure)
    Serial.printf("Average pressure: %.2f hPa\n", avgPressure);

  if (haveRadiation)
    Serial.printf("Average radiation: %.2f W/m^2\n", avgRadiation);

  if (haveWindSpeed)
    Serial.printf("Average wind speed: %.2f m/s\n", avgWindSpeed);

  if (haveWindDir)
    Serial.printf("Average wind direction: %.1f degrees\n", avgWindDir);

  if (haveRain)
    Serial.printf("Rain: %.3f in last hour, %.3f in today\n", rainHourIn, rainDailyIn);

  HTTPClient http;
  http.setConnectTimeout(3000);
  http.setTimeout(5000);

  if (!http.begin(url)) {
    Serial.println("WU upload failed: unable to start HTTP request");
    return;
  }

  int httpCode = http.GET();

  if (httpCode == 200) {
    String response = http.getString();
    Serial.print("WU upload response: ");
    Serial.println(response);
  } else {
    Serial.printf("WU upload failed, HTTP code: %d\n", httpCode);
  }

  http.end();
}

// Rain is intentionally NOT included here, so the display never sees it.
void handleWeather() {
  lastWeatherRequest = millis(); 
  JsonDocument doc;

  if (g_dataValid) {
    doc["temperature"]   = g_temperature;
    doc["humidity"]      = g_humidity;
    doc["pressure"]      = g_pressure;
    doc["radiation"]     = g_radiation;

    doc["windSpeed"]     = g_windSpeed;
    doc["windDirection"] = g_windDirection;
    doc["windSensorsAgree"] = g_windSensorsAgree;

    doc["windSpeedVaisala"] = g_windSpeedVaisala;
    doc["windDirVaisala"]   = g_windDirVaisala;
    doc["windSpeedSonic"]   = g_windSpeedSonic;
    doc["windDirSonic"]     = g_windDirSonic;
  } else {
    doc["error"] = "sensor data not yet available";
  }

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());

  pinMode(ledPin, OUTPUT);

  Wire.begin();
  Wire.setTimeOut(100);

  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiStart > 30000) {
      Serial.println("\nWiFi connect timed out, restarting");
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("weatherstation")) {
    Serial.println("mDNS responder started: weatherstation.local");
  }

  server.on("/weather", handleWeather);
  server.begin();

  // ---------- Configure OTA ----------
  ArduinoOTA.setHostname("weatherstation");

  ArduinoOTA.onStart([]() {
    esp_task_wdt_reset();
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    esp_task_wdt_reset();
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  // -----------------------------------

  // NTP sync (GMT+2)
  configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync");
  unsigned long ntpStart = millis();
  time_t now = time(nullptr);
  while (now < MIN_VALID_TIME && millis() - ntpStart < 30000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println();
  Serial.println(now >= MIN_VALID_TIME ? "Time synced." : "NTP not synced yet, continuing.");

  // Watchdog
  esp_task_wdt_deinit();
  esp_task_wdt_config_t cfg = { .timeout_ms = WDT_TIMEOUT_MS, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&cfg);
  esp_task_wdt_add(NULL);

  lastGoodRead = millis();
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  ArduinoOTA.handle(); // Handle wireless uploads

  if (millis() - lastRead >= readInterval) {
    lastRead = millis();

    checkDailyRainReset();

    digitalWrite(ledPin, HIGH);

    SensorData nanoData;
    if (readSensorDataFromNano(nanoData)) {
      g_temperature = nanoData.temperature + 9;
      g_humidity    = nanoData.humidity;
      g_pressure    = nanoData.pressure;
      g_radiation   = nanoData.radiation;

      g_windSpeedVaisala = nanoData.windSpeedVaisala;
      g_windDirVaisala   = nanoData.windDirVaisala;
      g_windSpeedSonic   = nanoData.windSpeedSonic;
      g_windDirSonic     = nanoData.windDirSonic;

      g_rainDailyMm = nanoData.rainDailyMm;

      float combinedSpeed, combinedDir;
      bool agree;
      validateWindData(g_windSpeedVaisala, g_windDirVaisala, g_windSpeedSonic, g_windDirSonic,
                       combinedSpeed, combinedDir, agree);
      g_windSpeed = combinedSpeed;
      g_windDirection = combinedDir;
      g_windSensorsAgree = agree;

      g_dataValid = true;
      nanoFailCount = 0;
      lastGoodRead = millis();

      // Add this successful sensor reading to the WU minute average.
      accumulateWUSample();

    } else {
      Serial.println("Failed to read from Nano 33 IoT over I2C");
      nanoFailCount++;
      if (nanoFailCount >= 3) {
        g_dataValid = false;
      }
      if (nanoFailCount % 5 == 0) {
        Serial.println("Restarting I2C bus");
        restartI2C();
      }
    }

    Serial.println("----");
    Serial.print("Temperature: "); Serial.print(g_temperature); Serial.println(" C");
    Serial.print("Humidity: "); Serial.print(g_humidity); Serial.println(" %RH");
    Serial.print("Pressure: "); Serial.print(g_pressure); Serial.println(" hPa");
    Serial.print("Solar Radiation: "); Serial.print(g_radiation, 1); Serial.println(" W/m^2");
    Serial.print("Rain today: "); Serial.print(g_rainDailyMm, 2); Serial.println(" mm");

    Serial.print("Vaisala:   "); Serial.print(g_windSpeedVaisala, 2); Serial.print(" m/s @ ");
    Serial.print(g_windDirVaisala, 1); Serial.println(" deg");
    Serial.print("WindSonic: "); Serial.print(g_windSpeedSonic, 2); Serial.print(" m/s @ ");
    Serial.print(g_windDirSonic, 1); Serial.println(" deg");
    Serial.print("Combined:  "); Serial.print(g_windSpeed, 2); Serial.print(" m/s @ ");
    Serial.print(g_windDirection, 1); Serial.print(" deg  (sensors agree: ");
    Serial.print(g_windSensorsAgree ? "yes" : "NO");
    Serial.println(")");

    digitalWrite(ledPin, LOW);
  }

  // WiFi supervision
  if (millis() - lastWifiCheck > 15000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiDownSince == 0) wifiDownSince = millis();
      Serial.println("WiFi down, reconnecting");
      WiFi.disconnect();
      WiFi.reconnect();
      if (millis() - wifiDownSince > WIFI_DEAD_RESTART_MS) {
        Serial.println("WiFi down too long, restarting");
        ESP.restart();
      }
    } else {
      wifiDownSince = 0;
    }
  }

  // Web server backstop: restart if nobody has fetched /weather for a while
  if (millis() - lastWeatherRequest > NO_REQUEST_RESTART_MS) {
    Serial.println("No /weather requests for 5 min, restarting");
    delay(100);
    ESP.restart();
  }

  // Diagnostics
  if (millis() - lastDiag > 30000) {
    lastDiag = millis();
    Serial.printf("up=%lus heap=%u minHeap=%u rssi=%d nanoFails=%d lastGood=%lus ago lastReq=%lus ago\n",
                  millis() / 1000, ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                  WiFi.RSSI(), nanoFailCount, (millis() - lastGoodRead) / 1000,
                  (millis() - lastWeatherRequest) / 1000);
  }

  if (millis() - lastWuUpload >= wuUploadInterval) {
    lastWuUpload = millis();
    uploadToWeatherUnderground();
  }
}