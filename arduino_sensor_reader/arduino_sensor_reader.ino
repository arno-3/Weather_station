#include <Wire.h>
#include <Adafruit_SleepyDog.h>

#define I2C_ADDR 0x08

// ---------- Analog inputs (Nano 33 IoT, 12-bit, 3.3 V) ----------
const int tempPin  = A0;   // Temperature (4-20mA loop)
const int rhPin    = A1;   // Humidity (4-20mA loop)
const int pressPin = A2;   // Pressure (4-20mA loop)
const int solarPin = A3;   // Solar radiation (0-1V)

const float SHUNT_RESISTOR = 165.0;
const float ADC_MAX = 4095.0;
const float VREF = 3.3;

const int ledPin = 13;

// ---------- Timing ----------
const unsigned long LOOP_PERIOD_MS = 1000;
unsigned long nextLoopMs = 0;
unsigned long lastSampleMs = 0;

// ---------- Watchdog ----------
const int WDT_TIMEOUT_MS = 8000;   // reboot if the loop stalls for 8 s (SAMD21 max is about 16 s)

// ---------- Wind speed (Vaisala WAA151) ----------
const int windSpeedPin = 2;
volatile unsigned long pulseCount = 0;
const float PULSES_PER_HZ_PER_MS = 10.0; // 10 Hz per 1 m/s

// ---------- Rain gauge (tipping bucket, reed switch) ----------
const int rainPin = 3;                          // reed switch between D3 and GND
const float RAIN_MM_PER_TIP = 0.1;              // CHANGE to match your bucket
const unsigned long RAIN_DEBOUNCE_MS = 100;
const byte CMD_RESET_RAIN = 0xAA;               // must match the ESP32

volatile uint32_t rainTips = 0;
volatile unsigned long lastRainTipMs = 0;
volatile bool rainResetRequested = false;

void rainISR() {
  unsigned long now = millis();
  if (now - lastRainTipMs >= RAIN_DEBOUNCE_MS) {
    rainTips++;
    lastRainTipMs = now;
  }
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    byte c = Wire.read();
    if (c == CMD_RESET_RAIN) rainResetRequested = true;
  }
}

// Both sensors are mounted with their north marker pointing south
const float VAISALA_DIR_OFFSET   = 180.0;
const float WINDSONIC_DIR_OFFSET = 180.0;

float normalizeDeg(float deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0)    deg += 360.0;
  return deg;
}

void windSpeedISR() {
  pulseCount++;
}

// ---------- Wind direction (Vaisala WAV151), 6-bit Gray code ----------
// Confirm bit order against your wiring. Inputs must never exceed 3.3 V.
const int dirPins[6] = {11, 4, 5, 6, 7, 8};

int grayToBinary(int gray) {
  int binary = gray;
  for (int mask = binary >> 1; mask != 0; mask >>= 1) {
    binary ^= mask;
  }
  return binary;
}

float readWindDirectionVaisala() {
  int rawGray = 0;
  for (int i = 0; i < 6; i++) {
    rawGray |= (digitalRead(dirPins[i]) << i);
  }
  float deg = grayToBinary(rawGray) * (360.0 / 64.0);
  return normalizeDeg(deg + VAISALA_DIR_OFFSET);
}

// ---------- Gill WindSonic via RS232->TTL (MAX3232) on Serial1 ----------
// Format: Q,ddd,sss.ss,M,00,   (direction field is empty in calm conditions)
const long WINDSONIC_BAUD = 9600;
const int WINDSONIC_BUF_LEN = 64;
char windSonicBuffer[WINDSONIC_BUF_LEN];
int windSonicBufIdx = 0;

float windSonicSpeed = 0.0;
float windSonicDirection = 0.0;
bool windSonicDataValid = false;

void readWindSonic() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      windSonicBuffer[windSonicBufIdx] = '\0';

      char *firstComma  = strchr(windSonicBuffer, ',');
      char *secondComma = firstComma  ? strchr(firstComma + 1, ',')  : NULL;
      char *thirdComma  = secondComma ? strchr(secondComma + 1, ',') : NULL;

      if (firstComma && secondComma && thirdComma) {
        // Direction field empty (only spaces) means calm conditions
        bool hasDir = false;
        for (char *p = firstComma + 1; p < secondComma; p++) {
          if (*p != ' ') { hasDir = true; break; }
        }

        float parsedDir   = hasDir ? atof(firstComma + 1) : 0.0;
        float parsedSpeed = atof(secondComma + 1);

        bool speedOk = (parsedSpeed >= 0 && parsedSpeed <= 75);
        bool dirOk   = !hasDir || (parsedDir >= 0 && parsedDir <= 360);

        if (speedOk && dirOk) {
          windSonicSpeed = parsedSpeed;
          if (hasDir) {
            windSonicDirection = normalizeDeg(parsedDir + WINDSONIC_DIR_OFFSET);
          }
          windSonicDataValid = true;
        }
      }
      windSonicBufIdx = 0;
    } else if (c != '\r') {
      if (windSonicBufIdx < WINDSONIC_BUF_LEN - 1) {
        windSonicBuffer[windSonicBufIdx++] = c;
      } else {
        windSonicBufIdx = 0; // corrupted or never-terminated line
      }
    }
  }
}

// ---------- Shared struct, MUST match the ESP32 exactly (9 floats) ----------
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

// Only written inside noInterrupts(), so requestEvent() always sees a complete set
SensorData sharedData = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// ---------- Helpers ----------
float readVoltage(int pin) {
  int raw = analogRead(pin);
  return (raw / ADC_MAX) * VREF;
}

float readMilliAmps(int pin) {
  return (readVoltage(pin) / SHUNT_RESISTOR) * 1000.0;
}

float mapCurrent(float mA, float minVal, float maxVal) {
  return (mA - 4.0) * (maxVal - minVal) / 16.0 + minVal;
}

float circularDiff(float a, float b) {
  float diff = a - b;
  while (diff > 180)  diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(WINDSONIC_BAUD);

  analogReadResolution(12);
  pinMode(ledPin, OUTPUT);

  pinMode(windSpeedPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), windSpeedISR, FALLING);

  for (int i = 0; i < 6; i++) {
    pinMode(dirPins[i], INPUT);
  }

  // Rain gauge reed switch
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), rainISR, FALLING);

  Wire.begin(I2C_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  noInterrupts();
  pulseCount = 0;
  interrupts();
  lastSampleMs = millis();
  nextLoopMs   = lastSampleMs + LOOP_PERIOD_MS;

  // Keep this LAST in setup()
  int actualMs = Watchdog.enable(WDT_TIMEOUT_MS);
  Serial.print("Watchdog enabled, timeout ms: ");
  Serial.println(actualMs);
}

void loop() {
  // Wait for the next cycle, draining the WindSonic UART and feeding the watchdog
  while ((long)(millis() - nextLoopMs) < 0) {
    readWindSonic();
    Watchdog.reset();
  }
  nextLoopMs += LOOP_PERIOD_MS;
  Watchdog.reset();

  // Daily rain reset requested by the ESP32 at local midnight
  if (rainResetRequested) {
    noInterrupts();
    rainTips = 0;
    rainResetRequested = false;
    interrupts();
    Serial.println("Daily rain reset");
  }

  digitalWrite(ledPin, HIGH);
  readWindSonic();

  // Build this cycle's readings locally
  SensorData data;

  data.temperature = mapCurrent(readMilliAmps(tempPin),  -50.0, 50.0);
  data.humidity    = mapCurrent(readMilliAmps(rhPin),      0.0, 100.0);
  data.pressure    = mapCurrent(readMilliAmps(pressPin), 500.0, 1100.0);
  data.radiation   = readVoltage(solarPin) * 1396.0;

  // Vaisala speed: pulses divided by the real elapsed time
  unsigned long now = millis();
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  float elapsedSec = (now - lastSampleMs) / 1000.0;
  lastSampleMs = now;

  float frequencyHz = (elapsedSec > 0.0) ? (pulses / elapsedSec) : 0.0;
  data.windSpeedVaisala = frequencyHz / PULSES_PER_HZ_PER_MS;
  data.windDirVaisala   = readWindDirectionVaisala();

  data.windSpeedSonic = windSonicSpeed;
  data.windDirSonic   = windSonicDirection;

  // Rain: daily total from tip count
  noInterrupts();
  uint32_t tipsSnapshot = rainTips;
  interrupts();
  data.rainDailyMm = tipsSnapshot * RAIN_MM_PER_TIP;

  // Publish the complete set in one atomic copy
  noInterrupts();
  sharedData = data;
  interrupts();

  Serial.println("----");
  Serial.print("Temperature: "); Serial.print(data.temperature); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(data.humidity); Serial.println(" %RH");
  Serial.print("Pressure: "); Serial.print(data.pressure); Serial.println(" hPa");
  Serial.print("Solar Radiation: "); Serial.print(data.radiation, 1); Serial.println(" W/m^2");
  Serial.print("Rain today: "); Serial.print(data.rainDailyMm, 2); Serial.println(" mm");

  Serial.println("-- Wind comparison --");
  Serial.print("Vaisala:   "); Serial.print(data.windSpeedVaisala, 2); Serial.print(" m/s @ ");
  Serial.print(data.windDirVaisala, 1); Serial.println(" deg");

  if (windSonicDataValid) {
    Serial.print("WindSonic: "); Serial.print(data.windSpeedSonic, 2); Serial.print(" m/s @ ");
    Serial.print(data.windDirSonic, 1); Serial.println(" deg");
    Serial.print("Difference: ");
    Serial.print(data.windSpeedVaisala - data.windSpeedSonic, 2); Serial.print(" m/s, ");
    Serial.print(circularDiff(data.windDirVaisala, data.windDirSonic), 1); Serial.println(" deg");
  } else {
    Serial.println("WindSonic: no valid data yet");
  }

  digitalWrite(ledPin, LOW);
}

void requestEvent() {
  Wire.write((byte*)&sharedData, sizeof(sharedData));
}