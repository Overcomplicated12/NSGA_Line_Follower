/*
 * Zumo line following (LOOSER) + telemetry + stop conditions + recovery
 * + ESP8266 (AT firmware) UDP telemetry + optional remote FAIL command
 *
 * UNO + Serial Monitor compatible:
 * - Serial (USB) = logs/CSV
 * - ESP8266 AT = SoftwareSerial on ESP_RX_PIN / ESP_TX_PIN
 *
 * IMPORTANT:
 * - SoftwareSerial is unreliable at high baud. Use 9600 (recommended).
 * - Set your ESP8266 AT firmware UART baud to 9600, OR change ESP_BAUD if yours is different.
 *
 * PMD Way Uno+ESP board:
 * - Set DIP switches so ESP8266 is connected to Arduino via GPIO (NOT via USB, NOT via Arduino HW Serial).
 */

#include <Wire.h>
#include <ZumoShield.h>
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>

// =====================
// ESP8266 CONFIG (AT)
// =====================
#define USE_ESP8266 1

#if USE_ESP8266
// Pick pins that your board routes ESP TX/RX to in "Arduino <-> ESP via GPIO" mode.
// ESP_TX -> Arduino RX pin, ESP_RX <- Arduino TX pin.
const uint8_t ESP_RX_PIN = 2;   // Arduino receives from ESP (connect to ESP TX)
const uint8_t ESP_TX_PIN = 3;   // Arduino transmits to ESP (connect to ESP RX)

SoftwareSerial ESP(ESP_RX_PIN, ESP_TX_PIN); // (RX, TX)

// WiFi credentials
const char WIFI_SSID[] = "YOUR_SSID";
const char WIFI_PASS[] = "YOUR_PASSWORD";

// UDP destination (your PC IP + port)
const char UDP_HOST[] = "192.168.1.50";
const int  UDP_PORT   = 9999;

// IMPORTANT: keep low for SoftwareSerial stability
const unsigned long ESP_BAUD = 9600;

const unsigned long ESP_CMD_TIMEOUT = 2500;
#endif

// =====================
// ZUMO OBJECTS
// =====================
ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

// =====================
// NSGA-II PASTE LINE
// Order: kp,kd,base_speed,min_base_speed,corner1,corner2,corner3,brake_pwr
// =====================
#define NSGA_PARAMS \
  0.180000,1.350000,260,90,900,1500,2000,120

// ---------- TUNING (auto-filled from NSGA_PARAMS) ----------
const int MAX_SPEED = 400;

const float KP = [](){ float a[] = { NSGA_PARAMS }; return a[0]; }();
const float KD = [](){ float a[] = { NSGA_PARAMS }; return a[1]; }();

const int BASE_SPEED     = (int)([](){ float a[] = { NSGA_PARAMS }; return a[2]; }());
const int MIN_BASE_SPEED = (int)([](){ float a[] = { NSGA_PARAMS }; return a[3]; }());

const int CORNER_ERR1 = (int)([](){ float a[] = { NSGA_PARAMS }; return a[4]; }());
const int CORNER_ERR2 = (int)([](){ float a[] = { NSGA_PARAMS }; return a[5]; }());
const int CORNER_ERR3 = (int)([](){ float a[] = { NSGA_PARAMS }; return a[6]; }());

const bool USE_BRAKE_PULSE = true;
const int BRAKE_ERR = 1900;
const int BRAKE_MS  = 20;
const int BRAKE_PWR = (int)([](){ float a[] = { NSGA_PARAMS }; return a[7]; }());
// ----------------------------------------------------------

// ---------- RUN CONTROL ----------
const unsigned long RUN_TIME_MS   = 20000;
const unsigned long KILL_HOLD_MS  = 1000;
const unsigned long FINISH_MS     = 100;
const unsigned long TELEMETRY_MS  = 100;
// --------------------------------

// ---------- RECOVERY / LOOSENESS ----------
const int TURN_CAP = 350;
const unsigned long LOST_LINE_MS = 120;
const unsigned long STUCK_CORNER_MS = 250;
const int SEARCH_SPEED = 180;
const int ESCAPE_SPEED = 240;
const unsigned int WHITE_MAX = 200;
// -----------------------------------------

// ---------- SERIAL/REMOTE FAIL ----------
bool serialFailTriggered = false;
// ---------------------------------------

int lastError = 0;

// =====================
// SMALL UTILS
// =====================
static int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static int iabs(int x) { return x < 0 ? -x : x; }

void stopMotors() { motors.setSpeeds(0, 0); }

bool allBlack(const unsigned int s[6], unsigned int thresh) {
  for (int i = 0; i < 6; i++) if (s[i] < thresh) return false;
  return true;
}
bool allWhiteish(const unsigned int s[6], unsigned int thresh) {
  for (int i = 0; i < 6; i++) if (s[i] > thresh) return false;
  return true;
}

bool buttonHeldFor(unsigned long ms) {
  unsigned long t0 = millis();
  while (button.isPressed()) {
    if (millis() - t0 >= ms) return true;
  }
  return false;
}

// =====================
// ESP8266 (AT) HELPERS
// =====================
#if USE_ESP8266

static void espFlush() {
  while (ESP.available()) ESP.read();
}

static bool espWaitFor(const char* token, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  size_t idx = 0;
  size_t n = strlen(token);

  while (millis() - t0 < timeoutMs) {
    while (ESP.available()) {
      char c = ESP.read();
      if (c == token[idx]) {
        idx++;
        if (idx >= n) return true;
      } else {
        idx = (c == token[0]) ? 1 : 0;
      }
    }
  }
  return false;
}

static bool espSendCmd(const char* cmd, const char* okToken = "OK", unsigned long timeoutMs = ESP_CMD_TIMEOUT) {
  espFlush();
  ESP.println(cmd);
  if (!okToken) return true;
  return espWaitFor(okToken, timeoutMs);
}

static bool espInitWifiUdp() {
  ESP.begin(ESP_BAUD);
  delay(300);

  if (!espSendCmd("AT", "OK")) return false;
  espSendCmd("ATE0", "OK");
  espSendCmd("AT+CWMODE=1", "OK");

  char join[180];
  snprintf(join, sizeof(join), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);

  // Join can be slow
  bool gotIP = espSendCmd(join, "WIFI GOT IP", 15000);
  if (!gotIP) {
    // sometimes it just ends with OK
    if (!espWaitFor("OK", 5000)) return false;
  } else {
    espWaitFor("OK", 5000);
  }

  espSendCmd("AT+CIPMUX=0", "OK");

  char start[180];
  snprintf(start, sizeof(start), "AT+CIPSTART=\"UDP\",\"%s\",%d", UDP_HOST, UDP_PORT);

  if (!espSendCmd(start, "OK", 6000)) {
    // some firmwares say CONNECT instead of OK
    if (!espWaitFor("CONNECT", 6000)) return false;
  } else {
    espWaitFor("CONNECT", 2000);
  }

  return true;
}

static bool espUdpSend(const char* msg) {
  int len = (int)strlen(msg);
  if (len <= 0) return true;

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);

  if (!espSendCmd(cmd, ">", 3000)) return false;
  ESP.print(msg);
  return espWaitFor("SEND OK", 4000);
}

// Optional: detect remote FAIL from incoming ESP data.
// Many AT firmwares output +IPD,...:<payload> for incoming UDP.
// We just scan for 'F' anywhere.
static void espPollRemoteFail() {
  while (ESP.available()) {
    char c = ESP.read();
    if (c == 'F') serialFailTriggered = true;
  }
}
#endif

// =====================
// CALIBRATION
// =====================
void calibrateSpin() {
  Serial.println(F("Calibrating... spinning"));
  digitalWrite(13, HIGH);

  for (int i = 0; i < 80; i++) {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);

    reflectanceSensors.calibrate();
    delay(20);
  }

  stopMotors();
  digitalWrite(13, LOW);
  Serial.println(F("Calibration done."));
}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  buzzer.play(">g32>>c32");
  reflectanceSensors.init();

#if USE_ESP8266
  Serial.println(F("ESP8266: init WiFi+UDP..."));
  bool ok = espInitWifiUdp();
  Serial.println(ok ? F("ESP8266: OK") : F("ESP8266: FAILED (continuing without UDP)"));
#endif

  Serial.println(F("\nZumo line follower starting..."));
  Serial.println(F("Press button to CALIBRATE."));
  button.waitForButton();
  delay(200);

  calibrateSpin();

  buzzer.play(">g32>>c32");
  Serial.println(F("Press button to RUN."));
  button.waitForButton();
  delay(200);
}

void loop() {
  serialFailTriggered = false;

  unsigned long startMs = millis();
  unsigned long lastTel = 0;
  unsigned long blackStart = 0;

  // stats
  uint32_t loops = 0;
  uint64_t absErrSum = 0;
  uint64_t errSqSum  = 0;
  int maxAbsErr = 0;

  uint32_t lineLostCount = 0;
  uint8_t finishDetected = 0;

  unsigned long lostSince = 0;
  unsigned long bigErrSince = 0;

  lastError = 0;

  Serial.println(F("RUN: starting. Hold button 1s to KILL. Type F + enter to FAIL."));
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying()) {}

  while (true) {
#if USE_ESP8266
    espPollRemoteFail();
    if (serialFailTriggered) {
      Serial.println(F("FAIL: remote command received."));
      finishDetected = 0;
      break;
    }
#endif

    // Local FAIL from Serial Monitor: send "F" + Enter
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'F') serialFailTriggered = true;
    }
    if (serialFailTriggered) {
      Serial.println(F("FAIL: serial command received."));
      finishDetected = 0;
      break;
    }

    if (button.isPressed() && buttonHeldFor(KILL_HOLD_MS)) {
      Serial.println(F("KILL: button held."));
      break;
    }

    unsigned long now = millis();
    if (now - startMs >= RUN_TIME_MS) {
      Serial.println(F("STOP: time limit reached."));
      break;
    }

    unsigned int sensors[6];
    int position = reflectanceSensors.readLine(sensors);
    int error = position - 2500;
    int dError = error - lastError;
    lastError = error;

    int absError = iabs(error);
    if (absError > maxAbsErr) maxAbsErr = absError;

    absErrSum += (uint32_t)absError;
    errSqSum  += (uint64_t)error * (uint64_t)error;
    loops++;

    // Finish bar detect
    const unsigned int BLACK_THRESH = 800;
    if (allBlack(sensors, BLACK_THRESH)) {
      if (blackStart == 0) blackStart = now;
      if (now - blackStart >= FINISH_MS) {
        Serial.println(F("STOP: finish line detected."));
        finishDetected = 1;
        break;
      }
    } else {
      blackStart = 0;
    }

    // LINE LOST
    bool lost = allWhiteish(sensors, WHITE_MAX);
    if (lost) {
      if (lostSince == 0) lostSince = now;
      if (now - lostSince >= LOST_LINE_MS) {
        lineLostCount++;

        int dir = (lastError >= 0) ? 1 : -1;
        motors.setSpeeds(-dir * SEARCH_SPEED, dir * SEARCH_SPEED);

        if (now - lastTel >= TELEMETRY_MS) {
          lastTel = now;
          Serial.print(F("SEARCH t=")); Serial.print(now - startMs);
          Serial.print(F(" lastErr=")); Serial.print(lastError);
          Serial.print(F(" s=["));
          for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
          Serial.println(']');

#if USE_ESP8266
          char msg[220];
          int n = snprintf(msg, sizeof(msg),
            "SEARCH t=%lu lastErr=%d\n",
            (unsigned long)(now - startMs), lastError
          );
          if (n > 0 && n < (int)sizeof(msg)) espUdpSend(msg);
#endif
        }
        continue;
      }
    } else {
      lostSince = 0;
    }

    // CORNER ESCAPE
    if (absError > 2000) {
      if (bigErrSince == 0) bigErrSince = now;
      if (now - bigErrSince >= STUCK_CORNER_MS) {
        int dir = (error >= 0) ? 1 : -1;
        motors.setSpeeds(-dir * ESCAPE_SPEED, dir * ESCAPE_SPEED);

        if (now - lastTel >= TELEMETRY_MS) {
          lastTel = now;
          Serial.print(F("ESCAPE t=")); Serial.print(now - startMs);
          Serial.print(F(" err=")); Serial.print(error);
          Serial.println();

#if USE_ESP8266
          char msg[180];
          int n = snprintf(msg, sizeof(msg),
            "ESCAPE t=%lu err=%d\n",
            (unsigned long)(now - startMs), error
          );
          if (n > 0 && n < (int)sizeof(msg)) espUdpSend(msg);
#endif
        }
        continue;
      }
    } else {
      bigErrSince = 0;
    }

    // SPEED SCHEDULING
    float e = absError / 2500.0f;
    if (e > 1.0f) e = 1.0f;

    int base = (int)(BASE_SPEED - e * 0.55f * (BASE_SPEED - MIN_BASE_SPEED));
    base = clampInt(base, MIN_BASE_SPEED, BASE_SPEED);

    if (absError > CORNER_ERR1) base = min(base, 240);
    if (absError > CORNER_ERR2) base = min(base, 200);
    if (absError > CORNER_ERR3) base = min(base, 170);

    if (USE_BRAKE_PULSE && BRAKE_PWR > 0 && absError > BRAKE_ERR) {
      motors.setSpeeds(-BRAKE_PWR, -BRAKE_PWR);
      delay(BRAKE_MS);
    }

    int turn = (int)(KP * error + KD * dError);
    turn = clampInt(turn, -TURN_CAP, TURN_CAP);

    int left  = clampInt(base + turn, -MAX_SPEED, MAX_SPEED);
    int right = clampInt(base - turn, -MAX_SPEED, MAX_SPEED);

    motors.setSpeeds(left, right);

    // Telemetry
    if (now - lastTel >= TELEMETRY_MS) {
      lastTel = now;
      Serial.print(F("t=")); Serial.print(now - startMs);
      Serial.print(F(" pos=")); Serial.print(position);
      Serial.print(F(" err=")); Serial.print(error);
      Serial.print(F(" dErr=")); Serial.print(dError);
      Serial.print(F(" base=")); Serial.print(base);
      Serial.print(F(" turn=")); Serial.print(turn);
      Serial.print(F(" L=")); Serial.print(left);
      Serial.print(F(" R=")); Serial.print(right);
      Serial.println();

#if USE_ESP8266
      char msg[220];
      int n = snprintf(msg, sizeof(msg),
        "t=%lu pos=%d err=%d dErr=%d base=%d turn=%d L=%d R=%d\n",
        (unsigned long)(now - startMs),
        position, error, dError, base, turn, left, right
      );
      if (n > 0 && n < (int)sizeof(msg)) espUdpSend(msg);
#endif
    }
  }

  stopMotors();

  // metrics
  unsigned long runtime = millis() - startMs;
  float mad = (loops == 0) ? 0.0f : (float)absErrSum / (float)loops;
  float veerScore = (mad / 2500.0f) * 100.0f;

  // CSV line to Serial Monitor
  Serial.print(KP, 6);               Serial.print(',');
  Serial.print(KD, 6);               Serial.print(',');
  Serial.print(BASE_SPEED);          Serial.print(',');
  Serial.print(MIN_BASE_SPEED);      Serial.print(',');
  Serial.print(CORNER_ERR1);         Serial.print(',');
  Serial.print(CORNER_ERR2);         Serial.print(',');
  Serial.print(CORNER_ERR3);         Serial.print(',');
  Serial.print(BRAKE_PWR);           Serial.print(',');
  Serial.print(runtime);             Serial.print(',');
  Serial.print(veerScore, 2);        Serial.print(',');
  Serial.print(lineLostCount);       Serial.print(',');
  Serial.println(finishDetected);

#if USE_ESP8266
  // Same CSV via UDP (optional)
  char csv[260];
  int n = snprintf(csv, sizeof(csv),
    "%.6f,%.6f,%d,%d,%d,%d,%d,%d,%lu,%.2f,%lu,%u\n",
    KP, KD,
    BASE_SPEED, MIN_BASE_SPEED,
    CORNER_ERR1, CORNER_ERR2, CORNER_ERR3,
    BRAKE_PWR,
    (unsigned long)runtime,
    veerScore,
    (unsigned long)lineLostCount,
    (unsigned)finishDetected
  );
  if (n > 0 && n < (int)sizeof(csv)) espUdpSend(csv);
#endif

  Serial.println(F("Press button to run again."));
  button.waitForButton();
  delay(250);
}
