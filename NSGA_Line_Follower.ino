/*
 * Zumo line following (LOOSER) + telemetry + stop conditions + recovery
 * + ESP8266 (AT firmware) UDP telemetry + remote FAIL command
 *
 * IMPORTANT for your PMD Way Uno+ESP8266 combo board:
 * - When RUNNING with ESP enabled, the Uno's hardware Serial (pins 0/1) is used to talk to the ESP.
 * - That means you generally CANNOT use the USB Serial Monitor at the same time (because Serial is "for ESP").
 * - Set DIP switches to connect: Arduino <-> ESP serial TX/RX (and NOT Arduino <-> USB) while running.
 * - Use UDP telemetry to see output on your PC instead of Serial Monitor.
 *
 * ESP expects AT firmware.
 *
 * Remote FAIL:
 * - Send a UDP packet containing "F\n" to the ESP (to its UDP stream). This code will treat any received 'F' as FAIL.
 *
 * Telemetry:
 * - Sends periodic "t=... pos=... err=..." lines via UDP
 * - Sends final CSV summary line via UDP:
 *   kp,kd,base_speed,min_base_speed,corner1,corner2,corner3,brake_pwr,runtime_ms,veerScore,lineLost,finish
 */

#include <Wire.h>
#include <ZumoShield.h>
#include <string.h>
#include <math.h>

// =====================
// ESP8266 CONFIG
// =====================
// For your Uno+ESP board: use HW Serial to ESP (Serial pins 0/1 routed by DIP).
#define USE_ESP8266 1

#if USE_ESP8266
  // ESP is on the Uno hardware serial
  #define ESP Serial

  // WiFi credentials
  const char WIFI_SSID[] = "YOUR_SSID";
  const char WIFI_PASS[] = "YOUR_PASSWORD";

  // UDP sink (your PC/server IP and port to receive telemetry)
  const char UDP_HOST[] = "192.168.1.50";
  const int  UDP_PORT   = 9999;

  // ESP baud: must match your ESP AT firmware setting.
  // Common values: 115200, 57600, 9600
  const unsigned long ESP_BAUD = 115200;

  const unsigned long ESP_CMD_TIMEOUT = 2500;

  // If true, we avoid printing AT responses anywhere; all "logging" goes out via UDP.
  // (Do not use Serial Monitor while ESP is attached on Serial.)
  #define LOG_OVER_UDP 1
#else
  #define LOG_OVER_UDP 0
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
const int MAX_SPEED = 400;  // allow full speed (Zumo is <=400)

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
const int TURN_CAP = 350;                 // looser: cap the turning strength
const unsigned long LOST_LINE_MS = 120;   // how long before we consider it "lost"
const unsigned long STUCK_CORNER_MS = 250;// big error for this long => escape
const int SEARCH_SPEED = 180;             // pivot speed during search
const int ESCAPE_SPEED = 240;             // stronger pivot escape
// For sensor "line lost" detection using calibrated readings:
const unsigned int WHITE_MAX = 200;       // if ALL sensors <= this => likely no black line
// -----------------------------------------

// ---------- FAIL COMMAND FLAGS ----------
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

static bool espInitWifi() {
  ESP.begin(ESP_BAUD);
  delay(300);

  // Basic AT sanity
  if (!espSendCmd("AT", "OK")) return false;

  // Less noise
  espSendCmd("ATE0", "OK");

  // Station mode
  espSendCmd("AT+CWMODE=1", "OK");

  // Join AP
  char join[180];
  snprintf(join, sizeof(join), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);

  // Some firmwares emit "WIFI GOT IP" before OK; accept either
  bool got = espSendCmd(join, "WIFI GOT IP", 15000);
  if (!got) {
    // maybe it just returns OK
    if (!espWaitFor("OK", 3000)) return false;
  } else {
    // also wait for OK if it comes
    espWaitFor("OK", 3000);
  }

  // Single connection
  espSendCmd("AT+CIPMUX=0", "OK");

  // Start UDP connection
  char start[180];
  snprintf(start, sizeof(start), "AT+CIPSTART=\"UDP\",\"%s\",%d", UDP_HOST, UDP_PORT);

  // Accept OK/CONNECT-ish responses
  if (!espSendCmd(start, "OK", 5000)) {
    // Some firmwares might answer "CONNECT"
    if (!espWaitFor("CONNECT", 5000)) return false;
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

  if (!espSendCmd(cmd, ">", 2500)) return false;
  ESP.print(msg);

  // Wait for SEND OK
  return espWaitFor("SEND OK", 3500);
}

// Very lightweight "remote FAIL" detector.
// Many AT firmwares will output incoming UDP packets with "+IPD,...:<payload>"
// We simply scan incoming bytes; if we see an 'F' anywhere, trigger FAIL.
static void espPollRemoteFail() {
  while (ESP.available()) {
    char c = ESP.read();
    if (c == 'F') {
      serialFailTriggered = true;
    }
  }
}

// Logging helpers
static void logLine(const char* s) {
#if LOG_OVER_UDP
  espUdpSend(s);
  espUdpSend("\n");
#else
  // If you ever disable LOG_OVER_UDP, beware: printing to Serial will go to ESP and break AT.
  (void)s;
#endif
}

#endif // USE_ESP8266

// =====================
// CALIBRATION
// =====================
void calibrateSpin() {
#if USE_ESP8266
  logLine("Calibrating... spinning");
#endif
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

#if USE_ESP8266
  logLine("Calibration done.");
#endif
}

// =====================
// SETUP / LOOP
// =====================
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

#if USE_ESP8266
  // Initialize ESP first. (No Serial Monitor in this mode.)
  bool ok = espInitWifi();

  // Beep patterns to indicate ESP status
  if (ok) buzzer.play(">g16>>c16");
  else    buzzer.play("l16 c"); // short sad beep

  // Give some time for buzzer
  delay(200);

  if (ok) logLine("ESP8266: WiFi+UDP OK");
  else    logLine("ESP8266: init FAILED (continuing without UDP)");
#else
  // Non-ESP mode (normal Arduino USB Serial Monitor)
  Serial.begin(115200);
  Serial.println(F("\nZumo line follower starting..."));
#endif

  buzzer.play(">g32>>c32");
  reflectanceSensors.init();

#if !USE_ESP8266
  Serial.println(F("Press button to CALIBRATE."));
#endif

  button.waitForButton();
  delay(200);

  calibrateSpin();

  buzzer.play(">g32>>c32");

#if !USE_ESP8266
  Serial.println(F("Press button to RUN."));
#endif

  button.waitForButton();
  delay(200);
}

void loop() {
  // ---- RESET "FAIL" state for the next run ----
  serialFailTriggered = false;
  // --------------------------------------------

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

#if USE_ESP8266
  logLine("RUN: starting. Hold button 1s to KILL.");
#else
  Serial.println(F("RUN: starting. Hold button 1s to KILL."));
#endif

  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying()) {}

  while (true) {
#if USE_ESP8266
    // Remote FAIL polling from ESP incoming stream
    espPollRemoteFail();
    if (serialFailTriggered) {
      logLine("FAIL: remote command received.");
      finishDetected = 0;
      break;
    }
#endif

    if (button.isPressed() && buttonHeldFor(KILL_HOLD_MS)) {
#if USE_ESP8266
      logLine("KILL: button held.");
#else
      Serial.println(F("KILL: button held."));
#endif
      break;
    }

    unsigned long now = millis();
    if (now - startMs >= RUN_TIME_MS) {
#if USE_ESP8266
      logLine("STOP: time limit reached.");
#else
      Serial.println(F("STOP: time limit reached."));
#endif
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

    // Finish bar detect (all black)
    const unsigned int BLACK_THRESH = 800;
    if (allBlack(sensors, BLACK_THRESH)) {
      if (blackStart == 0) blackStart = now;
      if (now - blackStart >= FINISH_MS) {
#if USE_ESP8266
        logLine("STOP: finish line detected.");
#else
        Serial.println(F("STOP: finish line detected."));
#endif
        finishDetected = 1;
        break;
      }
    } else {
      blackStart = 0;
    }

    // --------------------
    // LINE LOST DETECTION
    // --------------------
    bool lost = allWhiteish(sensors, WHITE_MAX);
    if (lost) {
      if (lostSince == 0) lostSince = now;
      if (now - lostSince >= LOST_LINE_MS) {
        lineLostCount++;

        int dir = (lastError >= 0) ? 1 : -1;
        motors.setSpeeds(-dir * SEARCH_SPEED, dir * SEARCH_SPEED);

        if (now - lastTel >= TELEMETRY_MS) {
          lastTel = now;

#if USE_ESP8266
          // UDP "SEARCH" line
          char msg[260];
          int n = snprintf(msg, sizeof(msg),
            "SEARCH t=%lu lastErr=%d s=[%u,%u,%u,%u,%u,%u]\n",
            (unsigned long)(now - startMs), lastError,
            sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5]
          );
          if (n > 0 && n < (int)sizeof(msg)) espUdpSend(msg);
#else
          Serial.print(F("SEARCH t=")); Serial.print(now - startMs);
          Serial.print(F(" lastErr=")); Serial.print(lastError);
          Serial.print(F(" s=["));
          for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
          Serial.println(']');
#endif
        }
        continue;
      }
    } else {
      lostSince = 0;
    }

    // --------------------
    // CORNER STUCK ESCAPE
    // --------------------
    if (absError > 2000) {
      if (bigErrSince == 0) bigErrSince = now;
      if (now - bigErrSince >= STUCK_CORNER_MS) {
        int dir = (error >= 0) ? 1 : -1;
        motors.setSpeeds(-dir * ESCAPE_SPEED, dir * ESCAPE_SPEED);

        if (now - lastTel >= TELEMETRY_MS) {
          lastTel = now;

#if USE_ESP8266
          char msg[260];
          int n = snprintf(msg, sizeof(msg),
            "ESCAPE t=%lu err=%d s=[%u,%u,%u,%u,%u,%u]\n",
            (unsigned long)(now - startMs), error,
            sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5]
          );
          if (n > 0 && n < (int)sizeof(msg)) espUdpSend(msg);
#else
          Serial.print(F("ESCAPE t=")); Serial.print(now - startMs);
          Serial.print(F(" err=")); Serial.print(error);
          Serial.print(F(" s=["));
          for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
          Serial.println(']');
#endif
        }
        continue;
      }
    } else {
      bigErrSince = 0;
    }

    // --------------------
    // LOOSER SPEED SCHEDULING
    // --------------------
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

#if USE_ESP8266
      char msg[260];
      int n = snprintf(msg, sizeof(msg),
        "t=%lu pos=%d err=%d dErr=%d base=%d turn=%d L=%d R=%d s=[%u,%u,%u,%u,%u,%u]\n",
        (unsigned long)(now - startMs),
        position, error, dError, base, turn, left, right,
        sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5]
      );
      if (n > 0 && n < (int)sizeof(msg)) espUdpSend(msg);
#else
      Serial.print(F("t=")); Serial.print(now - startMs);
      Serial.print(F(" pos=")); Serial.print(position);
      Serial.print(F(" err=")); Serial.print(error);
      Serial.print(F(" dErr=")); Serial.print(dError);
      Serial.print(F(" base=")); Serial.print(base);
      Serial.print(F(" turn=")); Serial.print(turn);
      Serial.print(F(" L=")); Serial.print(left);
      Serial.print(F(" R=")); Serial.print(right);

      Serial.print(F(" s=["));
      for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
      Serial.println(']');
#endif
    }
  }

  stopMotors();

  // ----- metrics -----
  unsigned long runtime = millis() - startMs;
  float mad = (loops == 0) ? 0.0f : (float)absErrSum / (float)loops;
  float rms = (loops == 0) ? 0.0f : sqrt((float)errSqSum / (float)loops);
  float veerScore = (mad / 2500.0f) * 100.0f;
  (void)rms; // keep if you want it later

  // CSV line: kp,kd,base_speed,min_base_speed,corner1,corner2,corner3,brake_pwr,runtime_ms,veerScore,lineLost,finish
#if USE_ESP8266
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

  logLine("Press button to run again.");
#else
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

  Serial.println(F("Press button to run again."));
#endif

  button.waitForButton();
  delay(250);
}
