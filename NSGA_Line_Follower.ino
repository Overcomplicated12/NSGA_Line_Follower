/*
 * Zumo line following (LOOSER) + telemetry + stop conditions + recovery
 * - Looser control (less twitchy)
 * - Line-lost search to prevent "run away"
 * - Corner-stuck escape (pivot)
 * - CSV summary line for trials.csv
 */

#include <Wire.h>
#include <ZumoShield.h>

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
const int TURN_CAP = 350;                 // <-- looser: cap the turning strength
const unsigned long LOST_LINE_MS = 120;   // how long before we consider it "lost"
const unsigned long STUCK_CORNER_MS = 250;// big error for this long => escape
const int SEARCH_SPEED = 180;             // pivot speed during search
const int ESCAPE_SPEED = 240;             // stronger pivot escape
// For sensor "line lost" detection using calibrated readings:
const unsigned int WHITE_MAX = 200;       // if ALL sensors <= this => likely no black line
// -----------------------------------------

// ---------- SERIAL FAIL COMMAND ----------
char serialCmd[8];
uint8_t serialIdx = 0;
bool serialFailTriggered = false;
// ---------------------------------------


int lastError = 0;

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

void checkSerialFail() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Ignore line endings
    if (c == '\n' || c == '\r') {
      serialCmd[serialIdx] = '\0';
      if (strcmp(serialCmd, "FAIL") == 0) {
        serialFailTriggered = true;
      }
      serialIdx = 0;
      return;
    }

    // Accumulate command (protect buffer)
    if (serialIdx < sizeof(serialCmd) - 1) {
      serialCmd[serialIdx++] = c;
    }
  }
}

void loop() {
  // ---- RESET "FAIL" state for the next run ----
  serialFailTriggered = false;
  serialIdx = 0;
  serialCmd[0] = '\0';
  while (Serial.available() > 0) Serial.read(); // flush leftover input
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

  Serial.println(F("RUN: starting. Hold button 1s to KILL."));
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying()) {}

  while (true) {
    // Serial FAIL command
    checkSerialFail();
    if (serialFailTriggered) {
      Serial.println(F("FAIL: serial command received."));
      finishDetected = 0;   // mark as failed run
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

    // Finish bar detect (all black)
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

    // --------------------
    // LINE LOST DETECTION
    // --------------------
    bool lost = allWhiteish(sensors, WHITE_MAX); // "everything looks white"
    if (lost) {
      if (lostSince == 0) lostSince = now;
      if (now - lostSince >= LOST_LINE_MS) {
        lineLostCount++;

        // Search/pivot in direction of last known error
        // If lastError > 0, line was to the right -> turn right to find it.
        int dir = (lastError >= 0) ? 1 : -1;
        motors.setSpeeds(-dir * SEARCH_SPEED, dir * SEARCH_SPEED);

        // keep printing while searching
        if (now - lastTel >= TELEMETRY_MS) {
          lastTel = now;
          Serial.print(F("SEARCH t=")); Serial.print(now - startMs);
          Serial.print(F(" lastErr=")); Serial.print(lastError);
          Serial.print(F(" s=["));
          for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
          Serial.println(']');
        }
        continue; // skip normal control while lost
      }
    } else {
      lostSince = 0;
    }

    // --------------------
    // CORNER STUCK ESCAPE
    // --------------------
    // If error is huge for a while, force a pivot (helps corners)
    if (absError > 2000) {
      if (bigErrSince == 0) bigErrSince = now;
      if (now - bigErrSince >= STUCK_CORNER_MS) {
        int dir = (error >= 0) ? 1 : -1; // pivot toward the line
        motors.setSpeeds(-dir * ESCAPE_SPEED, dir * ESCAPE_SPEED);

        if (now - lastTel >= TELEMETRY_MS) {
          lastTel = now;
          Serial.print(F("ESCAPE t=")); Serial.print(now - startMs);
          Serial.print(F(" err=")); Serial.print(error);
          Serial.print(F(" s=["));
          for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
          Serial.println(']');
        }
        continue;
      }
    } else {
      bigErrSince = 0;
    }

    // --------------------
    // LOOSER SPEED SCHEDULING
    // (milder slowdown than your quadratic)
    // --------------------
    float e = absError / 2500.0f;
    if (e > 1.0f) e = 1.0f;

    // milder: linear-ish slowdown (looser)
    int base = (int)(BASE_SPEED - e * 0.55f * (BASE_SPEED - MIN_BASE_SPEED));
    base = clampInt(base, MIN_BASE_SPEED, BASE_SPEED);

    // still cap speed in corners, but less aggressively
    if (absError > CORNER_ERR1) base = min(base, 240);
    if (absError > CORNER_ERR2) base = min(base, 200);
    if (absError > CORNER_ERR3) base = min(base, 170);

    if (USE_BRAKE_PULSE && BRAKE_PWR > 0 && absError > BRAKE_ERR) {
      motors.setSpeeds(-BRAKE_PWR, -BRAKE_PWR);
      delay(BRAKE_MS);
    }

    // PD turn + TURN CAP (this makes it feel “looser”)
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

      Serial.print(F(" s=["));
      for (int i = 0; i < 6; i++) { Serial.print(sensors[i]); if (i < 5) Serial.print(','); }
      Serial.println(']');
    }
  }

  stopMotors();

  // ----- metrics -----
  unsigned long runtime = millis() - startMs;
  float mad = (loops == 0) ? 0.0f : (float)absErrSum / (float)loops;
  float rms = (loops == 0) ? 0.0f : sqrt((float)errSqSum / (float)loops);
  float veerScore = (mad / 2500.0f) * 100.0f;

  // CSV line: kp,kd,base_speed,min_base_speed,corner1,corner2,corner3,brake_pwr,runtime_ms,veerScore,lineLost,finish
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
  button.waitForButton();
  delay(250);
}
