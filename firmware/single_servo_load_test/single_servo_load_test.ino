#include <Servo.h>
#include <math.h>

// Only for an Arduino UNO R3 and ONE position-type MG90S on D3.
// These are commanded angle units, NOT measured shaft angle or tongue mm.
const byte SERVO_PIN = 3;
const byte JOYSTICK_X_PIN = A0;
const byte JOYSTICK_Y_PIN = A1;
const byte BUTTON_PIN = 2;
const float NEUTRAL_DEG = 90.0f;
const float MAX_PULL_DEG = 3.0f;
const int PULL_DIRECTION = 1;       // Change to -1 ONLY after an unloaded test.
const bool INVERT_Y = true;
const float DEADZONE = 0.10f;
const float STEP_PER_FRAME_DEG = 0.10f;
const unsigned long FRAME_MS = 20;
const unsigned long MAX_PULL_TIME_MS = 5000;

Servo motor;
int centerX = 512, centerY = 512;
bool calibrationOk = false;
bool armed = false;
bool joystickMode = false;
float currentPull = 0.0f, targetPull = 0.0f;
unsigned long lastFrameMs = 0, lastReportMs = 0, pullStartedMs = 0;
bool pullTimerRunning = false;
bool buttonWasDown = false;

float clampValue(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float readAxis(byte pin, int center) {
  int raw = analogRead(pin);
  float value = raw >= center
    ? float(raw - center) / float(max(1, 1023 - center))
    : float(raw - center) / float(max(1, center));
  value = clampValue(value, -1.0f, 1.0f);
  float magnitude = fabs(value);
  if (magnitude <= DEADZONE) return 0.0f;
  magnitude = (magnitude - DEADZONE) / (1.0f - DEADZONE);
  return value < 0 ? -magnitude : magnitude;
}

bool stickCentered() {
  return readAxis(JOYSTICK_X_PIN, centerX) == 0.0f
      && readAxis(JOYSTICK_Y_PIN, centerY) == 0.0f;
}

void releaseAndLock(const __FlashStringHelper* reason) {
  targetPull = 0.0f;
  armed = false;
  joystickMode = false;
  Serial.println(reason);
}

void printStatus() {
  Serial.print(!armed ? F("LOCKED") : joystickMode ? F("JOY") : F("STEP"));
  Serial.print(F(",pullCommand=")); Serial.print(currentPull, 2);
  Serial.print(F(",targetCommand=")); Serial.print(targetPull, 2);
  Serial.print(F(",shaftCommand="));
  Serial.print(NEUTRAL_DEG + PULL_DIRECTION * currentPull, 2);
  Serial.print(F(",X=")); Serial.print(readAxis(JOYSTICK_X_PIN, centerX), 2);
  float y = readAxis(JOYSTICK_Y_PIN, centerY);
  Serial.print(F(",Y=")); Serial.println(INVERT_Y ? -y : y, 2);
}

void printHelp() {
  Serial.println(F("Single-servo LOAD TEST: D3 only; no AUTO; boot LOCKED."));
  Serial.print(F("MAX_PULL_DEG=")); Serial.println(MAX_PULL_DEG, 1);
  Serial.println(F("e=enable at neutral; +=pull 1 command deg; -=release 1"));
  Serial.println(F("0=release+lock; j=joystick at neutral; m=step/hold; p=status"));
  Serial.println(F("SW button=release+lock. Continuous pull >=5s: release+lock."));
  Serial.println(F("Status is COMMANDS, NOT position/force feedback."));
  Serial.println(F("Power OFF for wiring, reset/upload, horn or cable changes."));
}

void handleSerial(char command) {
  if (command == '\r' || command == '\n' || command == ' ') return;
  if (command == '0') {
    releaseAndLock(F("Release requested; returning to neutral, then stay LOCKED."));
    return;
  }
  if (command == 'p') { printStatus(); return; }
  if (command == '?') { printHelp(); return; }
  if (command == 'e') {
    if (!calibrationOk || !stickCentered() || digitalRead(BUTTON_PIN) == LOW
        || currentPull > 0.05f || targetPull > 0.05f) {
      Serial.println(F("Cannot enable: center stick, release SW, wait for neutral; check calibration."));
      return;
    }
    armed = true;
    joystickMode = false;
    Serial.println(F("STEP enabled. Start with ONE +, then send 0 to release."));
    return;
  }
  if (!armed) {
    Serial.println(F("LOCKED: send e only after checking the mechanism."));
    return;
  }
  if (command == 'm') {
    joystickMode = false;
    targetPull = currentPull;
    Serial.println(F("STEP mode: holding present command; 0 releases."));
  } else if (command == 'j') {
    if (!stickCentered() || currentPull > 0.05f || targetPull > 0.05f) {
      Serial.println(F("First return to neutral with 0, then e, then j."));
      return;
    }
    joystickMode = true;
    Serial.println(F("JOY mode: only positive Y pulls; centered/negative Y releases."));
  } else if (command == '+' || command == '-') {
    if (joystickMode) {
      Serial.println(F("Send m for STEP mode, or 0 to release and lock."));
      return;
    }
    targetPull = clampValue(targetPull + (command == '+' ? 1.0f : -1.0f),
                            0.0f, MAX_PULL_DEG);
  } else {
    Serial.println(F("Unknown command. Send ? for help."));
  }
}

void writePullCommand() {
  // Match Servo.write(angle)'s default pulse mapping, but retain sub-degree
  // command increments instead of rounding every update to a whole degree.
  float angle = NEUTRAL_DEG + PULL_DIRECTION * currentPull;
  int pulseUs = int(MIN_PULSE_WIDTH
    + angle * float(MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180.0f + 0.5f);
  motor.writeMicroseconds(pulseUs);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  motor.write(int(NEUTRAL_DEG));  // Set command before enabling D3 pulses.
  motor.attach(SERVO_PIN);       // D5/D9/D10 are deliberately not attached.
  writePullCommand();
  Serial.println(F("Keep joystick centered for 2 seconds. Motor commanded neutral."));
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 200; ++i) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    delay(10);
  }
  centerX = sumX / 200;
  centerY = sumY / 200;
  calibrationOk = centerX >= 150 && centerX <= 875
               && centerY >= 150 && centerY <= 875;
  if (!calibrationOk) {
    Serial.println(F("Calibration implausible: power OFF, check joystick wires, then restart."));
  }
  printHelp();
}

void loop() {
  bool buttonDown = digitalRead(BUTTON_PIN) == LOW;
  if (buttonDown && (!buttonWasDown || armed)) {
    releaseAndLock(F("SW pressed: release+lock. This is NOT a hardware emergency stop."));
  }
  buttonWasDown = buttonDown;
  while (Serial.available() > 0) handleSerial(char(Serial.read()));

  if (!armed) {
    targetPull = 0.0f;
  } else if (joystickMode) {
    float y = readAxis(JOYSTICK_Y_PIN, centerY);
    if (INVERT_Y) y = -y;
    targetPull = max(0.0f, y) * MAX_PULL_DEG;
  }

  unsigned long now = millis();
  if (targetPull > 0.05f || currentPull > 0.05f) {
    if (!pullTimerRunning) {
      pullStartedMs = now;
      pullTimerRunning = true;
    }
    if (armed && now - pullStartedMs >= MAX_PULL_TIME_MS) {
      releaseAndLock(F("5-second pull limit: returning to neutral and LOCKED. Center stick, then e."));
    }
  } else {
    pullTimerRunning = false;
  }

  if (now - lastFrameMs >= FRAME_MS) {
    lastFrameMs = now;
    float delta = clampValue(targetPull - currentPull,
                             -STEP_PER_FRAME_DEG, STEP_PER_FRAME_DEG);
    currentPull = clampValue(currentPull + delta, 0.0f, MAX_PULL_DEG);
    writePullCommand();
  }
  if (now - lastReportMs >= 200) {
    lastReportMs = now;
    printStatus();
  }
}
