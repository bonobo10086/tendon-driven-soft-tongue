#include <Servo.h>

// Arduino UNO R3. Only the joystick takes power from UNO 5V.
// Servos use a separate supply and share its GND with UNO.
enum Channel : uint8_t { UP, DOWN, LEFT, RIGHT, CHANNELS };
constexpr uint8_t PINS[CHANNELS] = {3, 5, 9, 10};
constexpr uint8_t X_PIN = A0, Y_PIN = A1, BUTTON_PIN = 2;

// 下列数组都按「上、下、左、右」排列。数值是命令角度，不是实测角度。
constexpr int NEUTRAL[CHANNELS] = {90, 90, 90, 90};
constexpr int8_t PULL_SIGN[CHANNELS] = {+1, +1, +1, +1};
constexpr float PULL_DEG[CHANNELS] = {3, 3, 3, 3};
constexpr float RELEASE_DEG[CHANNELS] = {3, 3, 3, 3};
constexpr bool INVERT_X = false, INVERT_Y = true;

constexpr uint32_t FRAME_MS = 20, DEBOUNCE_MS = 30;
constexpr uint32_t ARM_HOLD_MS = 800, SETTLE_MS = 600, HOLD_LIMIT_MS = 5000;
constexpr float DEAD_ZONE = 0.10f, AXIS_STEP = 0.075f;

enum Mode : uint8_t { LOCKED, SETTLING, ACTIVE, RETURNING, FAULT };
Servo motor[CHANNELS];
Mode mode = LOCKED;
int centerX = 512, centerY = 512;
int rawX = 512, rawY = 512;
float currentX = 0, currentY = 0;
int commanded[CHANNELS] = {0, 0, 0, 0};
bool pulsesStarted = false;
bool lastButton = HIGH, stableButton = HIGH, armCandidate = false;
bool holding = false;
uint32_t buttonChanged = 0, armSince = 0, settledSince = 0;
uint32_t holdSince = 0, lastFrame = 0;

bool validSettings() {
  for (uint8_t i = 0; i < CHANNELS; ++i) {
    const float travel = max(PULL_DEG[i], RELEASE_DEG[i]);
    if ((PULL_SIGN[i] != 1 && PULL_SIGN[i] != -1) ||
        !(PULL_DEG[i] >= 0 && PULL_DEG[i] <= 20) ||
        !(RELEASE_DEG[i] >= 0 && RELEASE_DEG[i] <= 20) ||
        NEUTRAL[i] - travel < 45 || NEUTRAL[i] + travel > 135) return false;
  }
  return true;
}

bool readStartupCenter() {
  constexpr int N = 64;
  long totalX = 0, totalY = 0;
  int lowX = 1023, highX = 0, lowY = 1023, highY = 0;
  for (int i = 0; i < N; ++i) {
    const int x = analogRead(X_PIN), y = analogRead(Y_PIN);
    totalX += x; totalY += y;
    lowX = min(lowX, x); highX = max(highX, x);
    lowY = min(lowY, y); highY = max(highY, y);
    delay(10);
  }
  centerX = totalX / N;
  centerY = totalY / N;
  return centerX >= 330 && centerX <= 690 && centerY >= 330 && centerY <= 690 &&
         highX - lowX <= 32 && highY - lowY <= 32;
}

float axisValue(int raw, int center, bool invert) {
  const float span = max(1, raw >= center ? (1023 - center) : center);
  const float value = (raw - center) / span * (invert ? -1.0f : 1.0f);
  const float magnitude = fabs(value);
  if (magnitude <= DEAD_ZONE) return 0;
  return (value < 0 ? -1.0f : 1.0f) * (magnitude - DEAD_ZONE) / (1.0f - DEAD_ZONE);
}

float approach(float oldValue, float target) {
  if ((oldValue > 0 && target < 0) || (oldValue < 0 && target > 0)) target = 0;
  const float delta = target - oldValue;
  if (delta > AXIS_STEP) return oldValue + AXIS_STEP;
  if (delta < -AXIS_STEP) return oldValue - AXIS_STEP;
  return target;
}

void writeAxes(float x, float y) {
  // Positive = take up that cable; negative = let it out. Opposite channels
  // share one smoothed axis, so reversal passes through neutral together.
  const float action[CHANNELS] = {y, -y, -x, x};
  for (uint8_t i = 0; i < CHANNELS; ++i) {
    const float range = action[i] >= 0 ? PULL_DEG[i] : RELEASE_DEG[i];
    const float angle = NEUTRAL[i] + PULL_SIGN[i] * range * action[i];
    const int rounded = int(angle + 0.5f);
    if (rounded != commanded[i]) {
      motor[i].write(rounded);
      commanded[i] = rounded;
    }
  }
}

void beginArming(uint32_t now) {
  currentX = currentY = 0;
  if (!pulsesStarted) {
    for (uint8_t i = 0; i < CHANNELS; ++i) {
      // Seed the neutral pulse BEFORE attaching on the UNO AVR Servo library.
      motor[i].write(NEUTRAL[i]);
      commanded[i] = NEUTRAL[i];
      motor[i].attach(PINS[i]);
    }
    pulsesStarted = true;
  }
  mode = SETTLING;
  settledSince = now;
  Serial.println(F("SETTLING: keep the stick centered"));
}

void returnAndLock() {
  armCandidate = holding = false;
  mode = RETURNING;
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("RETURNING: neutral then lock; motor power is NOT cut"));
}

void updateButton(uint32_t now, bool centered) {
  const bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButton) {
    lastButton = reading;
    buttonChanged = now;
  }
  if (!centered) armCandidate = false;
  if (reading == stableButton || now - buttonChanged < DEBOUNCE_MS) return;
  stableButton = reading;
  if (stableButton == LOW) {
    if (mode == ACTIVE || mode == SETTLING) returnAndLock();
    else if (mode == LOCKED && centered) {
      armCandidate = true;
      armSince = now;
    }
  } else {
    const bool arm = armCandidate && mode == LOCKED && centered && now - armSince >= ARM_HOLD_MS;
    armCandidate = false;
    if (arm) beginArming(now);
  }
}

void printStatus() {
  Serial.print(F("state="));
  switch (mode) {
    case LOCKED: Serial.print(F("LOCKED")); break;
    case SETTLING: Serial.print(F("SETTLING")); break;
    case ACTIVE: Serial.print(F("ACTIVE")); break;
    case RETURNING: Serial.print(F("RETURNING")); break;
    case FAULT: Serial.print(F("FAULT")); break;
  }
  Serial.print(F(" rawXY=")); Serial.print(rawX); Serial.print('/'); Serial.print(rawY);
  Serial.print(F(" centerXY=")); Serial.print(centerX); Serial.print('/'); Serial.print(centerY);
  Serial.print(F(" pulses=")); Serial.print(pulsesStarted ? F("ON") : F("OFF"));
  Serial.print(F(" cmd(U/D/L/R)="));
  for (uint8_t i = 0; i < CHANNELS; ++i) {
    if (i) Serial.print('/');
    Serial.print(commanded[i]);
  }
  Serial.println();
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  for (uint8_t i = 0; i < CHANNELS; ++i) {
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], LOW);
    commanded[i] = NEUTRAL[i];
  }
  Serial.begin(115200);
  Serial.println(F("Keep joystick centered during boot"));
  if (!validSettings() || !readStartupCenter()) {
    mode = FAULT;
    Serial.println(F("FAULT: settings or joystick center; check wiring, center stick and reset"));
  } else {
    Serial.println(F("LOCKED: center stick, hold its button 1 s, then release"));
  }
  // Holding the button during reset must not count as an arm request.
  lastButton = stableButton = digitalRead(BUTTON_PIN);
  buttonChanged = lastFrame = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastFrame < FRAME_MS) return;
  lastFrame = now;
  rawX = analogRead(X_PIN);
  rawY = analogRead(Y_PIN);
  const float x = axisValue(rawX, centerX, INVERT_X);
  const float y = axisValue(rawY, centerY, INVERT_Y);
  const bool centered = x == 0 && y == 0;

  if (mode != FAULT) updateButton(now, centered);
  if (mode == SETTLING && now - settledSince >= SETTLE_MS && centered) {
    mode = ACTIVE;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(F("ACTIVE"));
  }
  if (mode == ACTIVE) {
    if (centered) holding = false;
    else if (!holding) { holding = true; holdSince = now; }
    else if (now - holdSince >= HOLD_LIMIT_MS) returnAndLock();
  }
  if (mode == ACTIVE || mode == RETURNING) {
    currentX = approach(currentX, mode == ACTIVE ? x : 0);
    currentY = approach(currentY, mode == ACTIVE ? y : 0);
    writeAxes(currentX, currentY);
    if (mode == RETURNING && currentX == 0 && currentY == 0) {
      mode = LOCKED;
      Serial.println(F("LOCKED: still holding neutral, cut motor power before touching the mechanism"));
    }
  }
  // Limit diagnostic work to a single byte per frame. No serial motion command.
  if (Serial.available() > 0 && Serial.read() == 'p') printStatus();
}
