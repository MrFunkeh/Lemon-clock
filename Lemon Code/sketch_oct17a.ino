#include <Wire.h>
#include <RTClib.h>

// ============================================================
// ESP32-C3 CLOCK
// DS1307 + rotary encoder + 3-digit 74HC595 display
// 28BYJ-48 + ULN2003 + zero-position switch
// ============================================================

// -------------------- GPIO --------------------
constexpr uint8_t PIN_SR_DATA  = 6;
constexpr uint8_t PIN_SR_CLOCK = 4;
constexpr uint8_t PIN_SR_LATCH = 5;

constexpr uint8_t PIN_SDA = 8;
constexpr uint8_t PIN_SCL = 9;

constexpr uint8_t PIN_ENC_A  = 2;
constexpr uint8_t PIN_ENC_B  = 3;
constexpr uint8_t PIN_ENC_SW = 10;

constexpr uint8_t PIN_ZERO = 1;

constexpr uint8_t MOTOR_PINS[4] = {18, 19, 20, 21};

// -------------------- DISPLAY CONFIG --------------------
// Assumes two cascaded 74HC595s:
// first shifted byte ends up at the SECOND register.
// Change these masks to suit your wiring.
//
// Segment bits: bit 0=a, 1=b, 2=c, 3=d,
//               4=e, 5=f, 6=g, 7=dp
//
// Common-cathode segment outputs: HIGH = ON.
constexpr bool SEGMENTS_ACTIVE_LOW = false;

// Digit-select register: bit 0,1,2 enable digits 0,1,2.
// Assumes HIGH enables a digit driver.
// Change to true if your digit drivers are active LOW.
constexpr bool DIGITS_ACTIVE_LOW = false;

constexpr uint8_t DIGIT_MASKS[3] = {
  0b00000001,  // left digit
  0b00000010,  // middle digit
  0b00000100   // right digit
};

// Segment lookup table: 0-9, blank, minus.
// Bit order: dp g f e d c b a would require a different table;
// this table assumes bit 0=a through bit 6=g.
const uint8_t SEGMENT_MAP[] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00000000, // blank
  0b01000000  // minus
};

constexpr uint8_t CHAR_BLANK = 10;

// -------------------- MOTOR CONFIG --------------------
// Typical 28BYJ-48 half-step count is approximately 4096
// half-steps per output revolution. Actual gearboxes vary.
//
// Calibrate this value so the hand completes exactly one
// revolution after 60 minute ticks.
constexpr long HALF_STEPS_PER_REV = 4096;
constexpr long STEPS_PER_MINUTE_NUM = HALF_STEPS_PER_REV;
constexpr long MINUTES_PER_REV = 60;

// Half-step delay. Increase if the motor misses steps.
constexpr uint32_t MOTOR_STEP_INTERVAL_US = 1800;

// Homing travel limit prevents an endless search if the
// switch is disconnected or never activates.
constexpr long HOMING_MAX_STEPS = HALF_STEPS_PER_REV + 512;

// Normally open switch connected between GPIO and GND.
constexpr bool ZERO_SWITCH_ACTIVE_LOW = true;

// -------------------- TIME SETTING --------------------
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t LONG_PRESS_MS = 900;
constexpr uint32_t DISPLAY_SWITCH_MS = 1500;

RTC_DS1307 rtc;

enum ClockState {
  HOMING,
  RUNNING,
  SETTING,
  HOME_ERROR
};

ClockState state = HOMING;

int selectedHour = 0;
int selectedMinute = 0;
bool editingMinutes = true;

int lastRtcMinuteOfDay = -1;
int displayedMinuteOfDay = -1;

// Motor position is measured in half-steps from home.
// Position 0 = zero switch / minute 00.
long motorPosition = 0;
long motorTarget = 0;
long homeStepCount = 0;

bool motorBusy = false;
bool homingStarted = false;
bool zeroWasActive = false;

uint32_t lastMotorStepUs = 0;
uint32_t lastDisplayChangeMs = 0;
uint32_t lastDisplayScanUs = 0;

int displayDigits[3] = {CHAR_BLANK, 0, 0};
uint8_t displayScanDigit = 0;

// Encoder state
int lastEncoderA = HIGH;
int encoderAccumulator = 0;

// Button state
bool buttonStable = HIGH;
bool buttonLastReading = HIGH;
uint32_t buttonLastChangeMs = 0;
uint32_t buttonDownMs = 0;
bool longPressHandled = false;

// 28BYJ-48 eight-state half-step sequence.
// Each entry maps to IN1, IN2, IN3, IN4.
const uint8_t HALF_STEP_SEQUENCE[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

int motorPhase = 0;

// ============================================================
// DISPLAY
// ============================================================

uint8_t segmentByte(int digit) {
  if (digit < 0 || digit > 9) {
    return SEGMENTS_ACTIVE_LOW ? 0xFF : 0x00;
  }

  uint8_t value = SEGMENT_MAP[digit];
  return SEGMENTS_ACTIVE_LOW ? ~value : value;
}

uint8_t digitSelectByte(uint8_t digit) {
  uint8_t value = DIGIT_MASKS[digit];
  return DIGITS_ACTIVE_LOW ? ~value : value;
}

void writeShiftRegisters(uint8_t segments, uint8_t digits) {
  digitalWrite(PIN_SR_LATCH, LOW);

  // Two chained 74HC595s:
  // first byte shifts to the register farthest from the ESP32.
  shiftOut(PIN_SR_DATA, PIN_SR_CLOCK, MSBFIRST, digits);
  shiftOut(PIN_SR_DATA, PIN_SR_CLOCK, MSBFIRST, segments);

  digitalWrite(PIN_SR_LATCH, HIGH);
}

void displayScan() {
  uint32_t now = micros();

  // Multiplex each digit rapidly.
  if ((uint32_t)(now - lastDisplayScanUs) < 2000) return;
  lastDisplayScanUs = now;

  // Blank briefly between digit changes.
  writeShiftRegisters(
    segmentByte(CHAR_BLANK),
    DIGITS_ACTIVE_LOW ? 0xFF : 0x00
  );

  uint8_t segments = segmentByte(displayDigits[displayScanDigit]);
  uint8_t digitMask = digitSelectByte(displayScanDigit);

  writeShiftRegisters(segments, digitMask);

  displayScanDigit = (displayScanDigit + 1) % 3;
}

void setDisplayNumber(int number) {
  number %= 1000;
  if (number < 0) number += 1000;

  displayDigits[0] = number / 100;
  displayDigits[1] = (number / 10) % 10;
  displayDigits[2] = number % 10;
}

void setDisplayTwoDigits(int number) {
  number %= 100;
  if (number < 0) number += 100;

  // Right-align two digits on the three-digit display.
  displayDigits[0] = CHAR_BLANK;
  displayDigits[1] = number / 10;
  displayDigits[2] = number % 10;
}

void updateDisplay() {
  uint32_t now = millis();

  if (state == HOME_ERROR) {
    // Display E-1 using E, minus, 1.
    displayDigits[0] = 11;
    displayDigits[1] = 11;
    displayDigits[2] = 1;
    return;
  }

  if (state == SETTING) {
    int value = editingMinutes ? selectedMinute : selectedHour;
    setDisplayTwoDigits(value);

    // Blink the selected field by blanking the display.
    if ((now / 300) % 2 == 0) {
      displayDigits[0] = CHAR_BLANK;
    }
    return;
  }

  if (now - lastDisplayChangeMs >= DISPLAY_SWITCH_MS) {
    lastDisplayChangeMs = now;
    displayedMinuteOfDay = (displayedMinuteOfDay + 1) % 2;
  }

  DateTime current = rtc.now();

  if (displayedMinuteOfDay == 0) {
    setDisplayTwoDigits(current.hour());
  } else {
    setDisplayTwoDigits(current.minute());
  }
}

// ============================================================
// ZERO SWITCH AND MOTOR
// ============================================================

bool zeroSwitchActive() {
  int reading = digitalRead(PIN_ZERO);
  return ZERO_SWITCH_ACTIVE_LOW ? reading == LOW : reading == HIGH;
}

void setMotorPhase(int phase) {
  phase = (phase + 8) % 8;
  motorPhase = phase;

  for (int i = 0; i < 4; i++) {
    digitalWrite(MOTOR_PINS[i], HALF_STEP_SEQUENCE[phase][i]);
  }
}

void releaseMotor() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(MOTOR_PINS[i], LOW);
  }
}

void startHoming() {
  state = HOMING;
  motorBusy = true;
  homingStarted = false;
  homeStepCount = 0;
  motorTarget = 0;
  zeroWasActive = zeroSwitchActive();
}

void failHoming() {
  motorBusy = false;
  state = HOME_ERROR;
  releaseMotor();
}

void startMoveTo(long target) {
  motorTarget = target;
  motorBusy = motorPosition != motorTarget;
}

void serviceMotor() {
  if (!motorBusy) return;

  uint32_t now = micros();
  if ((uint32_t)(now - lastMotorStepUs) < MOTOR_STEP_INTERVAL_US) {
    return;
  }
  lastMotorStepUs = now;

  if (state == HOMING) {
    bool active = zeroSwitchActive();

    // If the switch is already active, move away until it
    // releases, then approach it again to find a repeatable edge.
    if (!homingStarted && active) {
      setMotorPhase(motorPhase + 1);
      homeStepCount++;

      if (!zeroSwitchActive()) {
        homingStarted = true;
        homeStepCount = 0;
      }

      if (homeStepCount > HOMING_MAX_STEPS) {
        failHoming();
      }
      return;
    }

    // Search toward the switch in the reverse direction.
    setMotorPhase(motorPhase - 1);
    homeStepCount++;

    if (zeroSwitchActive()) {
      motorBusy = false;
      motorPosition = 0;
      motorTarget = 0;
      state = RUNNING;
      releaseMotor();

      DateTime nowTime = rtc.now();
      lastRtcMinuteOfDay = nowTime.hour() * 60 + nowTime.minute();

      // The minute hand repeats every 60 minutes.
      long target = (long)nowTime.minute() *
                    HALF_STEPS_PER_REV / MINUTES_PER_REV;
      startMoveTo(target);
    } else if (homeStepCount > HOMING_MAX_STEPS) {
      failHoming();
    }

    return;
  }

  // Normal movement: either direction is allowed.
  if (motorPosition < motorTarget) {
    setMotorPhase(motorPhase + 1);
    motorPosition++;
  } else if (motorPosition > motorTarget) {
    setMotorPhase(motorPhase - 1);
    motorPosition--;
  }

  if (motorPosition == motorTarget) {
    motorBusy = false;
    releaseMotor();
  }
}

long minuteToSteps(int minute) {
  return ((long)minute * HALF_STEPS_PER_REV) / MINUTES_PER_REV;
}

// ============================================================
// ENCODER
// ============================================================

void serviceEncoder() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);

  if (a != lastEncoderA && a == LOW) {
    int direction = (b != a) ? 1 : -1;
    encoderAccumulator += direction;

    // Most mechanical encoders produce 2 or 4 transitions per detent.
    if (abs(encoderAccumulator) >= 4) {
      int detents = encoderAccumulator / 4;
      encoderAccumulator %= 4;

      if (state == SETTING) {
        if (editingMinutes) {
          selectedMinute = (selectedMinute + detents) % 60;
          if (selectedMinute < 0) selectedMinute += 60;
        } else {
          selectedHour = (selectedHour + detents) % 24;
          if (selectedHour < 0) selectedHour += 24;
        }
      }
    }
  }

  lastEncoderA = a;
}

void saveSelectedTime() {
  DateTime oldTime = rtc.now();

  // Preserve the current date while replacing hour/minute.
  DateTime newTime(
    oldTime.year(), oldTime.month(), oldTime.day(),
    selectedHour, selectedMinute, 0
  );

  rtc.adjust(newTime);

  lastRtcMinuteOfDay = selectedHour * 60 + selectedMinute;
  lastDisplayChangeMs = millis();
  displayedMinuteOfDay = 0;

  state = RUNNING;

  // Move the hand to the selected minute position.
  // This can move either forward or backward.
  startMoveTo(minuteToSteps(selectedMinute));
}

void serviceButton() {
  uint32_t now = millis();
  bool reading = digitalRead(PIN_ENC_SW);

  if (reading != buttonLastReading) {
    buttonLastChangeMs = now;
    buttonLastReading = reading;
  }

  if (now - buttonLastChangeMs < BUTTON_DEBOUNCE_MS) return;

  if (reading != buttonStable) {
    buttonStable = reading;

    if (buttonStable == LOW) {
      buttonDownMs = now;
      longPressHandled = false;
    } else {
      // Short press switches between hour and minute selection.
      if (!longPressHandled && state == SETTING) {
        editingMinutes = !editingMinutes;
      } else if (!longPressHandled && state == RUNNING) {
        DateTime t = rtc.now();
        selectedHour = t.hour();
        selectedMinute = t.minute();
        editingMinutes = true;
        state = SETTING;
      }
    }
  }

  if (buttonStable == LOW && !longPressHandled &&
      now - buttonDownMs >= LONG_PRESS_MS) {
    longPressHandled = true;

    if (state == SETTING) {
      saveSelectedTime();
    }
  }
}

// ============================================================
// RTC AND CLOCK SYNCHRONIZATION
// ============================================================

void serviceClock() {
  if (state != RUNNING) return;

  DateTime t = rtc.now();
  int minuteOfDay = t.hour() * 60 + t.minute();

  if (minuteOfDay != lastRtcMinuteOfDay) {
    lastRtcMinuteOfDay = minuteOfDay;

    // Move to the current minute position on the dial.
    // The minute hand makes one revolution per hour.
    startMoveTo(minuteToSteps(t.minute()));
  }
}

// ============================================================
// SETUP AND LOOP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SR_DATA, OUTPUT);
  pinMode(PIN_SR_CLOCK, OUTPUT);
  pinMode(PIN_SR_LATCH, OUTPUT);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_ZERO, INPUT_PULLUP);

  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    digitalWrite(MOTOR_PINS[i], LOW);
  }

  // Initialize display to blank.
  writeShiftRegisters(
    segmentByte(CHAR_BLANK),
    DIGITS_ACTIVE_LOW ? 0xFF : 0x00
  );

  Wire.begin(PIN_SDA, PIN_SCL);

  if (!rtc.begin()) {
    Serial.println("ERROR: DS1307 not found.");
    state = HOME_ERROR;
    return;
  }

  if (!rtc.isrunning()) {
    // Set to the compile time only if the RTC oscillator stopped.
    // Replace with your preferred initial time if necessary.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();
  Serial.printf("RTC time: %02d:%02d:%02d\n",
                now.hour(), now.minute(), now.second());

  lastEncoderA = digitalRead(PIN_ENC_A);
  lastDisplayChangeMs = millis();
  displayedMinuteOfDay = 0;

  startHoming();
}

void loop() {
  // Keep multiplexed display scanning rapidly.
  displayScan();

  serviceEncoder();
  serviceButton();
  serviceMotor();
  serviceClock();
  updateDisplay();

  // Avoid long delay() calls so the display stays refreshed.
}
