/*
  DIY Sim Racing Control Box Firmware
  Board: Arduino Pro Micro / Arduino Micro / ATmega32U4
  USB HID: stable 32-button joystick + 1 analog handbrake axis
  Joystick library: MHeironimus ArduinoJoystickLibrary

  Design notes:
  - USB identity is stable. Profiles only remap behavior/button numbers; they do NOT
    create different USB devices.
  - Matrix inputs are represented as named logical controls first, then mapped to HID.
  - Encoder rotation creates short joystick button pulses, never held states.
  - Selected profile is stored in EEPROM only when it changes.

  RGB LED type:
  - Set RGB_COMMON_ANODE below. false = common cathode, true = common anode.
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <Joystick.h>

// -----------------------------------------------------------------------------
// User-editable hardware configuration
// -----------------------------------------------------------------------------

// If your RGB LED common pin goes to GND, use false. If it goes to +5V, use true.
const bool RGB_COMMON_ANODE = false;

const uint8_t PIN_HANDBRAKE = A3;

// 4x5 diode-protected button matrix using Arduino digital pins 0..8.
// Suggested wiring with your new plan:
//   Rows:    0, 1, 2, 3
//   Columns: 4, 5, 6, 7, 8
// Scanning assumption: rows are driven LOW one-at-a-time, columns use INPUT_PULLUP.
// With a pressed switch, the active column reads LOW. This wiring expects diode current
// to flow from column pullup -> switch/diode -> driven-low row. If your diode direction
// is opposite, either reverse the scan logic or rewire diodes consistently.
// Note: pins 0/1 are also Serial RX/TX. That is OK for USB HID firmware, but avoid
// using Serial debugging while they are part of the matrix.
const uint8_t MATRIX_ROWS = 4;
const uint8_t MATRIX_COLS = 5;
const uint8_t rowPins[MATRIX_ROWS] = {3, 2, 1, 0};
const uint8_t colPins[MATRIX_COLS] = {4, 5, 6, 7, 8};

// Direct rotary encoder pins. Encoder push switches remain in the matrix.
const uint8_t ENC1_CLK_PIN = 15;
const uint8_t ENC1_DT_PIN  = 14;
const uint8_t ENC2_CLK_PIN = 16;
const uint8_t ENC2_DT_PIN  = 10;

// Remaining practical Pro Micro pins after your new wiring: 9, A0, A1, A2.
// A non-addressable RGB LED needs 3 output pins, so connect:
//   RGB R -> A0 through a resistor
//   RGB G -> A1 through a resistor
//   RGB B -> A2 through a resistor
// These analog pins work as digital outputs; on most Pro Micro boards they are NOT PWM,
// so profile colors are reliable on/off color mixes, not smooth dimmed colors.
const uint8_t RGB_R_PIN = A0;
const uint8_t RGB_G_PIN = A1;
const uint8_t RGB_B_PIN = A2;

// Only one fully spare GPIO remains for single-color LEDs with this wiring.
// Connect LED 1 anode -> pin 9 through a resistor, cathode -> GND.
// LED 2/3 need more hardware/pins: use a shift register/I2C expander, or free pins by
// moving matrix/encoder/RGB wiring. Leave them disabled for v1.
const int8_t SINGLE_LED_PINS[3] = {9, -1, -1};

// Handbrake calibration. Adjust after reading raw A3 values if needed.
const int HANDBRAKE_RAW_MIN = 0;
const int HANDBRAKE_RAW_MAX = 1023;
const bool HANDBRAKE_INVERT = false;
const uint8_t HANDBRAKE_SMOOTHING_SHIFT = 2; // 0=no smoothing, 2=light 1/4 IIR smoothing.

// Timing constants.
const uint16_t MATRIX_DEBOUNCE_MS = 25;
const uint16_t HID_PULSE_MS = 45;
const uint16_t PROFILE_HOLD_MS = 2000;
const uint16_t PROFILE_CHANGE_LOCKOUT_MS = 1200;
const uint16_t LOOP_DELAY_MS = 2;

// HID layout constants.
const uint8_t HID_BUTTON_COUNT = 32;
const int HID_AXIS_MIN = 0;
const int HID_AXIS_MAX = 1023;

// -----------------------------------------------------------------------------
// Logical controls: physical inputs are named here before any HID mapping happens.
// Edit matrixControlMap below if your wiring changes.
// -----------------------------------------------------------------------------

enum LogicalControl : uint8_t {
  LC_BTN_1,
  LC_BTN_2,
  LC_BTN_3,
  LC_BTN_4,
  LC_BTN_5,
  LC_BTN_6,
  LC_BTN_7,

  LC_NORMAL_SWITCH,
  LC_NORMAL_SWITCH_ALT, // Optional second matrix position if your 2-state switch uses two throws.

  LC_LATCH1_UP,
  LC_LATCH1_DOWN,
  LC_LATCH2_UP,
  LC_LATCH2_DOWN,

  LC_MAIN_SWITCH,

  LC_MOM_UP,
  LC_MOM_DOWN,

  LC_ENC1_SW,
  LC_ENC2_SW,

  LC_UNUSED_18,
  LC_UNUSED_19,

  LC_COUNT
};

const char *const logicalControlNames[LC_COUNT] = {
  "BTN_1", "BTN_2", "BTN_3", "BTN_4", "BTN_5", "BTN_6", "BTN_7",
  "NORMAL_SWITCH", "NORMAL_SWITCH_ALT",
  "LATCH1_UP", "LATCH1_DOWN", "LATCH2_UP", "LATCH2_DOWN",
  "MAIN_SWITCH",
  "MOM_UP", "MOM_DOWN",
  "ENC1_SW", "ENC2_SW",
  "UNUSED_18", "UNUSED_19"
};

// Matrix physical position -> logical control. Rows x columns = 20 positions.
// This stable default map is intentionally simple and predictable for debugging.
const LogicalControl matrixControlMap[MATRIX_ROWS][MATRIX_COLS] = {
  {LC_BTN_1,          LC_BTN_2,          LC_BTN_3,       LC_BTN_4,       LC_BTN_5},
  {LC_BTN_6,          LC_BTN_7,          LC_NORMAL_SWITCH, LC_MAIN_SWITCH, LC_ENC1_SW},
  {LC_LATCH1_UP,      LC_LATCH1_DOWN,    LC_LATCH2_UP,   LC_LATCH2_DOWN, LC_ENC2_SW},
  {LC_MOM_UP,         LC_MOM_DOWN,       LC_NORMAL_SWITCH_ALT, LC_UNUSED_18, LC_UNUSED_19}
};

// -----------------------------------------------------------------------------
// Profiles
// -----------------------------------------------------------------------------

enum ProfileId : uint8_t {
  PROFILE_GENERIC,
  PROFILE_ASSETTO_CORSA,
  PROFILE_ACC,
  PROFILE_WRC10,
  PROFILE_AUTOMOBILISTA,
  PROFILE_COUNT
};

enum ButtonMode : uint8_t {
  MODE_DISABLED,
  MODE_HELD,           // Physical state is mirrored to joystick button.
  MODE_PULSE_ON_PRESS  // Rising edge creates a short pulse. Good for latching switches.
};

struct ControlMapping {
  int8_t hidButton;     // 0..31, or -1 disabled.
  ButtonMode mode;
};

struct EncoderMapping {
  uint8_t cwButton;
  uint8_t ccwButton;
};

struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct Profile {
  const char *name;
  ControlMapping controls[LC_COUNT];
  EncoderMapping enc1;
  EncoderMapping enc2;
  RgbColor color;
};

#define MAP_DISABLED {-1, MODE_DISABLED}
#define MAP_HELD(btn) {btn, MODE_HELD}
#define MAP_PULSE(btn) {btn, MODE_PULSE_ON_PRESS}

// Button-number convention: MHeironimus library uses zero-based button indexes.
// Windows/game UIs may display these as Button 1..32.
const Profile profiles[PROFILE_COUNT] = {
  {
    "Generic",
    {
      MAP_HELD(0),  MAP_HELD(1),  MAP_HELD(2),  MAP_HELD(3),  MAP_HELD(4),
      MAP_HELD(5),  MAP_HELD(6),
      MAP_HELD(7),  MAP_HELD(8),
      MAP_PULSE(9), MAP_PULSE(10), MAP_PULSE(11), MAP_PULSE(12),
      MAP_HELD(13),
      MAP_HELD(14), MAP_HELD(15),
      MAP_HELD(16), MAP_HELD(17),
      MAP_DISABLED, MAP_DISABLED
    },
    {18, 19}, {20, 21}, {0, 0, 255}
  },
  {
    "Assetto Corsa",
    {
      MAP_HELD(0), MAP_HELD(1), MAP_HELD(2), MAP_HELD(3), MAP_HELD(4),
      MAP_HELD(5), MAP_HELD(6),
      MAP_HELD(7), MAP_HELD(8),
      MAP_PULSE(14), MAP_PULSE(15), MAP_PULSE(16), MAP_PULSE(17),
      MAP_HELD(9),
      MAP_HELD(10), MAP_HELD(11),
      MAP_HELD(12), MAP_HELD(13),
      MAP_DISABLED, MAP_DISABLED
    },
    {18, 19}, {20, 21}, {255, 0, 0}
  },
  {
    "Assetto Corsa Competizione",
    {
      MAP_HELD(0), MAP_HELD(1), MAP_HELD(2), MAP_HELD(3), MAP_HELD(4),
      MAP_HELD(5), MAP_HELD(6),
      MAP_HELD(12), MAP_HELD(13),
      MAP_PULSE(8), MAP_PULSE(9), MAP_PULSE(10), MAP_PULSE(11),
      MAP_HELD(14),
      MAP_HELD(15), MAP_HELD(16),
      MAP_HELD(17), MAP_HELD(18),
      MAP_DISABLED, MAP_DISABLED
    },
    {19, 20}, {21, 22}, {0, 255, 0}
  },
  {
    "WRC10",
    {
      MAP_HELD(0), MAP_HELD(1), MAP_HELD(2), MAP_HELD(3), MAP_HELD(4),
      MAP_HELD(5), MAP_HELD(6),
      MAP_HELD(7), MAP_HELD(8),
      MAP_PULSE(9), MAP_PULSE(10), MAP_PULSE(11), MAP_PULSE(12),
      MAP_HELD(13),
      MAP_HELD(14), MAP_HELD(15),
      MAP_HELD(16), MAP_HELD(17),
      MAP_DISABLED, MAP_DISABLED
    },
    {22, 23}, {24, 25}, {255, 160, 0}
  },
  {
    "Automobilista",
    {
      MAP_HELD(0), MAP_HELD(1), MAP_HELD(2), MAP_HELD(3), MAP_HELD(4),
      MAP_HELD(5), MAP_HELD(6),
      MAP_HELD(7), MAP_HELD(8),
      MAP_PULSE(18), MAP_PULSE(19), MAP_PULSE(20), MAP_PULSE(21),
      MAP_HELD(9),
      MAP_HELD(10), MAP_HELD(11),
      MAP_HELD(12), MAP_HELD(13),
      MAP_DISABLED, MAP_DISABLED
    },
    {14, 15}, {16, 17}, {180, 0, 255}
  }
};

// -----------------------------------------------------------------------------
// Runtime state
// -----------------------------------------------------------------------------

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  HID_BUTTON_COUNT,
  0,        // hat switches
  false,    // X
  false,    // Y
  true,     // Z = handbrake axis
  false, false, false, // Rx, Ry, Rz
  false, false, false, false, false // rudder, throttle, accelerator, brake, steering
);

struct DebouncedInput {
  bool raw = false;
  bool stable = false;
  bool previousStable = false;
  uint32_t lastRawChangeMs = 0;
};

DebouncedInput controls[LC_COUNT];
bool matrixRaw[MATRIX_ROWS][MATRIX_COLS];

bool hidHeld[HID_BUTTON_COUNT];
uint32_t hidPulseUntil[HID_BUTTON_COUNT];
bool hidLastSent[HID_BUTTON_COUNT];

uint8_t activeProfile = PROFILE_GENERIC;
uint32_t profileComboStartMs = 0;
bool profileComboArmed = true;
uint32_t profileLockoutUntilMs = 0;
uint32_t profileBlinkUntilMs = 0;

int filteredHandbrake = 0;

const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_PROFILE = 1;
const uint8_t EEPROM_MAGIC = 0x5A;

// Quadrature lookup table. Index = previous AB state << 2 | current AB state.
const int8_t encoderTransitionTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

struct EncoderState {
  uint8_t clkPin;
  uint8_t dtPin;
  uint8_t lastState;
  int8_t accumulator;
};

EncoderState enc1 = {ENC1_CLK_PIN, ENC1_DT_PIN, 0, 0};
EncoderState enc2 = {ENC2_CLK_PIN, ENC2_DT_PIN, 0, 0};

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------

void setRgbRaw(uint8_t r, uint8_t g, uint8_t b) {
  if (RGB_COMMON_ANODE) {
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  }
  // On non-PWM analog pins this behaves as digital off/on, which is fine for
  // profile-color indication with the current pin budget.
  analogWrite(RGB_R_PIN, r);
  analogWrite(RGB_G_PIN, g);
  analogWrite(RGB_B_PIN, b);
}

void setSingleLed(uint8_t index, bool on) {
  if (index >= 3 || SINGLE_LED_PINS[index] < 0) return;
  digitalWrite((uint8_t)SINGLE_LED_PINS[index], on ? HIGH : LOW);
}

void initSingleLeds() {
  for (uint8_t i = 0; i < 3; i++) {
    if (SINGLE_LED_PINS[i] >= 0) {
      pinMode((uint8_t)SINGLE_LED_PINS[i], OUTPUT);
      setSingleLed(i, false);
    }
  }
}

void setProfileLed() {
  RgbColor c = profiles[activeProfile].color;
  bool flashing = millis() < profileBlinkUntilMs;
  if (flashing) {
    // Bright white flash feedback after profile change.
    setRgbRaw(255, 255, 255);
  } else {
    setRgbRaw(c.r, c.g, c.b);
  }

  // Optional single LED 1: heartbeat/profile indicator. It turns on outside Generic
  // and blinks during profile-change feedback. LED 2/3 stay modular-disabled unless
  // you free extra pins or add an expander.
  setSingleLed(0, flashing || activeProfile != PROFILE_GENERIC);
  setSingleLed(1, false);
  setSingleLed(2, false);
}

void startHidPulse(uint8_t button) {
  if (button < HID_BUTTON_COUNT) {
    hidPulseUntil[button] = millis() + HID_PULSE_MS;
  }
}

void clearAllHidButtons() {
  for (uint8_t i = 0; i < HID_BUTTON_COUNT; i++) {
    hidHeld[i] = false;
    hidPulseUntil[i] = 0;
    if (hidLastSent[i]) {
      Joystick.setButton(i, 0);
      hidLastSent[i] = false;
    }
  }
  Joystick.sendState();
}

uint8_t readEncoderPins(const EncoderState &enc) {
  uint8_t a = digitalRead(enc.clkPin) ? 1 : 0;
  uint8_t b = digitalRead(enc.dtPin) ? 1 : 0;
  return (a << 1) | b;
}

void initEncoder(EncoderState &enc) {
  pinMode(enc.clkPin, INPUT_PULLUP);
  pinMode(enc.dtPin, INPUT_PULLUP);
  enc.lastState = readEncoderPins(enc);
  enc.accumulator = 0;
}

void serviceEncoder(EncoderState &enc, const EncoderMapping &mapping) {
  uint8_t current = readEncoderPins(enc);
  if (current == enc.lastState) return;

  uint8_t index = (enc.lastState << 2) | current;
  int8_t delta = encoderTransitionTable[index];
  enc.lastState = current;

  if (delta == 0) {
    // Invalid transition, likely contact bounce. Discard accumulated partial step.
    enc.accumulator = 0;
    return;
  }

  enc.accumulator += delta;

  // Typical mechanical encoders produce four valid transitions per detent.
  if (enc.accumulator >= 4) {
    startHidPulse(mapping.cwButton);
    enc.accumulator = 0;
  } else if (enc.accumulator <= -4) {
    startHidPulse(mapping.ccwButton);
    enc.accumulator = 0;
  }
}

// -----------------------------------------------------------------------------
// Matrix scanning and debouncing
// -----------------------------------------------------------------------------

void initMatrix() {
  for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
    pinMode(rowPins[r], INPUT); // high impedance until selected
  }
  for (uint8_t c = 0; c < MATRIX_COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }
}

void scanMatrixRaw() {
  for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
    // Select exactly one row by driving it LOW.
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(40); // settle time for wiring capacitance and diodes

    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
      matrixRaw[r][c] = (digitalRead(colPins[c]) == LOW);
    }

    // Return row to high impedance so it cannot back-drive another row.
    pinMode(rowPins[r], INPUT);
  }
}

void updateDebouncedControls() {
  bool rawByControl[LC_COUNT];
  for (uint8_t i = 0; i < LC_COUNT; i++) rawByControl[i] = false;

  for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
      LogicalControl lc = matrixControlMap[r][c];
      if (lc < LC_COUNT && matrixRaw[r][c]) rawByControl[lc] = true;
    }
  }

  uint32_t now = millis();
  for (uint8_t i = 0; i < LC_COUNT; i++) {
    DebouncedInput &input = controls[i];
    input.previousStable = input.stable;

    if (rawByControl[i] != input.raw) {
      input.raw = rawByControl[i];
      input.lastRawChangeMs = now;
    }

    if ((now - input.lastRawChangeMs) >= MATRIX_DEBOUNCE_MS) {
      input.stable = input.raw;
    }
  }
}

// -----------------------------------------------------------------------------
// Profile / EEPROM
// -----------------------------------------------------------------------------

void loadProfileFromEeprom() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  uint8_t savedProfile = EEPROM.read(EEPROM_ADDR_PROFILE);
  if (magic == EEPROM_MAGIC && savedProfile < PROFILE_COUNT) {
    activeProfile = savedProfile;
  } else {
    activeProfile = PROFILE_GENERIC;
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_PROFILE, activeProfile);
  }
}

void saveProfileToEepromIfChanged(uint8_t profile) {
  if (EEPROM.read(EEPROM_ADDR_PROFILE) != profile) {
    EEPROM.update(EEPROM_ADDR_PROFILE, profile);
  }
}

void setActiveProfile(uint8_t profile) {
  if (profile >= PROFILE_COUNT || profile == activeProfile) return;
  activeProfile = profile;
  saveProfileToEepromIfChanged(activeProfile);
  clearAllHidButtons();
  profileBlinkUntilMs = millis() + 250;
}

void serviceProfileSwitchCombo() {
  uint32_t now = millis();
  if (now < profileLockoutUntilMs) return;

  bool bothEncoderButtons = controls[LC_ENC1_SW].stable && controls[LC_ENC2_SW].stable;

  if (!bothEncoderButtons) {
    profileComboStartMs = 0;
    profileComboArmed = true;
    return;
  }

  if (!profileComboArmed) return;

  if (profileComboStartMs == 0) {
    profileComboStartMs = now;
  } else if ((now - profileComboStartMs) >= PROFILE_HOLD_MS) {
    uint8_t nextProfile = (activeProfile + 1) % PROFILE_COUNT;
    setActiveProfile(nextProfile);
    profileComboArmed = false;
    profileLockoutUntilMs = now + PROFILE_CHANGE_LOCKOUT_MS;
  }
}

// -----------------------------------------------------------------------------
// HID updates
// -----------------------------------------------------------------------------

void serviceMappedControls() {
  for (uint8_t i = 0; i < HID_BUTTON_COUNT; i++) hidHeld[i] = false;

  const Profile &profile = profiles[activeProfile];
  for (uint8_t i = 0; i < LC_COUNT; i++) {
    const ControlMapping &mapping = profile.controls[i];
    if (mapping.hidButton < 0 || mapping.hidButton >= HID_BUTTON_COUNT) continue;

    bool pressed = controls[i].stable;
    bool rose = controls[i].stable && !controls[i].previousStable;

    switch (mapping.mode) {
      case MODE_HELD:
        if (pressed) hidHeld[mapping.hidButton] = true;
        break;
      case MODE_PULSE_ON_PRESS:
        if (rose) startHidPulse(mapping.hidButton);
        break;
      case MODE_DISABLED:
      default:
        break;
    }
  }
}

void serviceHandbrakeAxis() {
  int raw = analogRead(PIN_HANDBRAKE);
  raw = constrain(raw, HANDBRAKE_RAW_MIN, HANDBRAKE_RAW_MAX);
  int mapped = map(raw, HANDBRAKE_RAW_MIN, HANDBRAKE_RAW_MAX, HID_AXIS_MIN, HID_AXIS_MAX);
  mapped = constrain(mapped, HID_AXIS_MIN, HID_AXIS_MAX);
  if (HANDBRAKE_INVERT) mapped = HID_AXIS_MAX - mapped;

  if (HANDBRAKE_SMOOTHING_SHIFT == 0) {
    filteredHandbrake = mapped;
  } else {
    filteredHandbrake += (mapped - filteredHandbrake) >> HANDBRAKE_SMOOTHING_SHIFT;
  }

  Joystick.setZAxis(filteredHandbrake);
}

void sendHidState() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < HID_BUTTON_COUNT; i++) {
    bool pulseActive = (hidPulseUntil[i] != 0 && (int32_t)(hidPulseUntil[i] - now) > 0);
    if (!pulseActive && hidPulseUntil[i] != 0) hidPulseUntil[i] = 0;

    bool finalState = hidHeld[i] || pulseActive;
    if (finalState != hidLastSent[i]) {
      Joystick.setButton(i, finalState ? 1 : 0);
      hidLastSent[i] = finalState;
    }
  }
  Joystick.sendState();
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  initSingleLeds();
  setRgbRaw(0, 0, 0);

  pinMode(PIN_HANDBRAKE, INPUT);
  initMatrix();
  initEncoder(enc1);
  initEncoder(enc2);

  loadProfileFromEeprom();
  filteredHandbrake = analogRead(PIN_HANDBRAKE);

  Joystick.setZAxisRange(HID_AXIS_MIN, HID_AXIS_MAX);
  Joystick.begin(false); // manual sendState for stable, grouped HID reports

  profileBlinkUntilMs = millis() + 500;
}

void loop() {
  scanMatrixRaw();
  updateDebouncedControls();

  serviceProfileSwitchCombo();

  const Profile &profile = profiles[activeProfile];
  serviceEncoder(enc1, profile.enc1);
  serviceEncoder(enc2, profile.enc2);
  serviceMappedControls();
  serviceHandbrakeAxis();
  sendHidState();
  setProfileLed();

  delay(LOOP_DELAY_MS);
}
