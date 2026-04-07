// =============================================================================
// GripSwitch – Directus + Schubhebel
// =============================================================================
// Griff:    Directus (16 Tasten via MCP23017, Thumbstick auf A0/A1)
// Unterbau: Schubhebel (1x AS5600, Z-Achse, bidirektional -32767 bis +32767)
// Board:    Arduino Pro Micro (ATmega32U4)
// Library:  M.Heironimus Joystick Library, Adafruit MCP23X17
// =============================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <Joystick.h>
#include <Adafruit_MCP23X17.h>

// =============================================================================
// === GRIP-SPECIFIC: MCP23017 BUTTON MAPPING ===
// =============================================================================
const uint8_t MCP_PIN_MAP[16] = {
  0, 1, 2, 3, 4, 5, 6, 7,        // Taste 1-8  → GPA0-GPA7
  15, 9, 8, 10, 11, 12, 13, 14   // Taste 9-16 → GPB7,1,0,2,3,4,5,6
};

#define MCP_ADDR  0x20

// =============================================================================
// === GRIP-SPECIFIC: CALIBRATION TRIGGER ===
// =============================================================================
#define CAL_MCP_PIN_A  13   // Taste 15 (GPB5)
#define CAL_MCP_PIN_B  14   // Taste 16 (GPB6 = 12-mm-Taster)

// =============================================================================
// === GRIP-SPECIFIC: HID CONFIGURATION ===
// =============================================================================
#define HID_BUTTON_COUNT  17
#define CAL_INDICATOR_BTN 16

// =============================================================================
// === GRIP-SPECIFIC: THUMBSTICK ===
// =============================================================================
#define THUMBSTICK_X_PIN  A0
#define THUMBSTICK_Y_PIN  A1

// --- AS5600 ---
#define AS5600_ADDR         0x36
#define AS5600_RAW_ANGLE_H  0x0C

// --- Filter ---
#define FILTER_SIZE 8

// --- EEPROM ---
#define EEPROM_FLAG_ADDR    0     // 1 Byte  – Validierungs-Flag
#define EEPROM_CENTER_ADDR  1     // 2 Bytes – centerAngle
#define EEPROM_MIN_ADDR     3     // 2 Bytes – minAngle
#define EEPROM_MAX_ADDR     5     // 2 Bytes – maxAngle
#define EEPROM_VALID_FLAG   0xAB

// --- Kalibrierung Timings ---
#define CAL_ACTIVATE_MS   10000
#define CAL_CENTER_MS      5000
#define CAL_STOP_MS        5000

// --- MCP23017 ---
Adafruit_MCP23X17 mcp;

// --- HID Joystick (Thumbstick X/Y + Schub Z) ---
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  HID_BUTTON_COUNT,    // Buttons
  0,                   // Hat Switches
  true,  true,  true,  // X (Thumbstick), Y (Thumbstick), Z (AS5600 Schub)
  false, false, false, // Rx, Ry, Rz
  false, false,        // Rudder, Throttle
  false, false, false  // Accelerator, Brake, Steering
);

// --- AS5600 Kalibrierung ---
int16_t centerAngle = 2048;
int16_t minAngle    = 0;
int16_t maxAngle    = 4095;

int filterBuffer[FILTER_SIZE];
int filterIndex = 0;
long filterSum  = 0;

// --- Überlauf-Erkennung ---
int16_t lastHardwareRaw = 2048;
int16_t virtualRaw      = 2048;

// =============================================================================
// AS5600 lesen (Überlauf-korrigiert)
// =============================================================================
int readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  int raw;
  if (Wire.available() >= 2) {
    int high = Wire.read();
    int low  = Wire.read();
    raw = ((high & 0x0F) << 8) | low;
  } else {
    return (int)virtualRaw;
  }
  int16_t delta = (int16_t)((int16_t)raw - lastHardwareRaw);
  if (delta >  2000) delta = (int16_t)(delta - 4096);
  if (delta < -2000) delta = (int16_t)(delta + 4096);
  lastHardwareRaw = (int16_t)raw;
  virtualRaw     += delta;
  return (int)virtualRaw;
}

// =============================================================================
// Moving Average Filter
// =============================================================================
int readFiltered() {
  int raw = readAS5600();
  filterSum -= filterBuffer[filterIndex];
  filterBuffer[filterIndex] = raw;
  filterSum += raw;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  return (int)(filterSum / FILTER_SIZE);
}

void initFilter(int initialValue) {
  for (int i = 0; i < FILTER_SIZE; i++) filterBuffer[i] = initialValue;
  filterSum   = (long)initialValue * FILTER_SIZE;
  filterIndex = 0;
}

// =============================================================================
// EEPROM
// =============================================================================
void loadCalibration() {
  if (EEPROM.read(EEPROM_FLAG_ADDR) == EEPROM_VALID_FLAG) {
    EEPROM.get(EEPROM_CENTER_ADDR, centerAngle);
    EEPROM.get(EEPROM_MIN_ADDR,    minAngle);
    EEPROM.get(EEPROM_MAX_ADDR,    maxAngle);
  }
}

void saveCalibration() {
  EEPROM.put(EEPROM_CENTER_ADDR, centerAngle);
  EEPROM.put(EEPROM_MIN_ADDR,    minAngle);
  EEPROM.put(EEPROM_MAX_ADDR,    maxAngle);
  EEPROM.write(EEPROM_FLAG_ADDR, EEPROM_VALID_FLAG);
}

// =============================================================================
// Achsen-Mapping: zweistufig mit Mittelpunkt
// =============================================================================
int32_t mapAxis(int angle) {
  if (angle <= centerAngle) {
    if (centerAngle == minAngle) return 0;
    return map((long)angle, minAngle, centerAngle, -32767, 0);
  } else {
    if (maxAngle == centerAngle) return 0;
    return map((long)angle, centerAngle, maxAngle, 0, 32767);
  }
}

// =============================================================================
// Kalibrierungs-Tasten gedrückt? (über MCP23017)
// =============================================================================
bool calButtonsHeld() {
  return (!mcp.digitalRead(CAL_MCP_PIN_A) && !mcp.digitalRead(CAL_MCP_PIN_B));
}

void blinkIndicator(unsigned long intervalMs) {
  bool on = ((millis() / intervalMs) % 2 == 0);
  Joystick.setButton(CAL_INDICATOR_BTN, on ? 1 : 0);
  Joystick.sendState();
}

// =============================================================================
// MCP23017-Buttons lesen
// =============================================================================
void readMCPButtons() {
  for (int i = 0; i < 16; i++) {
    bool pressed = !mcp.digitalRead(MCP_PIN_MAP[i]);
    Joystick.setButton(i, pressed ? 1 : 0);
  }
}

// =============================================================================
// Kalibrierungsmodus (AS5600 Schubachse)
//
// Blinkmuster:
//   Schnell (200ms)     = 2-Sek. Wartezeit nach Loslassen der Tasten
//   Langsam (500ms)     = Phase 1: Hebel in Mittelstellung halten
//   Dauerhaft AN        = Phase 2: Hebel voll vor und zurück bewegen
//   Sehr schnell (100ms)= Bestätigung: Werte gespeichert
// =============================================================================
void runCalibration() {
  Joystick.setButton(CAL_INDICATOR_BTN, 1);
  Joystick.sendState();

  while (calButtonsHeld()) delay(10);

  unsigned long waitStart = millis();
  while (millis() - waitStart < 2000) {
    blinkIndicator(200);
    delay(10);
  }

  // --- Phase 1: Mittelposition (5 Sekunden) ---
  long sum    = 0;
  int samples = 0;
  unsigned long phaseStart = millis();

  while (millis() - phaseStart < CAL_CENTER_MS) {
    blinkIndicator(500);
    sum += readAS5600();
    samples++;
    delay(10);
  }
  centerAngle = (int16_t)(sum / samples);
  minAngle    = centerAngle;
  maxAngle    = centerAngle;

  // --- Phase 2: Min/Max automatisch erkennen ---
  Joystick.setButton(CAL_INDICATOR_BTN, 1);
  Joystick.sendState();

  unsigned long holdStart = 0;
  bool holding = false;

  while (true) {
    int raw = readAS5600();
    if (raw < minAngle) minAngle = raw;
    if (raw > maxAngle) maxAngle = raw;

    if (calButtonsHeld()) {
      if (!holding) { holding = true; holdStart = millis(); }
      else if (millis() - holdStart >= CAL_STOP_MS) break;
    } else {
      holding = false;
    }

    Joystick.sendState();
    delay(10);
  }

  saveCalibration();
  initFilter(readAS5600());

  unsigned long confirmStart = millis();
  while (millis() - confirmStart < 1000) {
    blinkIndicator(100);
    delay(10);
  }

  Joystick.setButton(CAL_INDICATOR_BTN, 0);
  Joystick.sendState();
  while (calButtonsHeld()) delay(10);
}

// =============================================================================
// Überlauf-Tracking initialisieren
// =============================================================================
void initVirtualTracking() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  int hw = 2048;
  if (Wire.available() >= 2) {
    int high = Wire.read();
    int low  = Wire.read();
    hw = ((high & 0x0F) << 8) | low;
  }
  int16_t c0 = (int16_t)hw;
  int16_t c1 = (int16_t)(hw - 4096);
  int16_t c2 = (int16_t)(hw + 4096);
  int16_t best = c0;
  if (abs((int)(c1 - centerAngle)) < abs((int)(best - centerAngle))) best = c1;
  if (abs((int)(c2 - centerAngle)) < abs((int)(best - centerAngle))) best = c2;
  lastHardwareRaw = (int16_t)hw;
  virtualRaw      = best;
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Wire.begin();

  // MCP23017 initialisieren
  mcp.begin_I2C(MCP_ADDR);
  for (int i = 0; i < 16; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  // Joystick initialisieren
  Joystick.begin(false);
  Joystick.setXAxisRange(0, 1023);     // Thumbstick X
  Joystick.setYAxisRange(0, 1023);     // Thumbstick Y
  Joystick.setZAxisRange(-32767, 32767); // Schubachse (AS5600)

  // AS5600 Kalibrierung laden
  loadCalibration();
  initVirtualTracking();
  initFilter(readAS5600());

  // Kalibrier-Check beim Start
  if (calButtonsHeld()) {
    unsigned long holdStart = millis();
    bool activated = false;
    while (calButtonsHeld()) {
      if (millis() - holdStart >= CAL_ACTIVATE_MS) { activated = true; break; }
      delay(10);
    }
    if (activated) runCalibration();
  }
}

// =============================================================================
// Hauptschleife
// =============================================================================
void loop() {
  // Kalibrierung jederzeit aktivierbar
  static unsigned long calHoldStart = 0;
  static bool calHolding = false;

  if (calButtonsHeld()) {
    if (!calHolding) { calHolding = true; calHoldStart = millis(); }
    else if (millis() - calHoldStart >= CAL_ACTIVATE_MS) {
      runCalibration();
      calHolding = false;
    }
  } else {
    calHolding = false;
  }

  // MCP23017-Buttons lesen
  readMCPButtons();

  // Thumbstick (analog)
  Joystick.setXAxis(analogRead(THUMBSTICK_X_PIN));
  Joystick.setYAxis(analogRead(THUMBSTICK_Y_PIN));

  // Schubachse (AS5600, bidirektional)
  int angle    = readFiltered();
  int32_t axis = constrain(mapAxis(angle), -32767, 32767);
  Joystick.setZAxis((int)axis);

  // Kalibrierungs-Indikator aus
  Joystick.setButton(CAL_INDICATOR_BTN, 0);

  Joystick.sendState();
  delay(10); // ~100 Hz
}
