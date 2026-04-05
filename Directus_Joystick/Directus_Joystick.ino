// =============================================================================
// GripSwitch – Directus + Joystick/Fahrhebel
// =============================================================================
// Griff:    Directus (16 Tasten via MCP23017, Thumbstick auf A0/A1)
// Unterbau: Joystick/Fahrhebel (2x AS5600 via TCA9548A I2C-Multiplexer)
//           Rx-Achse: Multiplexer-Kanal 0
//           Ry-Achse: Multiplexer-Kanal 1
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
#define CAL_MCP_PIN_A  0    // Taste 1  (GPA0)
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

// --- I2C Multiplexer (TCA9548A) ---
#define MUX_ADDR   0x70
#define MUX_CH_X   0      // AS5600 für Rx-Achse an Kanal 0
#define MUX_CH_Y   1      // AS5600 für Ry-Achse an Kanal 1

// --- Filter ---
#define FILTER_SIZE 8

// --- EEPROM (2 Achsen: Rx und Ry) ---
#define EEPROM_FLAG_ADDR   0     // 1 Byte
#define EEPROM_X_CENTER    1     // 2 Bytes
#define EEPROM_X_MIN       3     // 2 Bytes
#define EEPROM_X_MAX       5     // 2 Bytes
#define EEPROM_Y_CENTER    7     // 2 Bytes
#define EEPROM_Y_MIN       9     // 2 Bytes
#define EEPROM_Y_MAX       11    // 2 Bytes
#define EEPROM_VALID_FLAG  0xAB  // Gesamt: 13 Bytes

// --- Kalibrierung Timings ---
#define CAL_ACTIVATE_MS  10000
#define CAL_CENTER_MS     5000
#define CAL_STOP_MS       5000

// --- MCP23017 ---
Adafruit_MCP23X17 mcp;

// --- HID Joystick (Thumbstick X/Y + Fahrhebel Rx/Ry) ---
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  HID_BUTTON_COUNT,    // Buttons
  0,                   // Hat Switches
  true,  true,  false, // X (Thumbstick), Y (Thumbstick), Z
  true,  true,  false, // Rx (AS5600), Ry (AS5600), Rz
  false, false,        // Rudder, Throttle
  false, false, false  // Accelerator, Brake, Steering
);

// --- Kalibrierungswerte Rx ---
int16_t rxCenter = 2048;
int16_t rxMin    = 0;
int16_t rxMax    = 4095;

// --- Kalibrierungswerte Ry ---
int16_t ryCenter = 2048;
int16_t ryMin    = 0;
int16_t ryMax    = 4095;

// --- Filter Rx ---
int rxFilterBuffer[FILTER_SIZE];
int rxFilterIndex = 0;
long rxFilterSum  = 0;

// --- Filter Ry ---
int ryFilterBuffer[FILTER_SIZE];
int ryFilterIndex = 0;
long ryFilterSum  = 0;

// --- Überlauf-Erkennung ---
int16_t rxLastHw = 2048, rxVirtual = 2048;
int16_t ryLastHw = 2048, ryVirtual = 2048;

// =============================================================================
// I2C Multiplexer: Kanal auswählen
// =============================================================================
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// =============================================================================
// AS5600 lesen – Überlauf-korrigiert
// Voraussetzung: MUX-Kanal muss vorher gesetzt sein
// =============================================================================
int readVirtual(int16_t &lastHw, int16_t &virt) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return (int)virt;
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return (int)virt;
  int raw = ((Wire.read() & 0x0F) << 8) | Wire.read();
  int16_t delta = (int16_t)((int16_t)raw - lastHw);
  if (delta >  2000) delta = (int16_t)(delta - 4096);
  if (delta < -2000) delta = (int16_t)(delta + 4096);
  lastHw  = (int16_t)raw;
  virt   += delta;
  return (int)virt;
}

// =============================================================================
// Gefilterte Achswerte lesen
// =============================================================================
int readFilteredRx() {
  selectMuxChannel(MUX_CH_X);
  int raw = readVirtual(rxLastHw, rxVirtual);
  rxFilterSum -= rxFilterBuffer[rxFilterIndex];
  rxFilterBuffer[rxFilterIndex] = raw;
  rxFilterSum += raw;
  rxFilterIndex = (rxFilterIndex + 1) % FILTER_SIZE;
  return (int)(rxFilterSum / FILTER_SIZE);
}

int readFilteredRy() {
  selectMuxChannel(MUX_CH_Y);
  int raw = readVirtual(ryLastHw, ryVirtual);
  ryFilterSum -= ryFilterBuffer[ryFilterIndex];
  ryFilterBuffer[ryFilterIndex] = raw;
  ryFilterSum += raw;
  ryFilterIndex = (ryFilterIndex + 1) % FILTER_SIZE;
  return (int)(ryFilterSum / FILTER_SIZE);
}

void initFilters() {
  int initRx = (int)rxVirtual;
  for (int i = 0; i < FILTER_SIZE; i++) rxFilterBuffer[i] = initRx;
  rxFilterSum   = (long)initRx * FILTER_SIZE;
  rxFilterIndex = 0;

  int initRy = (int)ryVirtual;
  for (int i = 0; i < FILTER_SIZE; i++) ryFilterBuffer[i] = initRy;
  ryFilterSum   = (long)initRy * FILTER_SIZE;
  ryFilterIndex = 0;
}

// =============================================================================
// EEPROM
// =============================================================================
void loadCalibration() {
  if (EEPROM.read(EEPROM_FLAG_ADDR) == EEPROM_VALID_FLAG) {
    EEPROM.get(EEPROM_X_CENTER, rxCenter);
    EEPROM.get(EEPROM_X_MIN,    rxMin);
    EEPROM.get(EEPROM_X_MAX,    rxMax);
    EEPROM.get(EEPROM_Y_CENTER, ryCenter);
    EEPROM.get(EEPROM_Y_MIN,    ryMin);
    EEPROM.get(EEPROM_Y_MAX,    ryMax);
  }
}

void saveCalibration() {
  EEPROM.put(EEPROM_X_CENTER, rxCenter);
  EEPROM.put(EEPROM_X_MIN,    rxMin);
  EEPROM.put(EEPROM_X_MAX,    rxMax);
  EEPROM.put(EEPROM_Y_CENTER, ryCenter);
  EEPROM.put(EEPROM_Y_MIN,    ryMin);
  EEPROM.put(EEPROM_Y_MAX,    ryMax);
  EEPROM.write(EEPROM_FLAG_ADDR, EEPROM_VALID_FLAG);
}

// =============================================================================
// Achsen-Mapping: zweistufig mit Mittelpunkt
// =============================================================================
int32_t mapAxis(int angle, int16_t center, int16_t minA, int16_t maxA) {
  if (angle <= center) {
    if (center == minA) return 0;
    return map((long)angle, minA, center, -32767, 0);
  } else {
    if (maxA == center) return 0;
    return map((long)angle, center, maxA, 0, 32767);
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
// Kalibrierungsmodus (2 AS5600-Achsen: Rx und Ry gleichzeitig)
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

  // --- Phase 1: Mittelposition Rx + Ry (5 Sekunden) ---
  long rxSum = 0, rySum = 0;
  int  samples = 0;
  unsigned long phaseStart = millis();

  while (millis() - phaseStart < CAL_CENTER_MS) {
    blinkIndicator(500);
    selectMuxChannel(MUX_CH_X); rxSum += readVirtual(rxLastHw, rxVirtual);
    selectMuxChannel(MUX_CH_Y); rySum += readVirtual(ryLastHw, ryVirtual);
    samples++;
    delay(10);
  }
  rxCenter = (int16_t)(rxSum / samples);
  ryCenter = (int16_t)(rySum / samples);
  rxMin = rxCenter; rxMax = rxCenter;
  ryMin = ryCenter; ryMax = ryCenter;

  // --- Phase 2: Min/Max Rx + Ry automatisch erkennen ---
  Joystick.setButton(CAL_INDICATOR_BTN, 1);
  Joystick.sendState();

  unsigned long holdStart = 0;
  bool holding = false;

  while (true) {
    selectMuxChannel(MUX_CH_X); int rawRx = readVirtual(rxLastHw, rxVirtual);
    if (rawRx < rxMin) rxMin = rawRx;
    if (rawRx > rxMax) rxMax = rawRx;

    selectMuxChannel(MUX_CH_Y); int rawRy = readVirtual(ryLastHw, ryVirtual);
    if (rawRy < ryMin) ryMin = rawRy;
    if (rawRy > ryMax) ryMax = rawRy;

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
  initFilters();

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
  int hw;
  // Rx-Achse
  selectMuxChannel(MUX_CH_X);
  Wire.beginTransmission(AS5600_ADDR); Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false); Wire.requestFrom(AS5600_ADDR, 2);
  hw = (Wire.available() >= 2) ? (((Wire.read() & 0x0F) << 8) | Wire.read()) : 2048;
  { int16_t c0=(int16_t)hw, c1=(int16_t)(hw-4096), c2=(int16_t)(hw+4096), best=c0;
    if (abs((int)(c1-rxCenter)) < abs((int)(best-rxCenter))) best=c1;
    if (abs((int)(c2-rxCenter)) < abs((int)(best-rxCenter))) best=c2;
    rxLastHw=(int16_t)hw; rxVirtual=best; }
  // Ry-Achse
  selectMuxChannel(MUX_CH_Y);
  Wire.beginTransmission(AS5600_ADDR); Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false); Wire.requestFrom(AS5600_ADDR, 2);
  hw = (Wire.available() >= 2) ? (((Wire.read() & 0x0F) << 8) | Wire.read()) : 2048;
  { int16_t c0=(int16_t)hw, c1=(int16_t)(hw-4096), c2=(int16_t)(hw+4096), best=c0;
    if (abs((int)(c1-ryCenter)) < abs((int)(best-ryCenter))) best=c1;
    if (abs((int)(c2-ryCenter)) < abs((int)(best-ryCenter))) best=c2;
    ryLastHw=(int16_t)hw; ryVirtual=best; }
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
  Joystick.setXAxisRange(0, 1023);       // Thumbstick X
  Joystick.setYAxisRange(0, 1023);       // Thumbstick Y
  Joystick.setRxAxisRange(-32767, 32767); // Fahrhebel Rx (AS5600)
  Joystick.setRyAxisRange(-32767, 32767); // Fahrhebel Ry (AS5600)

  // AS5600 Kalibrierung laden
  loadCalibration();
  initVirtualTracking();
  initFilters();

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

  // Fahrhebel-Achsen (AS5600 via Multiplexer)
  int rxAngle = readFilteredRx();
  int ryAngle = readFilteredRy();

  int32_t rxAxis = constrain(mapAxis(rxAngle, rxCenter, rxMin, rxMax), -32767, 32767);
  int32_t ryAxis = constrain(mapAxis(ryAngle, ryCenter, ryMin, ryMax), -32767, 32767);

  Joystick.setRxAxis((int)rxAxis);
  Joystick.setRyAxis((int)ryAxis);

  // Kalibrierungs-Indikator aus
  Joystick.setButton(CAL_INDICATOR_BTN, 0);

  Joystick.sendState();
  delay(10); // ~100 Hz
}
