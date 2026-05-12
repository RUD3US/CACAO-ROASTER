/*
 * ============================================================
 *  CACAO ROASTER CONTROLLER
 *  Hardware: Arduino Mega
 *  Components:
 *    - MAX6675 K-Type Thermocouple
 *    - DS3231 RTC
 *    - I2C LCD 20x4
 *    - DimmableLight (IR Heater via TRIAC)
 *    - Relay 1 → LPG Solenoid Valve
 *    - Relay 2 → Tempering Fan
 *    - Single Push Button
 * ============================================================
 *
 *  PIN ASSIGNMENTS:
 *    D18 → Zero-Cross input (INT5 on Mega) — used by DimmableLight
 *    D3  → TRIAC gate (DimmableLight output)
 *    D4  → MAX6675 CS
 *    D5  → MAX6675 SCK
 *    D6  → MAX6675 SO (MISO)
 *    D52 → Relay 1 (LPG Solenoid Valve) — Active LOW
 *    D51 → Relay 2 (Fan)                — Active LOW
 *    D40 → Button (INPUT_PULLUP)
 *    A4  → I2C SDA (LCD + RTC)
 *    A5  → I2C SCL (LCD + RTC)
 *
 *  LIBRARIES REQUIRED:
 *    - Wire.h               (built-in)
 *    - LiquidCrystal_I2C.h  (Frank de Brabander)
 *    - max6675.h            (Adafruit)
 *    - RTClib.h             (Adafruit)
 *    - PID_v1.h             (Brett Beauregard)
 *    - DimmableLight.h      (Fabiano Riccardi — "Dimmable Light for Arduino" v1.6+)
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>
#include <RTClib.h>
#include <PID_v1.h>
#include <DimmableLight.h>

// ─── PIN DEFINITIONS ────────────────────────────────────────
#define PIN_ZC          18    // Zero-cross input (INT5 on Mega)
#define PIN_DIMMER       3    // TRIAC gate output
#define PIN_TC_CS        4    // MAX6675 Chip Select
#define PIN_TC_SCK       5    // MAX6675 Clock
#define PIN_TC_SO        6    // MAX6675 Data Out
#define PIN_RELAY_VALVE 52    // LPG Solenoid Valve relay (Active LOW)
#define PIN_RELAY_FAN   51    // Tempering Fan relay (Active LOW)
#define PIN_BUTTON      40    // Single push button

// ─── TIMING CONSTANTS ───────────────────────────────────────
#define PHASE1_DURATION_MS   300000UL   // 5 min
#define PHASE2_DURATION_MS   1500000UL  // 25 min
#define TOTAL_ROAST_MS       1800000UL  // 30 min total
#define COOLING_TEMP_TARGET  50.0       // Stop fan below this °C

// ─── PID SETTINGS ───────────────────────────────────────────
#define PID_KP              2.0
#define PID_KI              0.5
#define PID_KD              1.0
#define PID_SAMPLE_MS       250

// ─── FAN FEEDBACK (anti-overshoot) ──────────────────────────
#define FAN_ON_THRESHOLD    3.0
#define FAN_OFF_HYSTERESIS  1.0

// ─── SETPOINT SETTINGS ──────────────────────────────────────
#define SETPOINT_MIN        160
#define SETPOINT_MAX        230
#define SETPOINT_STEP       5
#define SETPOINT_DEFAULT    200

// ─── SAFETY LIMITS ──────────────────────────────────────────
#define OVERTEMP_MARGIN     20.0
#define HEATER_FAIL_TIME_MS 120000UL

// ─── BUTTON TIMING ──────────────────────────────────────────
#define BTN_DEBOUNCE_MS     50
#define BTN_LONG_PRESS_MS   1500

// ─── OBJECTS ────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 20, 4);
MAX6675           thermocouple(PIN_TC_SCK, PIN_TC_CS, PIN_TC_SO);
RTC_DS3231        rtc;
DimmableLight     dimmer(PIN_DIMMER);

// ─── PID VARIABLES ──────────────────────────────────────────
double pidInput    = 0.0;
double pidOutput   = 0.0;
double pidSetpoint = SETPOINT_DEFAULT;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// ─── STATE MACHINE ──────────────────────────────────────────
enum SystemState {
  STATE_IDLE,
  STATE_SET_SETPOINT,
  STATE_PHASE1_PREHEAT,
  STATE_PHASE2_ROAST,
  STATE_COOLING,
  STATE_DONE,
  STATE_ERROR
};

enum ButtonEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

// ─── GLOBAL STATE ───────────────────────────────────────────
SystemState currentState = STATE_IDLE;
String      errorMessage = "";

// ─── RUNTIME VARIABLES ──────────────────────────────────────
int    setpointTemp  = SETPOINT_DEFAULT;
double currentTemp   = 0.0;
double peakTemp      = 0.0;
bool   fanOn         = false;
bool   valveOpen     = false;

unsigned long phaseStartTime   = 0;
unsigned long lastPIDTime      = 0;
unsigned long lastTempReadTime = 0;
unsigned long lastLCDTime      = 0;
unsigned long heaterFailTimer  = 0;
bool          heaterFailArmed  = false;

DateTime roastStartTime;
DateTime roastEndTime;

// ─── BUTTON VARIABLES ───────────────────────────────────────
bool          btnLastState     = HIGH;
bool          btnCurrentState  = HIGH;
unsigned long btnPressTime     = 0;
bool          longPressHandled = false;

// ─── CUSTOM LCD CHARACTER: DEGREE SYMBOL ────────────────────
byte degChar[8] = {
  0b00110, 0b01001, 0b01001, 0b00110,
  0b00000, 0b00000, 0b00000, 0b00000
};

// ============================================================
//  RELAY HELPERS (Active LOW)
// ============================================================
void valveOpen_() { digitalWrite(PIN_RELAY_VALVE, LOW);  valveOpen = true;  }
void valveClose() { digitalWrite(PIN_RELAY_VALVE, HIGH); valveOpen = false; }
void fanOn_()     { digitalWrite(PIN_RELAY_FAN,   LOW);  fanOn     = true;  }
void fanOff()     { digitalWrite(PIN_RELAY_FAN,   HIGH); fanOn     = false; }

void heaterSetPower(uint8_t power) { dimmer.setBrightness(power); }
void heaterOff()                   { dimmer.setBrightness(0);     }

void allOff() {
  heaterOff();
  valveClose();
  fanOff();
}

// ============================================================
//  TEMPERATURE READING
// ============================================================
bool readTemperature() {
  double t = thermocouple.readCelsius();
  if (isnan(t) || t <= 0.0) return false;
  currentTemp = t;
  if (currentTemp > peakTemp) peakTemp = currentTemp;
  return true;
}

// ============================================================
//  FAN FEEDBACK CONTROL
// ============================================================
void updateFanFeedback() {
  if (currentTemp >= (pidSetpoint + FAN_ON_THRESHOLD)) {
    fanOn_();
  } else if (currentTemp < (pidSetpoint + FAN_ON_THRESHOLD - FAN_OFF_HYSTERESIS)) {
    fanOff();
  }
}

// ============================================================
//  BUTTON READING
// ============================================================
ButtonEvent readButton() {
  bool raw = digitalRead(PIN_BUTTON);

  if (raw != btnLastState) {
    delay(BTN_DEBOUNCE_MS);
    raw = digitalRead(PIN_BUTTON);
  }
  btnLastState = raw;

  ButtonEvent event = BTN_NONE;

  if (raw == LOW && btnCurrentState == HIGH) {
    btnPressTime     = millis();
    longPressHandled = false;
    btnCurrentState  = LOW;
  }

  if (raw == LOW && !longPressHandled) {
    if (millis() - btnPressTime >= BTN_LONG_PRESS_MS) {
      longPressHandled = true;
      event = BTN_LONG;
    }
  }

  if (raw == HIGH && btnCurrentState == LOW) {
    btnCurrentState = HIGH;
    if (!longPressHandled) event = BTN_SHORT;
  }

  return event;
}

// ============================================================
//  LCD HELPERS
// ============================================================
void lcdClear() { lcd.clear(); }

String formatTime(unsigned long ms) {
  unsigned long secs = ms / 1000;
  unsigned long mins = secs / 60;
  secs = secs % 60;
  char buf[6];
  sprintf(buf, "%02lu:%02lu", mins, secs);
  return String(buf);
}

// ============================================================
//  LCD DISPLAY FUNCTIONS
// ============================================================
void displayIdle() {
  DateTime now = rtc.now();
  char timeBuf[9], dateBuf[11];
  sprintf(timeBuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  sprintf(dateBuf, "%04d-%02d-%02d", now.year(), now.month(), now.day());

  lcd.setCursor(0, 0); lcd.print(F("  CACAO ROASTER     "));
  lcd.setCursor(0, 1); lcd.print(F("  ")); lcd.print(timeBuf); lcd.print(F("         "));
  lcd.setCursor(0, 2); lcd.print(F("  ")); lcd.print(dateBuf); lcd.print(F("     "));
  lcd.setCursor(0, 3); lcd.print(F(" PRESS TO SET TEMP  "));
}

void displaySetpoint() {
  lcd.setCursor(0, 0); lcd.print(F("  SET ROAST TEMP    "));
  lcd.setCursor(0, 1); lcd.print(F("                    "));
  lcd.setCursor(0, 2);
  char buf[17];
  sprintf(buf, "   SETPOINT: %3d", setpointTemp);
  lcd.print(buf); lcd.write(0); lcd.print(F("C  "));
  lcd.setCursor(0, 3); lcd.print(F(" SHORT=+5 LONG=START"));
}

void displayPhase1() {
  unsigned long elapsed   = millis() - phaseStartTime;
  unsigned long remaining = (elapsed < PHASE1_DURATION_MS) ? (PHASE1_DURATION_MS - elapsed) : 0;

  lcd.setCursor(0, 0); lcd.print(F("PHASE 1: PRE-HEAT   "));
  lcd.setCursor(0, 1);
  char buf[21];
  sprintf(buf, "TIME: %s REM:%s", formatTime(elapsed).c_str(), formatTime(remaining).c_str());
  lcd.print(buf);
  lcd.setCursor(0, 2);
  char tbuf[21];
  sprintf(tbuf, "TEMP:%5.1f", currentTemp);
  lcd.print(tbuf); lcd.write(0); lcd.print(F("C SP:")); lcd.print(setpointTemp); lcd.write(0); lcd.print(F("C "));
  lcd.setCursor(0, 3);
  lcd.print(F("FAN:"));   lcd.print(fanOn     ? F("ON  ") : F("OFF "));
  lcd.print(F("VALVE:")); lcd.print(valveOpen ? F("OPEN ") : F("CLSD "));
}

void displayPhase2() {
  unsigned long elapsed   = millis() - phaseStartTime;
  unsigned long remaining = (elapsed < PHASE2_DURATION_MS) ? (PHASE2_DURATION_MS - elapsed) : 0;
  int irPct = (int)map((long)pidOutput, 0, 255, 0, 100);

  lcd.setCursor(0, 0); lcd.print(F("PHASE 2: ROASTING   "));
  lcd.setCursor(0, 1);
  char buf[21];
  sprintf(buf, "TIME: %s REM:%s", formatTime(elapsed).c_str(), formatTime(remaining).c_str());
  lcd.print(buf);
  lcd.setCursor(0, 2);
  char tbuf[21];
  sprintf(tbuf, "TEMP:%5.1f", currentTemp);
  lcd.print(tbuf); lcd.write(0); lcd.print(F("C SP:")); lcd.print(setpointTemp); lcd.write(0); lcd.print(F("C "));
  lcd.setCursor(0, 3);
  lcd.print(F("IR:"));
  char pbuf[4]; sprintf(pbuf, "%3d", irPct); lcd.print(pbuf);
  lcd.print(F("% FAN:")); lcd.print(fanOn ? F("ON  ") : F("OFF "));
}

void displayCooling() {
  lcd.setCursor(0, 0); lcd.print(F("    COOLING DOWN    "));
  lcd.setCursor(0, 1); lcd.print(F("                    "));
  lcd.setCursor(0, 2);
  char tbuf[21];
  sprintf(tbuf, "  TEMP: %5.1f", currentTemp);
  lcd.print(tbuf); lcd.write(0); lcd.print(F("C       "));
  lcd.setCursor(0, 3); lcd.print(F("  FAN: ON           "));
}

void displayDone() {
  unsigned long totalMs = (roastEndTime.unixtime() - roastStartTime.unixtime()) * 1000UL;

  lcd.setCursor(0, 0); lcd.print(F("   ROAST COMPLETE   "));
  lcd.setCursor(0, 1);
  char tbuf[21];
  sprintf(tbuf, "PEAK TEMP: %5.1f", peakTemp);
  lcd.print(tbuf); lcd.write(0); lcd.print(F("C "));
  lcd.setCursor(0, 2);
  char timebuf[21];
  sprintf(timebuf, "DURATION:   %s   ", formatTime(totalMs).c_str());
  lcd.print(timebuf);
  lcd.setCursor(0, 3); lcd.print(F(" PRESS TO RESTART   "));
}

void displayError() {
  lcd.setCursor(0, 0); lcd.print(F("!!!!  ERROR  !!!!   "));
  lcd.setCursor(0, 1); lcd.print(F("                    "));
  lcd.setCursor(0, 2);
  String padded = errorMessage;
  while (padded.length() < 20) padded += " ";
  lcd.print(padded.substring(0, 20));
  lcd.setCursor(0, 3); lcd.print(F(" PRESS TO RESET     "));
}

// ============================================================
//  SERIAL HELPERS
// ============================================================
void printSeparator() {
  Serial.println(F("=================================================="));
}

void printStateLabel(SystemState s) {
  switch (s) {
    case STATE_IDLE:           Serial.print(F("IDLE"));           break;
    case STATE_SET_SETPOINT:   Serial.print(F("SET_SETPOINT"));   break;
    case STATE_PHASE1_PREHEAT: Serial.print(F("PHASE1_PREHEAT")); break;
    case STATE_PHASE2_ROAST:   Serial.print(F("PHASE2_ROAST"));   break;
    case STATE_COOLING:        Serial.print(F("COOLING"));        break;
    case STATE_DONE:           Serial.print(F("DONE"));           break;
    case STATE_ERROR:          Serial.print(F("ERROR"));          break;
    default:                   Serial.print(F("UNKNOWN"));        break;
  }
}

void logTransition(SystemState from, SystemState to) {
  DateTime nowRtc = rtc.now();
  char timeBuf[9];
  sprintf(timeBuf, "%02d:%02d:%02d", nowRtc.hour(), nowRtc.minute(), nowRtc.second());

  printSeparator();
  Serial.print(F("["));
  Serial.print(timeBuf);
  Serial.print(F("] [TRANSITION] "));
  printStateLabel(from);
  Serial.print(F(" --> "));
  printStateLabel(to);
  Serial.print(F("  (uptime: "));
  Serial.print(millis() / 1000UL);
  Serial.println(F("s)"));
  printSeparator();
}

// ============================================================
//  SAFETY CHECK
// ============================================================
void checkSafety() {
  double raw = thermocouple.readCelsius();

  if (isnan(raw) || raw <= 0.0) {
    allOff();
    errorMessage = "  SENSOR FAULT!     ";
    SystemState prev = currentState;
    currentState = STATE_ERROR;
    lcdClear();
    printSeparator();
    Serial.println(F("[SAFETY] *** SENSOR FAULT — ALL OUTPUTS OFF ***"));
    logTransition(prev, currentState);
    return;
  }

  if (currentTemp > (pidSetpoint + OVERTEMP_MARGIN)) {
    allOff();
    errorMessage = "   OVER TEMP!!!     ";
    SystemState prev = currentState;
    currentState = STATE_ERROR;
    lcdClear();
    printSeparator();
    Serial.print(F("[SAFETY] *** OVER TEMP! TEMP="));
    Serial.print(currentTemp, 1);
    Serial.print(F("C  LIMIT="));
    Serial.print(pidSetpoint + OVERTEMP_MARGIN, 1);
    Serial.println(F("C — ALL OUTPUTS OFF ***"));
    logTransition(prev, currentState);
    return;
  }

  if (currentState == STATE_PHASE2_ROAST) {
    if (pidOutput >= 254 && currentTemp < (pidSetpoint - 20.0)) {
      if (!heaterFailArmed) {
        heaterFailArmed = true;
        heaterFailTimer = millis();
        Serial.println(F("[SAFETY] Heater fail watchdog ARMED (100% output, temp low)"));
      } else if (millis() - heaterFailTimer > HEATER_FAIL_TIME_MS) {
        allOff();
        errorMessage = "  HEATER FAILURE    ";
        SystemState prev = currentState;
        currentState = STATE_ERROR;
        lcdClear();
        printSeparator();
        Serial.println(F("[SAFETY] *** HEATER FAILURE! >2min at 100%, no temp rise — ALL OUTPUTS OFF ***"));
        logTransition(prev, currentState);
        return;
      }
    } else {
      if (heaterFailArmed) {
        Serial.println(F("[SAFETY] Heater fail watchdog DISARMED (temp recovered)"));
      }
      heaterFailArmed = false;
    }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(PIN_RELAY_VALVE, OUTPUT);
  pinMode(PIN_RELAY_FAN,   OUTPUT);
  pinMode(PIN_BUTTON,      INPUT_PULLUP);

  digitalWrite(PIN_RELAY_VALVE, HIGH);
  digitalWrite(PIN_RELAY_FAN,   HIGH);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, degChar);
  lcdClear();

  // RTC
  if (!rtc.begin()) {
    lcd.setCursor(0, 0); lcd.print(F("RTC NOT FOUND!"));
    Serial.println(F("[ERROR] RTC not found! Halting."));
    while (1);
  }

  // ── RTC lost-power: show single error line, block until fixed ──
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println(F("[WARN] RTC lost power — replace CR2032 battery."));

    lcdClear();
    lcd.setCursor(0, 1); lcd.print(F(" WARN: RTC NO POWER "));
    lcd.setCursor(0, 2); lcd.print(F(" REPLACE CR2032     "));

    while (rtc.lostPower()) { delay(500); }   // hold here until battery replaced

    Serial.println(F("[INFO] RTC power restored — continuing."));
    lcdClear();
  }

  // DimmableLight
  DimmableLight::setSyncPin(PIN_ZC);
  DimmableLight::begin();
  dimmer.setBrightness(0);

  // PID
  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(PID_SAMPLE_MS);

  // Splash
  lcdClear();
  lcd.setCursor(0, 0); lcd.print(F("  CACAO ROASTER     "));
  lcd.setCursor(0, 1); lcd.print(F("  Initializing...   "));

  // Boot banner
  printSeparator();
  Serial.println(F("      CACAO ROASTER CONTROLLER — BOOT      "));
  printSeparator();
  Serial.println(F("  Baud         : 9600"));
  Serial.print(  F("  Default SP   : ")); Serial.print(SETPOINT_DEFAULT);           Serial.println(F(" C"));
  Serial.print(  F("  Phase 1      : ")); Serial.print(PHASE1_DURATION_MS/60000UL); Serial.println(F(" min (LPG preheat)"));
  Serial.print(  F("  Phase 2      : ")); Serial.print(PHASE2_DURATION_MS/60000UL); Serial.println(F(" min (IR roast)"));
  Serial.print(  F("  Overtemp     : +")); Serial.print(OVERTEMP_MARGIN, 0);        Serial.println(F(" C above SP"));
  Serial.print(  F("  Cool target  : <=")); Serial.print(COOLING_TEMP_TARGET, 0);   Serial.println(F(" C"));
  Serial.print(  F("  PID Kp/Ki/Kd : "));
  Serial.print(PID_KP); Serial.print(F(" / "));
  Serial.print(PID_KI); Serial.print(F(" / "));
  Serial.println(PID_KD);
  Serial.println(F("  Dimmer lib   : DimmableLight (Fabiano Riccardi) v1.6+"));
  Serial.print(  F("  ZC pin       : D")); Serial.print(PIN_ZC); Serial.println(F(" (INT5)"));
  Serial.print(  F("  Gate pin     : D")); Serial.println(PIN_DIMMER);
  printSeparator();
  Serial.println(F("  FORMAT: [HH:MM:SS] STATE | TEMP | SP | PEAK | ..."));
  printSeparator();
  Serial.println();

  delay(1500);
  lcdClear();
  currentState = STATE_IDLE;
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();
  ButtonEvent   btn = readButton();

  if (now - lastTempReadTime >= 250) {
    lastTempReadTime = now;
    readTemperature();
  }

  if (currentState == STATE_PHASE1_PREHEAT ||
      currentState == STATE_PHASE2_ROAST   ||
      currentState == STATE_COOLING) {
    checkSafety();
    if (currentState == STATE_ERROR) return;
  }

  switch (currentState) {

    case STATE_IDLE:
      allOff();
      myPID.SetMode(MANUAL);
      peakTemp = 0.0;

      if (now - lastLCDTime >= 1000) {
        lastLCDTime = now;
        displayIdle();
      }

      if (btn == BTN_SHORT || btn == BTN_LONG) {
        setpointTemp = SETPOINT_DEFAULT;
        lcdClear();
        SystemState prev = currentState;
        currentState = STATE_SET_SETPOINT;
        logTransition(prev, currentState);
        Serial.print(F("[EVENT] Button → setpoint config. Default SP="));
        Serial.print(setpointTemp); Serial.println(F("C"));
      }
      break;

    case STATE_SET_SETPOINT:
      if (btn == BTN_SHORT) {
        setpointTemp += SETPOINT_STEP;
        if (setpointTemp > SETPOINT_MAX) setpointTemp = SETPOINT_MIN;
        displaySetpoint();
        Serial.print(F("[SETPOINT] Adjusted to: "));
        Serial.print(setpointTemp); Serial.println(F("C"));
      }

      if (btn == BTN_LONG) {
        pidSetpoint     = setpointTemp;
        phaseStartTime  = millis();
        roastStartTime  = rtc.now();
        heaterFailArmed = false;
        lcdClear();

        Serial.print(F("[SETPOINT] Confirmed: "));
        Serial.print(setpointTemp);
        Serial.println(F("C — starting Phase 1"));

        SystemState prev = currentState;
        currentState = STATE_PHASE1_PREHEAT;
        logTransition(prev, currentState);

        valveOpen_();
        heaterOff();

        Serial.println(F("[OUTPUT] LPG Valve: OPEN"));
        Serial.println(F("[OUTPUT] IR Heater: OFF (brightness=0)"));
        Serial.println(F("[OUTPUT] Fan: feedback mode"));
      }

      if (now - lastLCDTime >= 200) {
        lastLCDTime = now;
        displaySetpoint();
      }
      break;

    case STATE_PHASE1_PREHEAT:
      updateFanFeedback();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayPhase1();
      }

      if (now - phaseStartTime >= PHASE1_DURATION_MS) {
        valveClose();
        fanOff();
        Serial.println(F("[OUTPUT] Phase 1 done — LPG Valve: CLOSED, Fan: OFF"));
        Serial.println(F("[OUTPUT] Starting IR Heater + PID..."));

        pidInput = currentTemp;
        myPID.SetMode(AUTOMATIC);
        dimmer.setBrightness(0);
        phaseStartTime = millis();
        lcdClear();

        SystemState prev = currentState;
        currentState = STATE_PHASE2_ROAST;
        logTransition(prev, currentState);
      }

      if (btn == BTN_LONG) {
        Serial.println(F("[EVENT] Emergency stop in Phase 1 — cooling"));
        allOff();
        lcdClear();
        SystemState prev = currentState;
        currentState = STATE_COOLING;
        logTransition(prev, currentState);
      }
      break;

    case STATE_PHASE2_ROAST:
      pidInput = currentTemp;
      if (myPID.Compute()) {
        heaterSetPower((uint8_t)pidOutput);
      }

      updateFanFeedback();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayPhase2();
      }

      if (now - phaseStartTime >= PHASE2_DURATION_MS) {
        roastEndTime = rtc.now();
        allOff();
        fanOn_();
        Serial.println(F("[OUTPUT] Phase 2 done — Heater OFF, Fan ON (cooling)"));
        lcdClear();
        SystemState prev = currentState;
        currentState = STATE_COOLING;
        logTransition(prev, currentState);
      }

      if (btn == BTN_LONG) {
        roastEndTime = rtc.now();
        allOff();
        fanOn_();
        Serial.println(F("[EVENT] Manual stop in Phase 2 — cooling"));
        lcdClear();
        SystemState prev = currentState;
        currentState = STATE_COOLING;
        logTransition(prev, currentState);
      }
      break;

    case STATE_COOLING:
      heaterOff();
      valveClose();
      fanOn_();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayCooling();
      }

      if (currentTemp <= COOLING_TEMP_TARGET) {
        fanOff();
        Serial.print(F("[COOLING] Target reached (<="));
        Serial.print(COOLING_TEMP_TARGET, 0);
        Serial.print(F("C). Temp="));
        Serial.print(currentTemp, 1);
        Serial.println(F("C — Fan OFF"));
        lcdClear();
        SystemState prev = currentState;
        currentState = STATE_DONE;
        logTransition(prev, currentState);
      }
      break;

    case STATE_DONE:
      allOff();

      if (now - lastLCDTime >= 1000) {
        lastLCDTime = now;
        displayDone();
      }

      if (btn != BTN_NONE) {
        Serial.println(F("[EVENT] Button → IDLE"));
        lcdClear();
        SystemState prev = currentState;
        currentState = STATE_IDLE;
        logTransition(prev, currentState);
      }
      break;

    case STATE_ERROR:
      allOff();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayError();
      }

      if (btn != BTN_NONE) {
        Serial.println(F("[EVENT] Button → reset to IDLE"));
        SystemState prev = currentState;
        errorMessage = "";
        lcdClear();
        currentState = STATE_IDLE;
        logTransition(prev, currentState);
      }
      break;
  }

  // ── SERIAL: PERIODIC STATUS (every 1s) ────────────────────
  if (now - lastPIDTime >= 1000) {
    lastPIDTime = now;

    DateTime nowRtc = rtc.now();
    char timeBuf[9];
    sprintf(timeBuf, "%02d:%02d:%02d",
            nowRtc.hour(), nowRtc.minute(), nowRtc.second());

    Serial.print(F("["));        Serial.print(timeBuf);      Serial.print(F("] STATE:"));
    printStateLabel(currentState);

    Serial.print(F(" | TEMP:")); Serial.print(currentTemp, 1); Serial.print(F("C"));
    Serial.print(F(" | SP:"));   Serial.print(pidSetpoint, 0); Serial.print(F("C"));
    Serial.print(F(" | PEAK:")); Serial.print(peakTemp, 1);    Serial.print(F("C"));

    if (currentState == STATE_PHASE1_PREHEAT) {
      unsigned long el  = now - phaseStartTime;
      unsigned long rem = (el < PHASE1_DURATION_MS) ? (PHASE1_DURATION_MS - el) : 0;
      Serial.print(F(" | ELAP:")); Serial.print(el  / 1000UL); Serial.print(F("s"));
      Serial.print(F(" REM:"));    Serial.print(rem / 1000UL); Serial.print(F("s"));
      Serial.print(F(" | VALVE:")); Serial.print(valveOpen ? F("OPEN") : F("CLSD"));
    }

    if (currentState == STATE_PHASE2_ROAST) {
      unsigned long el  = now - phaseStartTime;
      unsigned long rem = (el < PHASE2_DURATION_MS) ? (PHASE2_DURATION_MS - el) : 0;
      int irPct = (int)map((long)pidOutput, 0, 255, 0, 100);
      Serial.print(F(" | ELAP:"));    Serial.print(el  / 1000UL); Serial.print(F("s"));
      Serial.print(F(" REM:"));       Serial.print(rem / 1000UL); Serial.print(F("s"));
      Serial.print(F(" | PID_OUT:")); Serial.print(pidOutput, 0);
      Serial.print(F(" BRIGHT:"));    Serial.print((uint8_t)pidOutput);
      Serial.print(F(" IR:"));        Serial.print(irPct); Serial.print(F("%"));
    }

    if (currentState == STATE_COOLING) {
      Serial.print(F(" | TARGET:<="));
      Serial.print(COOLING_TEMP_TARGET, 0);
      Serial.print(F("C"));
    }

    Serial.print(F(" | FAN:"));   Serial.print(fanOn    ? F("ON")   : F("OFF"));
    Serial.print(F(" | VALVE:")); Serial.print(valveOpen ? F("OPEN") : F("CLSD"));

    if (currentState == STATE_ERROR) {
      Serial.print(F(" | ERR:[")); Serial.print(errorMessage); Serial.print(F("]"));
    }

    Serial.println();
  }
}
