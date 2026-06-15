/*
 * ============================================================
 *  CACAO ROASTER CONTROLLER
 *  Hardware: Arduino Mega
 *  Components:
 *    - MAX6675 K-Type Thermocouple
 *    - DS3231 RTC
 *    - I2C LCD 16x2
 *    - Relay 3 → IR Heater (ON/OFF only, controlled by PID)
 *    - Relay 1 → LPG Solenoid Valve
 *    - Relay 2 → Tempering Fan (for cooling & overshoot prevention)
 *    - Single Push Button (START/PAUSE/CONTINUE)
 * ============================================================
 *
 *  PIN ASSIGNMENTS:
 *    D15 → Relay 3 (IR Heater)           — Active LOW
 *    D53 → MAX6675 CS
 *    D52 → MAX6675 SCK
 *    D50 → MAX6675 SO (MISO)
 *    D14 → Relay 1 (LPG Solenoid Valve) — Active LOW
 *    D48 → Relay 2 (Fan)                — Active LOW
 *    D42 → Button (INPUT_PULLUP) — START/PAUSE/CONTINUE
 *    A4  → I2C SDA (LCD + RTC)
 *    A5  → I2C SCL (LCD + RTC)
 *
 *  LIBRARIES REQUIRED:
 *    - Wire.h               (built-in)
 *    - LiquidCrystal_I2C.h  (Frank de Brabander)
 *    - max6675.h            (Adafruit)
 *    - RTClib.h             (Adafruit)
 *    - PID_v1.h             (Brett Beauregard)
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>
#include <RTClib.h>
#include <PID_v1.h>

// ─── PIN DEFINITIONS ────────────────────────────────────────
#define PIN_HEATER       15    // IR Heater relay (Active LOW)
#define PIN_TC_CS        53    // MAX6675 Chip Select
#define PIN_TC_SCK       52    // MAX6675 Clock
#define PIN_TC_SO        50    // MAX6675 Data Out
#define PIN_RELAY_VALVE 14    // LPG Solenoid Valve relay (Active LOW)
#define PIN_RELAY_FAN   48    // Tempering Fan relay (Active LOW)
#define PIN_BUTTON      42    // Single push button

// ─── PHASE SETPOINTS ────────────────────────────────────────
#define PHASE1_SETPOINT     100.0   // Pre-heating target
#define PHASE1_END_TEMP     100.0   // End Phase 1 when reached
#define PHASE2_SETPOINT     110.0   // Roasting target
#define PHASE3_SETPOINT     110.0   // Tempering (no heating, just timer)

// ─── PHASE TIMERS ───────────────────────────────────────────
#define PHASE1_TIMEOUT_MS   1200000UL  // 20 min max (safety limit, ends when 100°C reached)
#define PHASE2_DURATION_MS  1800000UL  // 30 min roasting
#define PHASE3_DURATION_MS  600000UL   // 10 min tempering

// ─── PID SETTINGS (for ON/OFF control) ──────────────────────
#define PID_KP              3.0
#define PID_KI              0.1
#define PID_KD              2.0
#define PID_SAMPLE_MS       250

// ─── HEATER ON/OFF THRESHOLD ────────────────────────────────
#define HEATER_THRESHOLD    127

// ─── FAN CONTROL (cooling & overshoot prevention) ───────────
#define FAN_OVERSHOOT_MARGIN    3.0    // Turn fan ON if temp > setpoint + 3°C
#define FAN_HYSTERESIS          1.0    // Turn fan OFF if temp < setpoint + 3°C - 1°C

// ─── SAFETY LIMITS ──────────────────────────────────────────
#define OVERTEMP_MARGIN     20.0
#define HEATER_FAIL_TIME_MS 120000UL

// ─── BUTTON TIMING ──────────────────────────────────────────
#define BTN_DEBOUNCE_MS     50
#define BTN_LONG_PRESS_MS   1500

// ─── OBJECTS ────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX6675           thermocouple(PIN_TC_SCK, PIN_TC_CS, PIN_TC_SO);
RTC_DS3231        rtc;

// ─── PID VARIABLES ──────────────────────────────────────────
double pidInput    = 0.0;
double pidOutput   = 0.0;
double pidSetpoint = PHASE1_SETPOINT;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// ─── STATE MACHINE ──────────────────────────────────────────
enum SystemState {
  STATE_IDLE,
  STATE_PHASE1_PREHEAT,
  STATE_PHASE2_ROAST,
  STATE_PHASE3_TEMPER,
  STATE_PAUSED,
  STATE_DONE,
  STATE_ERROR
};

enum ButtonEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

// ─── GLOBAL STATE ───────────────────────────────────────────
SystemState currentState = STATE_IDLE;
SystemState pausedFromState = STATE_IDLE;  // Track which state we paused from
String      errorMessage = "";

// ─── RUNTIME VARIABLES ──────────────────────────────────────
double currentTemp   = 0.0;
double peakTemp      = 0.0;
bool   fanOn         = false;
bool   valveOpen     = false;
bool   heaterOn      = false;

unsigned long phaseStartTime   = 0;
unsigned long pausedTime       = 0;      // When paused, store remaining time
unsigned long lastPIDTime      = 0;
unsigned long lastTempReadTime = 0;
unsigned long lastLCDTime      = 0;
unsigned long heaterFailTimer  = 0;
bool          heaterFailArmed  = false;

// ─── BUTTON VARIABLES ───────────────────────────────────────
bool          btnLastState     = HIGH;
bool          btnCurrentState  = HIGH;
unsigned long btnPressTime     = 0;
bool          longPressHandled = false;

// ============================================================
//  RELAY HELPERS (Active LOW)
// ============================================================
void valveOpen_() { digitalWrite(PIN_RELAY_VALVE, LOW);  valveOpen = true;  }
void valveClose() { digitalWrite(PIN_RELAY_VALVE, HIGH); valveOpen = false; }
void fanOn_()     { digitalWrite(PIN_RELAY_FAN,   LOW);  fanOn     = true;  }
void fanOff()     { digitalWrite(PIN_RELAY_FAN,   HIGH); fanOn     = false; }
void heaterOn_()  { digitalWrite(PIN_HEATER, LOW);  heaterOn = true;  }
void heaterOff()  { digitalWrite(PIN_HEATER, HIGH); heaterOn = false; }

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
//  HEATER CONTROL (PID-based ON/OFF)
// ============================================================
void updateHeater() {
  if (pidOutput > HEATER_THRESHOLD) {
    heaterOn_();
  } else {
    heaterOff();
  }
}

// ============================================================
//  FAN CONTROL (overshoot prevention + cooling)
// ============================================================
void updateFan() {
  if (currentTemp >= (pidSetpoint + FAN_OVERSHOOT_MARGIN)) {
    fanOn_();
  } 
  else if (currentTemp < (pidSetpoint + FAN_OVERSHOOT_MARGIN - FAN_HYSTERESIS)) {
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
//  LCD DISPLAY (16x2)
// ============================================================
void displayPhase(uint8_t phase, unsigned long elapsedMs, unsigned long totalMs) {
  char line1[17], line2[17];
  
  // Calculate remaining time
  unsigned long remainingMs = (elapsedMs < totalMs) ? (totalMs - elapsedMs) : 0;
  unsigned long remainingSecs = remainingMs / 1000;
  unsigned long remainingMins = remainingSecs / 60;
  remainingSecs = remainingSecs % 60;

  // Line 1: PHASE:X TM:MM:SS
  sprintf(line1, "PHASE:%d TM:%02lu:%02lu", phase, remainingMins, remainingSecs);
  
  // Line 2: SP:XXX CT:XXX.X
  // FIX 1: Manual conversion for floating point (sprintf doesn't support %f on Arduino)
  int spInt = (int)pidSetpoint;
  int ctInt = (int)currentTemp;
  int ctDec = (int)((currentTemp - ctInt) * 10);
  if (ctDec < 0) ctDec = -ctDec;  // Handle negative decimal part
  sprintf(line2, "SP:%3d CT:%3d.%1d", spInt, ctInt, ctDec);

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void displayIdle() {
  lcd.setCursor(0, 0);
  lcd.print("CACAO  ROASTER");
  lcd.setCursor(0, 1);
  lcd.print("PRESS TO START");
}

void displayPaused() {
  lcd.setCursor(0, 0);
  lcd.print("*** PAUSED ***");
  lcd.setCursor(0, 1);
  lcd.print("PRESS CONTINUE");
}

void displayError() {
  lcd.setCursor(0, 0);
  lcd.print("!!! ERROR !!!");
  lcd.setCursor(0, 1);
  String padded = errorMessage;
  while (padded.length() < 16) padded += " ";
  lcd.print(padded.substring(0, 16));
}

void displayDone() {
  char line1[17], line2[17];
  int peakInt = (int)peakTemp;
  int peakDec = (int)((peakTemp - peakInt) * 10);
  if (peakDec < 0) peakDec = -peakDec;  // Handle negative decimal part
  sprintf(line1, "PEAK:%3d.%1d", peakInt, peakDec);
  sprintf(line2, "DONE-PRESS RST");
  
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
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
    case STATE_PHASE1_PREHEAT: Serial.print(F("PHASE1_PREHEAT")); break;
    case STATE_PHASE2_ROAST:   Serial.print(F("PHASE2_ROAST"));   break;
    case STATE_PHASE3_TEMPER:  Serial.print(F("PHASE3_TEMPER"));  break;
    case STATE_PAUSED:         Serial.print(F("PAUSED"));         break;
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
  Serial.println(F("]"));
  printSeparator();
}

// ============================================================
//  SAFETY CHECK
// ============================================================
void checkSafety() {
  double raw = thermocouple.readCelsius();

  if (isnan(raw) || raw <= 0.0) {
    allOff();
    errorMessage = "SENSOR FAULT!";
    SystemState prev = currentState;
    currentState = STATE_ERROR;
    lcd.clear();
    printSeparator();
    Serial.println(F("[SAFETY] *** SENSOR FAULT — ALL OUTPUTS OFF ***"));
    logTransition(prev, currentState);
    return;
  }

  if (currentTemp > (pidSetpoint + OVERTEMP_MARGIN)) {
    allOff();
    errorMessage = "OVER TEMP!!!";
    SystemState prev = currentState;
    currentState = STATE_ERROR;
    lcd.clear();
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
    if (heaterOn && currentTemp < (pidSetpoint - 20.0)) {
      if (!heaterFailArmed) {
        heaterFailArmed = true;
        heaterFailTimer = millis();
        Serial.println(F("[SAFETY] Heater fail watchdog ARMED (heater ON, temp low)"));
      } else if (millis() - heaterFailTimer > HEATER_FAIL_TIME_MS) {
        allOff();
        errorMessage = "HEATER FAILURE";
        SystemState prev = currentState;
        currentState = STATE_ERROR;
        lcd.clear();
        printSeparator();
        Serial.println(F("[SAFETY] *** HEATER FAILURE! >2min ON, no temp rise — ALL OUTPUTS OFF ***"));
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

  pinMode(PIN_HEATER,       OUTPUT);
  pinMode(PIN_RELAY_VALVE, OUTPUT);
  pinMode(PIN_RELAY_FAN,   OUTPUT);
  pinMode(PIN_BUTTON,      INPUT_PULLUP);

  // FIX 2: All relays start LOW (OFF for Active LOW relays)
  digitalWrite(PIN_HEATER,       LOW);
  digitalWrite(PIN_RELAY_VALVE, LOW);
  digitalWrite(PIN_RELAY_FAN,   LOW);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // RTC
  if (!rtc.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("RTC NOT FOUND!");
    Serial.println(F("[ERROR] RTC not found! Halting."));
    while (1);
  }

  // ── RTC lost-power ──
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println(F("[WARN] RTC lost power — replace CR2032 battery."));
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WARN: RTC NO PWR");
    lcd.setCursor(0, 1);
    lcd.print("Replace CR2032");
    
    while (rtc.lostPower()) { delay(500); }
    
    Serial.println(F("[INFO] RTC power restored — continuing."));
    lcd.clear();
  }

  // PID
  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(PID_SAMPLE_MS);

  // Boot banner
  printSeparator();
  Serial.println(F("      CACAO ROASTER CONTROLLER — BOOT      "));
  printSeparator();
  Serial.println(F("  Baud         : 9600"));
  Serial.print(  F("  Phase 1 SP   : ")); Serial.print(PHASE1_SETPOINT);   Serial.println(F(" C (until 100C reached)"));
  Serial.print(  F("  Phase 2 SP   : ")); Serial.print(PHASE2_SETPOINT);   Serial.println(F(" C for 30 min"));
  Serial.print(  F("  Phase 3 SP   : ")); Serial.print(PHASE3_SETPOINT);   Serial.println(F(" C for 10 min (heater OFF)"));
  Serial.print(  F("  PID Kp/Ki/Kd : "));
  Serial.print(PID_KP); Serial.print(F(" / "));
  Serial.print(PID_KI); Serial.print(F(" / "));
  Serial.println(PID_KD);
  Serial.println(F("  Heater ctrl  : Digital ON/OFF (PID-based)"));
  Serial.println(F("  Button       : START/PAUSE/CONTINUE (all halts when paused)"));
  Serial.println(F("  Relay init   : All relays start LOW (OFF)"));
  printSeparator();
  Serial.println();

  delay(1500);
  lcd.clear();
  displayIdle();
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
      currentState == STATE_PHASE3_TEMPER) {
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

      if (btn == BTN_SHORT) {
        // Start roasting process
        phaseStartTime = millis();
        pidSetpoint = PHASE1_SETPOINT;
        myPID.SetMode(MANUAL);
        heaterFailArmed = false;
        peakTemp = 0.0;
        
        lcd.clear();
        SystemState prev = currentState;
        currentState = STATE_PHASE1_PREHEAT;
        logTransition(prev, currentState);
        
        valveOpen_();
        heaterOff();
        
        Serial.println(F("[OUTPUT] Phase 1: LPG Valve OPEN, Heater OFF"));
        Serial.println(F("[OUTPUT] Waiting to reach 100C..."));
      }
      break;

    case STATE_PHASE1_PREHEAT: {
      unsigned long elapsedMs = now - phaseStartTime;
      
      updateFan();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayPhase(1, elapsedMs, PHASE1_TIMEOUT_MS);
      }

      // End Phase 1 when 100°C is reached
      if (currentTemp >= PHASE1_END_TEMP) {
        valveClose();
        fanOff();
        Serial.println(F("[OUTPUT] Phase 1 done — 100C reached!"));
        Serial.println(F("[OUTPUT] LPG Valve: CLOSED"));
        Serial.println(F("[OUTPUT] Starting Phase 2: Roasting with PID control..."));

        pidSetpoint = PHASE2_SETPOINT;
        pidInput = currentTemp;
        myPID.SetMode(AUTOMATIC);
        heaterOff();
        phaseStartTime = millis();
        heaterFailArmed = false;
        lastLCDTime = now;
        lcd.clear();

        SystemState prev = currentState;
        currentState = STATE_PHASE2_ROAST;
        logTransition(prev, currentState);
      }

      // Safety: timeout if Phase 1 takes too long
      if (elapsedMs >= PHASE1_TIMEOUT_MS) {
        Serial.println(F("[SAFETY] Phase 1 timeout!"));
        allOff();
        lcd.clear();
        SystemState prev = currentState;
        currentState = STATE_ERROR;
        errorMessage = "PHASE1 TIMEOUT";
        logTransition(prev, currentState);
      }

      // Pause button
      if (btn == BTN_SHORT) {
        Serial.println(F("[EVENT] Paused in Phase 1"));
        pausedFromState = currentState;
        pausedTime = elapsedMs;
        allOff();
        lcd.clear();
        SystemState prev = currentState;
        currentState = STATE_PAUSED;
        logTransition(prev, currentState);
      }
      break;
    }

    case STATE_PHASE2_ROAST: {
      unsigned long elapsedMs = now - phaseStartTime;
      
      pidInput = currentTemp;
      if (myPID.Compute()) {
        updateHeater();
      }

      updateFan();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayPhase(2, elapsedMs, PHASE2_DURATION_MS);
      }

      // End Phase 2 after 30 minutes
      if (elapsedMs >= PHASE2_DURATION_MS) {
        heaterOff();
        Serial.println(F("[OUTPUT] Phase 2 done — 30 min roasting complete!"));
        Serial.println(F("[OUTPUT] Starting Phase 3: Tempering for 10 min..."));
        Serial.println(F("[OUTPUT] Heater: OFF, Fan continues"));

        pidSetpoint = PHASE3_SETPOINT;
        heaterOff();
        phaseStartTime = millis();
        lastLCDTime = now;
        lcd.clear();

        SystemState prev = currentState;
        currentState = STATE_PHASE3_TEMPER;
        logTransition(prev, currentState);
      }

      // Pause button
      if (btn == BTN_SHORT) {
        Serial.println(F("[EVENT] Paused in Phase 2"));
        pausedFromState = currentState;
        pausedTime = elapsedMs;
        allOff();
        lcd.clear();
        SystemState prev = currentState;
        currentState = STATE_PAUSED;
        logTransition(prev, currentState);
      }
      break;
    }

    case STATE_PHASE3_TEMPER: {
      unsigned long elapsedMs = now - phaseStartTime;
      
      heaterOff();
      updateFan();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayPhase(3, elapsedMs, PHASE3_DURATION_MS);
      }

      // End Phase 3 after 10 minutes
      if (elapsedMs >= PHASE3_DURATION_MS) {
        allOff();
        Serial.println(F("[OUTPUT] Phase 3 done — Tempering complete!"));
        Serial.println(F("[OUTPUT] All outputs OFF"));
        lcd.clear();

        SystemState prev = currentState;
        currentState = STATE_DONE;
        logTransition(prev, currentState);
      }

      // Pause button
      if (btn == BTN_SHORT) {
        Serial.println(F("[EVENT] Paused in Phase 3"));
        pausedFromState = currentState;
        pausedTime = elapsedMs;
        allOff();
        lcd.clear();
        SystemState prev = currentState;
        currentState = STATE_PAUSED;
        logTransition(prev, currentState);
      }
      break;
    }

    case STATE_PAUSED:
      // All outputs stay OFF (already turned off in previous state)
      
      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayPaused();
      }

      // Resume button
      if (btn == BTN_SHORT) {
        Serial.println(F("[EVENT] Resumed"));
        phaseStartTime = now - pausedTime;  // Adjust start time to continue timer
        lastLCDTime = now;
        lcd.clear();
        
        SystemState prev = currentState;
        currentState = pausedFromState;
        logTransition(prev, currentState);
      }

      // Emergency reset (long press)
      if (btn == BTN_LONG) {
        Serial.println(F("[EVENT] Emergency reset"));
        allOff();
        peakTemp = 0.0;
        lcd.clear();
        
        SystemState prev = currentState;
        currentState = STATE_IDLE;
        logTransition(prev, currentState);
        displayIdle();
      }
      break;

    case STATE_DONE:
      allOff();

      if (now - lastLCDTime >= 1000) {
        lastLCDTime = now;
        displayDone();
      }

      // Reset button
      if (btn == BTN_SHORT) {
        Serial.println(F("[EVENT] Button → IDLE"));
        peakTemp = 0.0;
        lcd.clear();
        
        SystemState prev = currentState;
        currentState = STATE_IDLE;
        logTransition(prev, currentState);
        displayIdle();
      }
      break;

    case STATE_ERROR:
      allOff();

      if (now - lastLCDTime >= 500) {
        lastLCDTime = now;
        displayError();
      }

      // Reset button
      if (btn == BTN_SHORT) {
        Serial.println(F("[EVENT] Button → reset to IDLE"));
        errorMessage = "";
        peakTemp = 0.0;
        lcd.clear();
        
        SystemState prev = currentState;
        currentState = STATE_IDLE;
        logTransition(prev, currentState);
        displayIdle();
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

    Serial.print(F("["));
    Serial.print(timeBuf);
    Serial.print(F("] STATE:"));
    printStateLabel(currentState);

    Serial.print(F(" | TEMP:"));
    Serial.print(currentTemp, 1);
    Serial.print(F("C | SP:"));
    Serial.print(pidSetpoint, 0);
    Serial.print(F("C | PEAK:"));
    Serial.print(peakTemp, 1);
    Serial.print(F("C"));

    if (currentState == STATE_PHASE1_PREHEAT || 
        currentState == STATE_PHASE2_ROAST || 
        currentState == STATE_PHASE3_TEMPER) {
      unsigned long el = now - phaseStartTime;
      Serial.print(F(" | ELAPSED:"));
      Serial.print(el / 1000UL);
      Serial.print(F("s"));
    }

    Serial.print(F(" | HTR:"));
    Serial.print(heaterOn ? F("ON") : F("OFF"));
    Serial.print(F(" | FAN:"));
    Serial.print(fanOn ? F("ON") : F("OFF"));
    Serial.print(F(" | VLV:"));
    Serial.print(valveOpen ? F("OPEN") : F("CLSD"));

    if (currentState == STATE_ERROR) {
      Serial.print(F(" | ERR:["));
      Serial.print(errorMessage);
      Serial.print(F("]"));
    }

    Serial.println();
  }
}
