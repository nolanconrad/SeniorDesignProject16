#include <Arduino.h>

const int BUZZ_SW = 25;        // Buzzer/MOSFET control pin

// --- Alarm thresholds ---
const float TEMP_THRESHOLD_F   = 85.0;  // °F
const unsigned long TEMP_ALARM_DELAY_MS = 3000; // must stay above threshold for 3s

// --- Non-blocking beep pattern (200 ms ON, 400 ms OFF) ---
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 400;

bool alarmActive = false;
bool buzzerState = false;
unsigned long phaseStart = 0;

// timer to track how long temp has been above threshold
unsigned long tempHighStart = 0;
bool tempHigh = false;

// Decide if alarm should sound (with delay logic)
bool shouldAlarm(float tempF) {
  unsigned long now = millis();

  if (tempF > TEMP_THRESHOLD_F) {
    if (!tempHigh) {
      tempHigh = true;
      tempHighStart = now;  // start timing
    }
    // if high long enough, activate alarm
    if (now - tempHighStart >= TEMP_ALARM_DELAY_MS) {
      return true;
    }
  } else {
    // reset timer if temp drops below threshold
    tempHigh = false;
    tempHighStart = 0;
  }
  return false;
}

// Drive the buzzer without delay()
void updateAlarmBuzzer(bool active) {
  if (!active) {
    buzzerState = false;
    digitalWrite(BUZZ_SW, LOW);
    return;
  }

  unsigned long now = millis();
  unsigned long phaseLen = buzzerState ? BEEP_ON_MS : BEEP_OFF_MS;

  if (now - phaseStart >= phaseLen) {
    buzzerState = !buzzerState;
    phaseStart = now;
    digitalWrite(BUZZ_SW, buzzerState ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW);
}

// --- Demo inputs (replace with your real readings) ---
float fakeTempF = 80.0;

void loop() {
  // Simulate rising temperature
  fakeTempF += 0.2;  // for demo
  float tempF = fakeTempF;

  alarmActive = shouldAlarm(tempF);
  updateAlarmBuzzer(alarmActive);

  // Optional serial debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Temp: "); Serial.print(tempF, 1); Serial.print(" F,  ");
    Serial.print("Alarm: "); Serial.println(alarmActive ? "ON" : "OFF");
  }

  // No delay() needed — remains responsive
}
