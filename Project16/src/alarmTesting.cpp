#include <Arduino.h>

const int BUZZ_SW = 25;        // Buzzer/MOSFET control pin

// --- Alarm thresholds ---
const float TEMP_THRESHOLD_F   = 85.0;  // °F
const float CURRENT_THRESHOLD_A = 0.4;  // A (400 mA)

// --- Non-blocking beep pattern (200 ms ON, 400 ms OFF) ---
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 400;

bool alarmActive = false;
bool buzzerState = false;
unsigned long phaseStart = 0;

// Decide if alarm should sound
inline bool shouldAlarm(float tempF, float currentA) {
  return (tempF > TEMP_THRESHOLD_F) && (currentA > CURRENT_THRESHOLD_A);
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
float fakeCurrentA = 0.0;

void loop() {
  // TODO: replace with your real sensor values:
  // float tempF = <your DallasTemperature in °F>;
  // float currentA = <your ACS712 reading in Amps>;
  // For demo, we slowly increase until thresholds are crossed:
  fakeTempF += 0.2;       // simulate warming
  fakeCurrentA += 0.02;   // simulate current rising

  float tempF = fakeTempF;
  float currentA = fakeCurrentA;

  alarmActive = shouldAlarm(tempF, currentA);
  updateAlarmBuzzer(alarmActive);

  // Optional serial debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Temp: "); Serial.print(tempF, 1); Serial.print(" F,  ");
    Serial.print("Current: "); Serial.print(currentA, 3); Serial.print(" A,  ");
    Serial.print("Alarm: "); Serial.println(alarmActive ? "ON" : "OFF");
  }

  // No delay() needed — stays responsive
}
