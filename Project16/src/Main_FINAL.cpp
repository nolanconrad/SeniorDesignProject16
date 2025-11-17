#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <elapsedMillis.h>

#define ONE_WIRE_BUS 4

constexpr int IN1_PIN = 23;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

// current sensor setup
int sensorPin = 32;
const float sensitivity = 0.066;
const float adcMax = 4095.0;
const float adcVoltageRange = 3.9;
const float zeroOffset = 2.5;

// alarm buzzer
const int BUZZ_SW = 18;

// timers
elapsedMillis tempCheck_timer, lcdPrint_timer, currentCheck_timer, alarmCheck_timer, motorCheck_timer;
const uint32_t tempCheck_MS = 1000;
const uint32_t lcdPrint_MS = 2000;
const uint32_t currentCheck_MS = 3000;
const uint32_t alarmCheck_MS = 100;
const uint32_t motorCheck_MS = 5000;

boolean motorState = false;
boolean alarmState = false;
float currentValue = 0.0;

// === Alarm (3s persistence + 1s cooldown) ===
const float TEMP_THRESHOLD_F = 85.0;
const unsigned long TEMP_ALARM_DELAY_MS = 3000;
const unsigned long TEMP_ALARM_COOLDOWN_MS = 1000;
const unsigned long BEEP_ON_MS = 200;
const unsigned long BEEP_OFF_MS = 400;

bool buzzerState = false;
unsigned long phaseStart = 0;
unsigned long tempHighStart = 0;
unsigned long tempLowStart = 0;
bool tempHigh = false;
float latestTempF = 0.0;

bool shouldTempAlarm(float tempF) {
  unsigned long now = millis();

  if (tempF >= TEMP_THRESHOLD_F) {
    tempLowStart = 0; // reset cooldown timer
    if (!tempHigh) {
      tempHigh = true;
      tempHighStart = now;
    }
    // stays above for 3 seconds
    if (now - tempHighStart >= TEMP_ALARM_DELAY_MS) {
      return true;
    }
  } else {
    tempHigh = false;
    tempHighStart = 0;
    // start cooldown timer
    if (tempLowStart == 0) tempLowStart = now;
    // stay below for 1 second before turning off
    if (now - tempLowStart >= TEMP_ALARM_COOLDOWN_MS) {
      return false;
    } else {
      // still within cooldown period — keep alarm active
      return true;
    }
  }
  return false;
}

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
  sensors.begin();
  sensors.setWaitForConversion(false);
  lcd.begin(16, 2);
  lcd.clear();

  printLine(0, "System Init...");
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  delay(5000);
  lcd.clear();

  printLine(0, "System Ready!");
  printLine(1, "Pump: OFF");
  delay(3000);
  lcd.clear();

  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW);

  analogSetPinAttenuation(sensorPin, ADC_11db);
}

void printLine(int row, const String &text) {
  lcd.setCursor(0, row);
  lcd.print(text);
}

void tempCheck_task() {
  sensors.requestTemperatures();
  float c1 = 0, c2 = 0;
  int count = sensors.getDeviceCount();

  if (count >= 1) c2 = sensors.getTempCByIndex(count - 1);
  if (count >= 2) c1 = sensors.getTempCByIndex(count - 2);

  float f1 = (c1 * 9.0 / 5.0) + 32.0;
  float f2 = (c2 * 9.0 / 5.0) + 32.0;
  latestTempF = max(f1, f2);

  printLine(0, "1:" + String(f1, 1) + "F 2:" + String(f2, 1) + "F");
  printLine(1, "I:" + String(currentValue, 4) + " A");

  if (f1 >= 82) motorState = true; else motorState = false;
  if (f2 >= 82) motorState = false;
}

void currentCheck_task() {
  const int SAMPLES = 10;
  long sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += analogRead(sensorPin);
    delay(2);
  }
  float adcAvg = (float)sum / SAMPLES;
  float voltage = (adcAvg / adcMax) * adcVoltageRange;
  currentValue = (voltage - zeroOffset) / sensitivity;
}

void alarmCheck_task() {
  bool wantAlarm = shouldTempAlarm(latestTempF);
  alarmState = wantAlarm;
  updateAlarmBuzzer(wantAlarm);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("Temp: ");
    Serial.print(latestTempF, 1);
    Serial.print(" F | Alarm: ");
    Serial.println(wantAlarm ? "ON" : "OFF");
  }
}

void motorCheck_task() {
  if (motorState) digitalWrite(IN1_PIN, HIGH);
  else digitalWrite(IN1_PIN, LOW);
}

void lcdPrintTask() {}

void loop() {
  if (tempCheck_timer >= tempCheck_MS) { tempCheck_timer = 0; tempCheck_task(); }
  if (lcdPrint_timer >= lcdPrint_MS) { lcdPrint_timer = 0; lcdPrintTask(); }
  if (currentCheck_timer >= currentCheck_MS) { currentCheck_timer = 0; currentCheck_task(); }
  if (alarmCheck_timer >= alarmCheck_MS) { alarmCheck_timer = 0; alarmCheck_task(); }
  if (motorCheck_timer >= motorCheck_MS) { motorCheck_timer = 0; motorCheck_task(); }
}
