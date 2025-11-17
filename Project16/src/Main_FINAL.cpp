#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <elapsedMillis.h>

#define ONE_WIRE_BUS 4            // D4 = GPIO4 on your board
constexpr int IN1_PIN = 23;       // motor driver IN1
const int BUZZ_SW = 18;           // buzzer switch pin

// --- OneWire / DS18B20 ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- LCD pins (RS, EN, D4, D5, D6, D7) ---
LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

// --- Current sensor (ACS712-30A style) ---
int sensorPin = 32;
const float sensitivity = 0.066;     // 66 mV/A
const float adcMax = 4095.0;         // 12-bit ADC
const float adcVoltageRange = 3.9;   // with ADC_11db attenuation on ESP32
const float zeroOffset = 2.5;        // 0A output ~2.5V when powered from 5V

// --- Timers ---
elapsedMillis tempCheck_timer, lcdPrint_timer, currentCheck_timer, alarmCheck_timer, motorCheck_timer;
const uint32_t tempCheck_MS   = 1000;
const uint32_t lcdPrint_MS    = 2000;
const uint32_t currentCheck_MS= 3000;
const uint32_t alarmCheck_MS  = 100;   // fast for buzzer cadence
const uint32_t motorCheck_MS  = 5000;

boolean motorState = false;
boolean alarmState = false;
float currentValue = 0.0;   // amps (from first code)
float AcsValueF    = 0.0;   // shown on LCD (mirrors currentValue)

// === Temperature alarm (from first code) ===
const float TEMP_THRESHOLD_F           = 85.0;
const unsigned long TEMP_ALARM_DELAY_MS   = 3000; // must stay high 3s
const unsigned long TEMP_ALARM_COOLDOWN_MS= 1000; // stay below 1s to clear
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 400;

bool buzzerState = false;
unsigned long phaseStart   = 0;
unsigned long tempHighStart= 0;
unsigned long tempLowStart = 0;
bool tempHigh = false;
float latestTempF = 0.0;

void printLine(int row, const String &text) {
  lcd.setCursor(0, row);
  lcd.print(text);
}

bool shouldTempAlarm(float tempF) {
  unsigned long now = millis();

  if (tempF >= TEMP_THRESHOLD_F) {
    tempLowStart = 0; // reset cooldown
    if (!tempHigh) {
      tempHigh = true;
      tempHighStart = now;
    }
    if (now - tempHighStart >= TEMP_ALARM_DELAY_MS) {
      return true; // high long enough
    }
  } else {
    tempHigh = false;
    tempHighStart = 0;
    if (tempLowStart == 0) tempLowStart = now;
    // keep alarm on during cooldown
    if (now - tempLowStart >= TEMP_ALARM_COOLDOWN_MS) {
      return false; // cleared
    } else {
      return true;  // still cooling down
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

// --- Tasks ---
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
  printLine(1, "I:" + String(AcsValueF, 4) + " A");

  // Keep the motor logic exactly as in your first code
  if (f1 >= 82) motorState = true; else motorState = false;
  if (f2 >= 82) motorState = false;

  // Optional debug:
  // Serial.printf("T1=%.1fF T2=%.1fF latest=%.1fF motor=%s\n", f1, f2, latestTempF, motorState?"ON":"OFF");
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
  AcsValueF = currentValue; // keep your LCD variable updated

  // Optional debug:
  // Serial.printf("ADCavg=%.1f V=%.3f I=%.3f A\n", adcAvg, voltage, currentValue);
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
  else            digitalWrite(IN1_PIN, LOW);
}

void lcdPrintTask() {
  // (kept as a placeholder; temp/current tasks already update LCD)
}

// --- Setup/Loop ---
void setup() {
  Serial.begin(115200);

  // Temperature sensors
  sensors.begin();
  sensors.setWaitForConversion(false); // non-blocking

  // LCD
  lcd.begin(16, 2);
  lcd.clear();

  printLine(0, "System Init...");
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  delay(1500);
  lcd.clear();

  printLine(0, "System Ready!");
  printLine(1, "Pump: OFF");
  delay(1500);
  lcd.clear();

  // Buzzer
  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW);

  // ADC config for ESP32
  analogSetPinAttenuation(sensorPin, ADC_11db);
}

void loop() {
  if (tempCheck_timer    >= tempCheck_MS)    { tempCheck_timer    = 0; tempCheck_task();    }
  if (lcdPrint_timer     >= lcdPrint_MS)     { lcdPrint_timer     = 0; lcdPrintTask();     }
  if (currentCheck_timer >= currentCheck_MS) { currentCheck_timer = 0; currentCheck_task(); }
  if (alarmCheck_timer   >= alarmCheck_MS)   { alarmCheck_timer   = 0; alarmCheck_task();   }
  if (motorCheck_timer   >= motorCheck_MS)   { motorCheck_timer   = 0; motorCheck_task();   }
}
