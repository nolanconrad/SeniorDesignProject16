#include <Arduino.h>
#include <LiquidCrystal.h>

// ------------------------- User Settings -------------------------
// LCD pins (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

// Pins
constexpr int PIN_ACS712   = 32;   // ACS712 analog output (ADC1)
constexpr int PIN_TMP36    = 34;   // TMP36 analog output (ADC1)

// ACS712 sensitivity (V/A)
// 5A: 0.185, 20A: 0.100, 30A: 0.066
constexpr float ACS_SENS   = 0.066f;   // default assumes 30A module

// Temperature sensor type
enum TempSensor { TMP36, LM35 };
constexpr TempSensor TEMP_SENSOR = TMP36;  // change to LM35 if needed

// ADC characteristics
constexpr float VREF       = 3.3f;    // ESP32 ADC (approx)
constexpr float ADC_MAX    = 4095.0f; // 12-bit

// Display update interval
constexpr uint32_t UI_MS   = 500;     // ms

// Smoothing
constexpr int SAMPLES_I    = 32;      // avg samples current
constexpr int SAMPLES_T    = 16;      // avg samples temperature

// ------------------------- Internals ----------------------------
float v_zero = 2.5f;  // ACS712 zero-current voltage (auto-cal)

static inline float adcToVolts(int adc) {
  return (adc / ADC_MAX) * VREF;
}

static int readAdcAvg(int pin, int n) {
  long sum = 0;
  for (int i = 0; i < n; ++i) {
    sum += analogRead(pin);
  }
  return (int)(sum / n);
}

float readCurrentA() {
  // averaged voltage at ACS712 pin
  int adc = readAdcAvg(PIN_ACS712, SAMPLES_I);
  float v  = adcToVolts(adc);
  float I  = (v - v_zero) / ACS_SENS;
  // kill tiny negatives due to noise/drift
  if (fabs(I) < 0.02f) I = 0.0f;
  return I;
}

float readTempC() {
  int adc = readAdcAvg(PIN_TMP36, SAMPLES_T);
  float v = adcToVolts(adc);
  if (TEMP_SENSOR == TMP36) {
    // TMP36: 10 mV/°C with 500 mV offset -> T = (V - 0.5)*100
    return (v - 0.5f) * 100.0f;
  } else { // LM35
    // LM35: 10 mV/°C, 0V at 0°C -> T = V * 100
    return v * 100.0f;
  }
}

void printFixedLine(int row, const String &s) {
  lcd.setCursor(0, row);
  String out = s;
  if (out.length() < 16) out += String(' ', 16 - out.length());
  else if (out.length() > 16) out.remove(16);
  lcd.print(out);
}

void setup() {
  Serial.begin(115200);

  // LCD init
  lcd.begin(16, 2);
  lcd.clear();
  printFixedLine(0, "ESP32 Monitor");
  printFixedLine(1, "Calibrating...");

  // Prefer ADC1 pins (32-39). Set attenuation for ~0..3.3V range
  analogSetPinAttenuation(PIN_ACS712, ADC_11db);
  analogSetPinAttenuation(PIN_TMP36,  ADC_11db);

  delay(250);
  // Auto-zero ACS712 (no load!)
  const int N = 200;
  double sumV = 0;
  for (int i = 0; i < N; ++i) {
    int adc = analogRead(PIN_ACS712);
    sumV += adcToVolts(adc);
    delay(2);
  }
  v_zero = sumV / N;

  Serial.print("ACS712 zero V = ");
  Serial.println(v_zero, 4);

  lcd.clear();
}

void loop() {
  static uint32_t t0 = 0;
  uint32_t now = millis();
  if (now - t0 >= UI_MS) {
    t0 = now;

    float I = readCurrentA();
    float T = readTempC();

    // Serial debug
    Serial.print("I="); Serial.print(I, 3); Serial.print(" A");
    Serial.print("  T="); Serial.print(T, 1); Serial.println(" C");

    // LCD: line1 Current, line2 Temperature
    // Example: "I= 1.23A         "
    char line1[17];
    snprintf(line1, sizeof(line1), "I=%6.2fA", I);

    char line2[17];
    snprintf(line2, sizeof(line2), "T=%6.1fC", T);

    printFixedLine(0, String(line1));
    printFixedLine(1, String(line2));
  }
}
