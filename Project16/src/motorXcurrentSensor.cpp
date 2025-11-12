// ESP32 + DRV8871 + ACS712 (ultra-basic demo)
// IN1 -> GPIO23 (change if needed)
// ACS712 OUT -> GPIO32 (ADC), VCC->5V, GND->GND

#include <Arduino.h>

constexpr int IN1_PIN     = 23;     // <-- change to your IN1 GPIO if needed
constexpr int CURRENT_PIN = 32;     // ACS712 OUT -> GPIO32 (ADC1)
// Voltage divider input (OUT -> Rtop 100k -> DIV_PIN -> Rbot 68k -> GND)
constexpr int DIV_PIN     = 18;     // D18 / GPIO18 (voltage divider)
constexpr float R_TOP     = 100000.0f; // ohms
constexpr float R_BOT     = 68000.0f;  // ohms
constexpr float DIV_MULT  = (R_TOP + R_BOT) / R_BOT; // multiplier to recover Vin from Vout (~2.470588)
constexpr float VREF      = 3.3;    // ESP32 ADC reference
constexpr float ADC_MAX   = 4095.0; // 12-bit ADC
constexpr float SENSITIVITY = 0.066; // V/A (0.185 for 5A, 0.100 for 20A, 0.066 for 30A)

float v_zero = 2.5f; // will be calibrated at startup

float readVoltage() {
  int adc = analogRead(CURRENT_PIN);
  return (adc / ADC_MAX) * VREF;
}

// Read the voltage at the ADC pin after the divider (Vout)
float readDividerVout() {
  int adc = analogRead(DIV_PIN);
  return (adc / ADC_MAX) * VREF;
}

// Recover the original Vin before the divider
float readDividerVin() {
  float vout = readDividerVout();
  return vout * DIV_MULT;
}

float readCurrent() {
  float v = readVoltage();
  return (v - v_zero) / SENSITIVITY;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);  // start OFF

  // Optional: ensure full-scale up to ~3.3V on this pin
  // analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

  // Quick zero calibration (no load ideally)
  delay(200);
  const int N = 100;
  float sum = 0;
  for (int i = 0; i < N; ++i) {
    sum += readVoltage();
    delay(2);
  }
  v_zero = sum / N;

  Serial.println("Ready. Toggling motor and printing current...");
  Serial.print("Zero voltage = ");
  Serial.print(v_zero, 3);
  Serial.println(" V");
}

void loop() {
  // ON for 3 seconds, sample every 500 ms
  digitalWrite(IN1_PIN, HIGH);
  Serial.println("Motor ON");
  for (int i = 0; i < 6; ++i) {
    int adc = analogRead(CURRENT_PIN);
    float v  = (adc / ADC_MAX) * VREF;
    float I  = (v - v_zero) / SENSITIVITY;
    Serial.print("ADC=");
    Serial.print(adc);
    Serial.print("  V=");
    Serial.print(v, 3);
    Serial.print(" V  I=");
    Serial.print(I, 3);
    Serial.println(" A");
    delay(500);
  }

  // OFF for 3 seconds, sample every 500 ms
  digitalWrite(IN1_PIN, LOW);
  Serial.println("Motor OFF");
  for (int i = 0; i < 6; ++i) {
    int adc = analogRead(CURRENT_PIN);
    float v  = (adc / ADC_MAX) * VREF;
    float I  = (v - v_zero) / SENSITIVITY;
    Serial.print("ADC=");
    Serial.print(adc);
    Serial.print("  V=");
    Serial.print(v, 3);
    Serial.print(" V  I=");
    Serial.print(I, 3);
    Serial.println(" A");
    delay(500);
  }
}
