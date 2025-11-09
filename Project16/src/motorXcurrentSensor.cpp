#include <Arduino.h>
// #include <math.h> // not needed

#define IN1 18           // DRV8871 IN1 (PWM/ON-OFF)
#define ACS_PIN 32       // ACS712 OUT -> GPIO32 (ADC1_CH4)

// ------ Configure your sensor here ------
const float SENS_MV_PER_A = 100.0f;  // 5A:185, 20A:100, 30A:66
const float DIV_RATIO     = 0.50f;  // 100k:68k divider -> 0.405; use 1.0 if no divider
// ---------------------------------------

const int   ADC_BITS   = 12;
const int   ADC_MAX    = (1 << ADC_BITS) - 1;
const float ADC_VREF   = 3.30f;     // ESP32 ADC scale (approx)

float vZero_pin = 0.0f; // measured zero at the ESP32 pin (after divider)

static float readAdcVolts() {
  // small oversample for smoother numbers
  uint32_t acc = 0;
  const int N = 16;
  for (int i = 0; i < N; ++i) acc += analogRead(ACS_PIN);
  float avg = (float)acc / N;
  return (avg / ADC_MAX) * ADC_VREF;  // volts at ESP32 pin (after divider)
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(IN1, OUTPUT);
  digitalWrite(IN1, LOW); // ensure off

  // ADC setup
  analogReadResolution(ADC_BITS);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(ACS_PIN, ADC_11db);

  // --- Quick zero calibration (motor OFF) ---
  delay(200);
  double acc = 0;
  const int CAL_N = 200;
  for (int i = 0; i < CAL_N; ++i) { acc += readAdcVolts(); delay(2); }
  vZero_pin = acc / CAL_N;

  float vZero_sensor = vZero_pin / DIV_RATIO; // estimated actual ACS OUT
  Serial.printf("ACS zero: ADCpin=%.3f V  (ACSout≈%.3f V)\n", vZero_pin, vZero_sensor);
  Serial.println("Turning motor ON for 10 seconds...");
  
  // -------- Motor ON (your original behavior) --------
  digitalWrite(IN1, HIGH);

  unsigned long t0 = millis();
  while (millis() - t0 < 10000UL) {
    float v_pin    = readAdcVolts();              // at ESP32 pin
    float v_sensor = v_pin / DIV_RATIO;           // ACS OUT estimate
    float delta_V  = (v_pin - vZero_pin) / DIV_RATIO; // deviation at sensor OUT
    float amps     = delta_V / (SENS_MV_PER_A / 1000.0f);

    Serial.printf("ADCpin=%.3f V  ACSout=%.3f V  I=%.3f A\n",
                  v_pin, v_sensor, amps);

    delay(150);
  }

  // -------- Motor OFF (your original behavior) --------
  digitalWrite(IN1, LOW);
  Serial.println("Motor OFF.");
}

void loop() { /* nothing */ }
