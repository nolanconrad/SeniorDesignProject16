#include <Arduino.h>
#include <math.h>

// ===================== USER SETTINGS =====================
// --- Motor driver (DRV8871) ---
const int MOTOR_IN1_PIN = 25;     // IN1 pin from ESP32 to DRV8871
// IN2 is tied to GND externally (as you mentioned)

// --- Current sensor (ACS712) ---
const int ACS_PIN = 35;           // ADC1 channel (GPIO32)
const float VCC = 3.30f;          // ESP32 3V3 rail used by ACS712
// Sensitivity (choose your ACS712 variant): 185=5A, 100=20A, 66=30A (mV/A)
const float ACS_MV_PER_A = 100.0f; // <-- set to 66, 100, or 185 as appropriate
// We auto-calibrate zero offset at startup, so no need to set mid-bias manually.

// --- Thermistors ---
// Put your NTCs in a divider: 3.3V -- R_FIXED --(ADC pin)-- NTC -- GND
// Edit pin list to match how many thermistors you have (1 or 2 typical).
const int THERM_PINS[] = {34, 35};     // ADC1-only pins; add/remove as needed
const int NUM_THERMS = sizeof(THERM_PINS) / sizeof(THERM_PINS[0]);

// Divider values/NTC model (typical 10k NTC, B=3950)
const float R_FIXED = 10000.0f;   // The fixed resistor (ohms)
const float R0 = 10000.0f;        // NTC resistance at T0 (ohms)
const float T0_K = 298.15f;       // 25°C in Kelvin
const float BETA = 3950.0f;       // Beta value (check your datasheet)

// --- Timing ---
const uint32_t REPORT_MS = 1000;      // print to Serial every 1 s
const uint32_t MOTOR_ON_MS  = 5000;   // pump ON duration (edit as needed)
const uint32_t MOTOR_OFF_MS = 5000;   // pump OFF duration (edit as needed)

// --- ADC sampling ---
const int ACS_SAMPLES = 32;       // averaging for ACS reading
const int THERM_SAMPLES = 16;     // averaging for thermistor reading
// =========================================================

enum MotorState { MOTOR_OFF = 0, MOTOR_ON = 1 };

uint32_t t_last_report = 0;
uint32_t t_state_start = 0;
MotorState motor_state = MOTOR_OFF;

float acs_zero_volts = VCC / 2.0f;  // will be measured at startup

// ===== Helpers =====
float readADCvolts(int pin, int samples) {
  // Take 'samples' readings and average; returns voltage
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) {
    acc += analogRead(pin);
  }
  float avg = (float)acc / (float)samples;
  // ESP32 default ADC is 12-bit (0..4095). We'll ensure this in setup().
  return (avg / 4095.0f) * VCC;
}

float thermistorCelsiusFromDivider(float v_meas) {
  // Divider: 3.3V -- R_FIXED -- node(ADC) -- NTC -- GND
  // Rt = R_FIXED * (V / (Vcc - V))
  if (v_meas <= 0.0f) v_meas = 1e-6f;          // avoid div-by-zero
  if (v_meas >= VCC)  v_meas = VCC - 1e-6f;

  float rt = R_FIXED * (v_meas / (VCC - v_meas));

  // Beta equation:
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  float invT = (1.0f / T0_K) + (1.0f / BETA) * logf(rt / R0);
  float T_K = 1.0f / invT;
  float T_C = T_K - 273.15f;
  return T_C;
}

float readACSamps() {
  // Read ACS voltage, subtract zero, convert to amps
  float v = readADCvolts(ACS_PIN, ACS_SAMPLES);
  float dv = v - acs_zero_volts;        // could be negative
  float amps = (dv * 1000.0f) / ACS_MV_PER_A;  // mV/A -> A
  return amps;
}

void setMotor(MotorState s) {
  motor_state = s;
  digitalWrite(MOTOR_IN1_PIN, (s == MOTOR_ON) ? HIGH : LOW);
  t_state_start = millis();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC config (ESP32)
  analogReadResolution(12);       // 0..4095
  // 11dB attenuation gives full-scale ~3.3V on ADC1 channels
  analogSetAttenuation(ADC_11db); // apply to all channels used

  // Pins
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, LOW); // start OFF

  // Thermistor pins are input by default; for completeness:
  for (int i = 0; i < NUM_THERMS; ++i) {
    pinMode(THERM_PINS[i], INPUT);
  }
  pinMode(ACS_PIN, INPUT);

  // Auto-calibrate ACS zero (assumes no motor load for ~500 ms)
  const uint32_t CAL_MS = 600;
  uint32_t t0 = millis();
  float sumV = 0.0f;
  int n = 0;
  while (millis() - t0 < CAL_MS) {
    sumV += readADCvolts(ACS_PIN, 1);
    n++;
    delay(2);
  }
  acs_zero_volts = (n > 0) ? (sumV / n) : (VCC / 2.0f);

  Serial.println(F("=== Pump cycle + ACS712 + Thermistors ==="));
  Serial.print(F("ACS zero (V): ")); Serial.println(acs_zero_volts, 4);
  Serial.println(F("time_s, motor, current_A, T1_C, T2_C"));
  // Start with motor OFF
  setMotor(MOTOR_OFF);
  t_last_report = millis();
}

void loop() {
  uint32_t now = millis();

  // ----- Motor state machine (non-blocking) -----
  if (motor_state == MOTOR_OFF) {
    if (now - t_state_start >= MOTOR_OFF_MS) {
      setMotor(MOTOR_ON);
    }
  } else { // MOTOR_ON
    if (now - t_state_start >= MOTOR_ON_MS) {
      setMotor(MOTOR_OFF);
    }
  }

  // ----- Reporting every REPORT_MS -----
  if (now - t_last_report >= REPORT_MS) {
    t_last_report = now;

    // Read current
    float I = readACSamps();

    // Read thermistors
    float tempsC[NUM_THERMS];
    for (int i = 0; i < NUM_THERMS; ++i) {
      float v_avg = readADCvolts(THERM_PINS[i], THERM_SAMPLES);
      tempsC[i] = thermistorCelsiusFromDivider(v_avg);
    }

    // Print CSV line
    Serial.print(now / 1000.0f, 1); Serial.print(F(", "));
    Serial.print((motor_state == MOTOR_ON) ? F("ON") : F("OFF")); Serial.print(F(", "));
    Serial.print(I, 3);

    for (int i = 0; i < NUM_THERMS; ++i) {
      Serial.print(F(", "));
      Serial.print(tempsC[i], 2);
    }
    Serial.println();
  }
}
