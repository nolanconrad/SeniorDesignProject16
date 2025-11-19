#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <elapsedMillis.h>

#define ONE_WIRE_BUS 4
constexpr int IN1_PIN = 23;
const int BUZZ_SW = 18;
float zeroOffsetV = 0.0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

int sensorPin = 32;

// Timers
elapsedMillis tempCheck_timer, lcdPrint_timer, currentCheck_timer, motorCheck_timer;
elapsedMillis cooldown_timer, buzzer_timer;
const uint32_t tempCheck_MS = 1000;
const uint32_t lcdPrint_MS = 500;  // Faster for better display updates
const uint32_t currentCheck_MS = 1000;
const uint32_t motorCheck_MS = 500;  // More responsive

// State variables
boolean motorDesiredState = false;  // What motor SHOULD be (from temp logic)
boolean motorAllowed = true;        // Whether motor CAN run (safety override)
float currentValue = 0.0;
float AcsValueF = 0.0;
float latestTempF = 0.0;

// Temperature alarm config
const float TEMP_ALARM_ON_F = 85.0;
const float TEMP_ALARM_OFF_F = 82.0;
const float TEMP_MOTOR_ON_LOW = 82.0;   // Motor enable range
const float TEMP_MOTOR_ON_HIGH = 85.0;
const unsigned long TEMP_ALARM_ON_DELAY_MS = 5000;
const unsigned long TEMP_ALARM_OFF_DELAY_MS = 3000;

// Cooldown state
bool isCooldown = false;
unsigned long cooldownStartMs = 0;
const unsigned long COOLDOWN_DURATION_MS = 5000;

// Buzzer state
bool buzzerActive = false;
bool buzzerState = false;
unsigned long buzzerPhaseStart = 0;
unsigned long buzzerChirpCount = 0;
unsigned long buzzerMaxChirps = 0;
unsigned long buzzerDuration = 0;
unsigned long buzzerCycleStart = 0;
const unsigned long BUZZER_CYCLE_MS = 3000;  // Repeat chirps every 3 seconds

// Alarm state
bool tempAlarm = false;
bool currentAlarm = false;
unsigned long hotStartMs = 0;
unsigned long coolStartMs = 0;

// LCD update control
String lcd_row0 = "";
String lcd_row1 = "";
bool lcd_needs_update = false;

// ============ LCD Helper Functions ============
void updateLCD(int row, const String &text) {
  if (row == 0 && lcd_row0 != text) {
    lcd_row0 = text;
    lcd_needs_update = true;
  } else if (row == 1 && lcd_row1 != text) {
    lcd_row1 = text;
    lcd_needs_update = true;
  }
}

void refreshLCD() {
  if (lcd_needs_update) {
    lcd.setCursor(0, 0);
    lcd.print(lcd_row0);
    // Pad to clear old characters
    for (int i = lcd_row0.length(); i < 16; i++) lcd.print(" ");
    
    lcd.setCursor(0, 1);
    lcd.print(lcd_row1);
    for (int i = lcd_row1.length(); i < 16; i++) lcd.print(" ");
    
    lcd_needs_update = false;
  }
}

// ============ Non-Blocking Buzzer ============
void startBuzzer(unsigned long durationMs, unsigned long chirps) {
  buzzerActive = true;
  buzzerChirpCount = 0;
  buzzerMaxChirps = chirps;
  buzzerDuration = durationMs;
  buzzerPhaseStart = millis();
  buzzerCycleStart = millis();
  buzzerState = false;
  digitalWrite(BUZZ_SW, LOW);
}

void stopBuzzer() {
  buzzerActive = false;
  buzzerState = false;
  buzzerChirpCount = 0;
  digitalWrite(BUZZ_SW, LOW);
}

void updateBuzzer() {
  if (!buzzerActive) return;

  unsigned long now = millis();
  
  // Restart chirp cycle every BUZZER_CYCLE_MS
  if (now - buzzerCycleStart >= BUZZER_CYCLE_MS) {
    buzzerChirpCount = 0;
    buzzerCycleStart = now;
    buzzerPhaseStart = now;
    buzzerState = false;
  }

  // Generate chirps
  if (buzzerChirpCount < buzzerMaxChirps) {
    if (now - buzzerPhaseStart >= buzzerDuration) {
      buzzerState = !buzzerState;
      buzzerPhaseStart = now;

      if (!buzzerState) {
        buzzerChirpCount++;
      }

      digitalWrite(BUZZ_SW, buzzerState ? HIGH : LOW);
    }
  } else {
    // Finished chirps for this cycle, wait for next cycle
    digitalWrite(BUZZ_SW, LOW);
  }
}

// ============ Temperature Task ============
void tempCheck_task() {
  sensors.requestTemperatures();
  int count = sensors.getDeviceCount();

  float c1 = 0, c2 = 0;
  if (count >= 1) c2 = sensors.getTempCByIndex(count - 1);
  if (count >= 2) c1 = sensors.getTempCByIndex(count - 2);

  float f1 = (c1 * 9.0f / 5.0f) + 32.0f;
  float f2 = (c2 * 9.0f / 5.0f) + 32.0f;
  latestTempF = max(f1, f2);

  unsigned long now = millis();
  bool prevAlarm = tempAlarm;

  // Temperature alarm with hysteresis
  if (latestTempF >= TEMP_ALARM_ON_F) {
    coolStartMs = 0;
    if (hotStartMs == 0) hotStartMs = now;
    if (now - hotStartMs >= TEMP_ALARM_ON_DELAY_MS) {
      tempAlarm = true;
    }
  } else if (latestTempF <= TEMP_ALARM_OFF_F) {
    hotStartMs = 0;
    if (coolStartMs == 0) coolStartMs = now;
    if (now - coolStartMs >= TEMP_ALARM_OFF_DELAY_MS) {
      tempAlarm = false;
    }
  } else {
    hotStartMs = 0;
    coolStartMs = 0;
  }

  // Handle alarm state changes
  if (tempAlarm && !prevAlarm) {
    // Alarm just triggered
    motorAllowed = false;
    isCooldown = true;
    cooldownStartMs = now;
    startBuzzer(500, 5);
    updateLCD(0, "** OVERTEMP! **");
    Serial.println("TEMP ALARM ON");
  } else if (!tempAlarm && prevAlarm) {
    // Alarm just cleared
    stopBuzzer();
    Serial.println("TEMP ALARM OFF");
  }

  // Motor desired state based on temperature (only when no alarms)
  if (!tempAlarm && !currentAlarm) {
    if (f1 >= TEMP_MOTOR_ON_LOW && f1 < TEMP_MOTOR_ON_HIGH) {
      motorDesiredState = true;
    } else {
      motorDesiredState = false;
    }
  }

  // Update LCD if no alarm
  if (!tempAlarm && !currentAlarm) {
    updateLCD(0, "1:" + String(f1, 1) + "F 2:" + String(f2, 1) + "F");
  }
}

// ============ Current Check Task ============
void currentCheck_task() {
  const float SENS_V_PER_A = 0.066f;
  const float DIV_RATIO = 2.0f / 3.0f;
  const int samples = 400;
  const int sampleDelay_us = 200;
  
  double sumSq = 0.0;
  double sumMv = 0.0;

  for (int i = 0; i < samples; ++i) {
    int mv = analogReadMilliVolts(sensorPin);
    float deltaMv_adc = mv - (zeroOffsetV * 1000.0f);
    float deltaMv_sensor = deltaMv_adc / DIV_RATIO;
    float instCurrent = (deltaMv_sensor / 1000.0f) / SENS_V_PER_A;

    sumSq += instCurrent * instCurrent;
    sumMv += mv;

    delayMicroseconds(sampleDelay_us);
  }

  float Irms = sqrt(sumSq / samples);
  float avgMv = sumMv / samples;

  // Re-center when idle
  if (Irms < 0.05f) {
    zeroOffsetV = 0.995f * zeroOffsetV + 0.005f * (avgMv / 1000.0f);
  }

  AcsValueF = Irms;
  currentValue = Irms;

  // Update LCD row 1
  updateLCD(1, "I:" + String(Irms, 3) + " A");

  // Check for overcurrent
  bool prevCurrentAlarm = currentAlarm;
  if (Irms > 1.0f) {
    currentAlarm = true;
    if (!prevCurrentAlarm) {
      // Just triggered
      motorAllowed = false;
      isCooldown = true;
      cooldownStartMs = millis();
      startBuzzer(100, 5);
      updateLCD(0, "** OVERCURRENT! **");
      Serial.println("CURRENT ALARM ON");
    }
  } else if (Irms < 0.8f) {  // Hysteresis
    if (prevCurrentAlarm) {
      currentAlarm = false;
      stopBuzzer();
      Serial.println("CURRENT ALARM OFF");
    }
  }
}

// ============ Non-Blocking Cooldown ============
void cooldown_task() {
  if (isCooldown) {
    unsigned long elapsed = millis() - cooldownStartMs;
    
    if (elapsed >= COOLDOWN_DURATION_MS) {
      // Cooldown complete
      isCooldown = false;
      motorAllowed = true;
      Serial.println("Cooldown complete");
    }
  }
}

// ============ Motor Control ============
void motorCheck_task() {
  bool shouldBeOn = motorDesiredState && motorAllowed && !isCooldown;
  
  static bool lastMotorState = false;
  
  if (shouldBeOn != lastMotorState) {
    digitalWrite(IN1_PIN, shouldBeOn ? HIGH : LOW);
    Serial.println(shouldBeOn ? "Motor ON" : "Motor OFF");
    lastMotorState = shouldBeOn;
  }
}

// ============ Setup ============
void setup() {
  Serial.begin(115200);

  sensors.begin();
  sensors.setWaitForConversion(false);

  lcd.begin(16, 2);
  lcd.clear();
  updateLCD(0, "System Init...");
  refreshLCD();
  
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  
  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(sensorPin, ADC_11db);

  // Calibrate current sensor
  Serial.println("Calibrating ACS712...");
  updateLCD(0, "Calibrating...");
  updateLCD(1, "Motor must be OFF");
  refreshLCD();
  
  const int zeroSamples = 300;
  long zeroSum = 0;
  for (int i = 0; i < zeroSamples; i++) {
    zeroSum += analogRead(sensorPin);
    delay(2);
  }
  
  float zeroADC = (float)zeroSum / zeroSamples;
  zeroOffsetV = (zeroADC / 4095.0) * 3.3;
  Serial.printf("Zero offset = %.3f V\n", zeroOffsetV);

  updateLCD(0, "System Ready!");
  updateLCD(1, "");
  refreshLCD();
  delay(1500);
  lcd.clear();
}

// ============ Main Loop ============
void loop() {
  if (tempCheck_timer >= tempCheck_MS) {
    tempCheck_timer = 0;
    tempCheck_task();
  }
  
  if (currentCheck_timer >= currentCheck_MS) {
    currentCheck_timer = 0;
    currentCheck_task();
  }
  
  if (motorCheck_timer >= motorCheck_MS) {
    motorCheck_timer = 0;
    motorCheck_task();
  }
  
  cooldown_task();  // Run every loop (non-blocking)
  updateBuzzer();   // Run every loop (non-blocking)
  
  if (lcdPrint_timer >= lcdPrint_MS) {
    lcdPrint_timer = 0;
    refreshLCD();
  }
}