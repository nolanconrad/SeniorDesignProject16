#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <elapsedMillis.h>

#define ONE_WIRE_BUS 4            // D4 = GPIO4 on your board
constexpr int IN1_PIN = 23;       // motor driver IN1
const int BUZZ_SW = 18;           // buzzer switch pin
float zeroOffsetV = 0.0;          // auto-calibrated offset voltage (in volts)

// --- OneWire / DS18B20 ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- LCD pins (RS, EN, D4, D5, D6, D7) ---
LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

// --- Current sensor / math (ESP32 + divider) ---
int sensorPin = 32;
// NOTE: with a 1k(top)/2k(bottom) divider, Vadc = Vsensor * (2/3)

// --- Timers ---
elapsedMillis tempCheck_timer, lcdPrint_timer, currentCheck_timer, alarmCheck_timer, motorCheck_timer, cooldown_timer;
const uint32_t tempCheck_MS    = 1000;
const uint32_t lcdPrint_MS     = 2000;
const uint32_t currentCheck_MS = 1000;
const uint32_t cooldownCheck_MS   = 500;  
const uint32_t motorCheck_MS   = 5000;

boolean motorState = false;
boolean alarmState = false;
boolean isCooldown = false;
float currentValue = 0.0;   // amps (for alarm)
float AcsValueF    = 0.0;   // shown on LCD (mirrors currentValue)

// === Temperature alarm config (WITH HYSTERESIS) ===
const float TEMP_ALARM_ON_F   = 85.0;      // trip temp
const float TEMP_ALARM_OFF_F  = 82.0;      // clear temp
const unsigned long TEMP_ALARM_ON_DELAY_MS  = 5000; // must be hot this long
const unsigned long TEMP_ALARM_OFF_DELAY_MS = 3000; // must be cool this long

// (old constants kept if you use elsewhere)
const float TEMP_THRESHOLD_F            = 85.0;
const unsigned long TEMP_ALARM_DELAY_MS = 3000;
const unsigned long TEMP_ALARM_COOLDOWN_MS = 1000;
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 400;

bool buzzerState = false;
unsigned long phaseStart   = 0;
unsigned long tempHighStart= 0;
unsigned long tempLowStart = 0;
bool tempHigh = false;
float latestTempF = 0.0;

// Hysteresis state/timers
bool tempAlarm = false;
unsigned long hotStartMs = 0;
unsigned long coolStartMs = 0;

//printing functon for lcd
void printLine(int row, const String &text) {
  lcd.setCursor(0, row);
  lcd.print(text);
}
void lcdPrintTask() {
  // optional; temp/current tasks already update the LCD
}

void isAlarmBuzzing(bool active, unsigned long durationMs, unsigned long amountofChirps) {
  static unsigned long chirpCount = 0;

  if (!active) {
    buzzerState = false;
    digitalWrite(BUZZ_SW, LOW);
    chirpCount = 0; // Reset chirp count when inactive
    return;
  }

  unsigned long now = millis();
  unsigned long phaseLen = durationMs;

  if (chirpCount < amountofChirps) {
    if (now - phaseStart >= phaseLen) {
      buzzerState = !buzzerState;
      phaseStart = now;

      if (!buzzerState) {
        chirpCount++; // Increment chirp count after each OFF phase
      }

      digitalWrite(BUZZ_SW, buzzerState ? HIGH : LOW);
    }
  } else {
    buzzerState = false;
    digitalWrite(BUZZ_SW, LOW); // Ensure buzzer is off after chirps are done
  }
}

// ================== TEMP TASK (fixed with hysteresis) ==================
void tempCheck_task()
{
  Serial.println("TEMPERATURE CHECK");
  sensors.requestTemperatures(); // non-blocking mode set in setup()
  int count = sensors.getDeviceCount();

  float c1 = 0, c2 = 0;
  if (count >= 1) c2 = sensors.getTempCByIndex(count - 1);
  if (count >= 2) c1 = sensors.getTempCByIndex(count - 2);

  float f1 = (c1 * 9.0f / 5.0f) + 32.0f;
  float f2 = (c2 * 9.0f / 5.0f) + 32.0f;
  latestTempF = max(f1, f2);

  printLine(0, "1:" + String(f1) + "F 2:" + String(f2) + "F");
  printLine(1, "I:" + String(AcsValueF, 4) + " A");

  unsigned long now = millis();
  bool prevAlarm = tempAlarm;

  // ON path (>=85F for 5s)
  if (latestTempF >= TEMP_ALARM_ON_F) {
    coolStartMs = 0;
    if (hotStartMs == 0) hotStartMs = now;
    if (now - hotStartMs >= TEMP_ALARM_ON_DELAY_MS) tempAlarm = true;
  }
  // OFF path (<=82F for 3s)
  else if (latestTempF <= TEMP_ALARM_OFF_F) {
    hotStartMs = 0;
    if (coolStartMs == 0) coolStartMs = now;
    if (now - coolStartMs >= TEMP_ALARM_OFF_DELAY_MS) tempAlarm = false;
  }
  // Deadband: keep state, reset timers
  else {
    hotStartMs = 0;
    coolStartMs = 0;
  }

  // Edge: alarm just turned on
  if (tempAlarm && !prevAlarm) {
    isCooldown = true;                 // shut motor via cooldown
    printLine(0, "** OVERTEMP! **");
  }
  // Edge: alarm just turned off
  if (!tempAlarm && prevAlarm) {
    isAlarmBuzzing(false, 0, 0);       // stop buzzer immediately
    lcd.setCursor(0,0); lcd.print("                "); // clear banner
  }

  // Drive buzzer non-blocking each call
  isAlarmBuzzing(tempAlarm, 500, 5);

  // Your motor hysteresis (kept as-is)
  if (f1 >= 82 && f1 < 85) motorState = true; else motorState = false;
}

// ---------------------- FIXED RMS CURRENT TASK ----------------------
void currentCheck_task() {
  Serial.println("CURRENT CHECK START");

  // ACS712-30A sensitivity is 66 mV/A at the sensor output (no divider)
  const float SENS_V_PER_A = 0.066f;
  // Your divider: top=1k (to sensor), bottom=2k (to GND) -> Vadc = Vsensor * (2/3)
  const float DIV_RATIO = 2.0f / 3.0f;

  const int   samples = 400;         // enough to cover several AC cycles
  const int   sampleDelay_us = 200;  // ~5 kS/s
  double sumSq = 0.0;
  double sumMv = 0.0;

  for (int i = 0; i < samples; ++i) {
    int mv = analogReadMilliVolts(sensorPin);         // ADC reading in mV
    float deltaMv_adc    = mv - (zeroOffsetV * 1000.0f); // remove offset (ADC domain)
    float deltaMv_sensor = deltaMv_adc / DIV_RATIO;      // undo divider
    float instCurrent    = (deltaMv_sensor / 1000.0f) / SENS_V_PER_A; // amps

    sumSq += instCurrent * instCurrent;
    sumMv += mv;

    delayMicroseconds(sampleDelay_us);
  }

  float Irms = sqrt(sumSq / samples);
  float avgMv = sumMv / samples;

  // gentle re-centering when basically idle (helps temperature drift)
  if (Irms < 0.05f) {
    zeroOffsetV = 0.995f * zeroOffsetV + 0.005f * (avgMv / 1000.0f);
  }

  AcsValueF = Irms;
  currentValue = Irms;

  Serial.printf("Zero offset: %.3f V | Irms: %.4f A\n", zeroOffsetV, Irms);
  printLine(1, "I:" + String(Irms, 4) + " A RMS");

  //alarm sounds if the current is over X amps (RMS)
  if (Irms > 1.0f) {
    Serial.println("** ALERT: Overcurrent condition! **");
    isAlarmBuzzing(true, 100, 5);
    printLine(0, "** OVERCURRENT! **");
    isCooldown = true; //turns motor off if it is on
  }

  delay(50);
}
// -------------------------------------------------------------------

void cooldown_task() 
{
  int restoreMotor = 1;
  if (isCooldown) {
    printLine(0, "Cooldown for 5secs.");
    if(motorState) {
      motorState = false;
      restoreMotor = 0;
      Serial.println("Motor OFF due to overcurrent");
    } else if (motorState == false) {
      Serial.println("Motor already OFF");
    } else {
      Serial.println("Motor state unknown");
    }
    delay(5000); // Cooldown period of 5 seconds
    if(restoreMotor == 0) motorState = true; //restore motor state
    printLine(1, "Resuming ops...");
    isCooldown = false;
  }
}

void motorCheck_task()
{
  if (motorState) {
    digitalWrite(IN1_PIN, HIGH);
    Serial.println("Motor ON");
  } else {
    digitalWrite(IN1_PIN, LOW);
    Serial.println("Motor OFF");
  }
}

// --- Setup --- //
void setup() {
  Serial.begin(115200);

  // Temperature sensors startup
  sensors.begin();
  sensors.setWaitForConversion(false);

  // LCD startup
  lcd.begin(16, 2);
  lcd.clear();
  printLine(0, "System Init...");
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  delay(1500);
  lcd.clear();

  // Buzzer setup
  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW);

  // ADC config for ESP32
  analogReadResolution(12);
  analogSetPinAttenuation(sensorPin, ADC_11db);

  // === AUTO-CALIBRATE CURRENT SENSOR OFFSET ===
  Serial.println("Calibrating ACS712 zero offset... make sure motor is OFF.");
  printLine(0, "Calibrating ACS712");
  printLine(1, "Ensure motor OFF");
  const int zeroSamples = 300;
  long zeroSum = 0;
  for (int i = 0; i < zeroSamples; i++) {
    zeroSum += analogRead(sensorPin);
    delay(2);
  }
  
  float zeroADC = (float)zeroSum / zeroSamples;
  zeroOffsetV = (zeroADC / 4095.0) * 3.3;    // convert to volts
  Serial.print("Measured zero offset = ");
  Serial.print(zeroOffsetV, 3);
  Serial.println(" V");
  delay(500);

  //system is ready
  printLine(0, "System Ready!");
  delay(1500);
  lcd.clear();
}

void loop()
{
  if (tempCheck_timer >= tempCheck_MS)   { tempCheck_timer = 0;   tempCheck_task(); }
  if (lcdPrint_timer >= lcdPrint_MS)     { lcdPrint_timer = 0;    lcdPrintTask(); }
  if (currentCheck_timer >= currentCheck_MS){ currentCheck_timer = 0; currentCheck_task(); }
  if (motorCheck_timer >= motorCheck_MS) { motorCheck_timer = 0;  motorCheck_task(); }
  if (cooldown_timer >= cooldownCheck_MS) { cooldown_timer = 0;  cooldown_task(); }
  /* 
    if (alarmCheck_timer >= alarmCheck_MS) { alarmCheck_timer = 0; isAlarmBuzzing(alarmState, BEEP_ON_MS, 3); } 
    replaced by isAlarmBuzzing calls in tempCheck_task()
  */
}
