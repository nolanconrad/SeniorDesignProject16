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

void tempCheck_task()
{
  Serial.print("TEMPERATURE CHECK \n");
  sensors.requestTemperatures(); // start conversion
  float c1 = 0;
  float c2 = 0;
  Serial.print("Number of sensors: ");
  Serial.println(sensors.getDeviceCount());
  int i = sensors.getDeviceCount();
  c2 = sensors.getTempCByIndex(i - 1); // read each device
  c1 = sensors.getTempCByIndex(i - 2); // read each device
  float f1 = (c1 * 9.0 / 5.0) + 32.0;
  float f2 = (c2 * 9.0 / 5.0) + 32.0;
  Serial.print("Sensor ");
  // String line = "T" + String(i+1) + " = " + String(c, 2) + " F"; //might need to fix this line
  // printLine(0, line);
  Serial.print(i - 1);
  Serial.print(": ");
  Serial.print(f1);
  Serial.println(" °F");
  Serial.print("Sensor ");
  Serial.print(i);
  Serial.print(": ");
  Serial.print(f2);
  Serial.println(" °F");
  printLine(0, "1:" + String(f1) + "F" + "2:" + String(f2) + "F");
  printLine(1, "I:" + String(AcsValueF, 2) + " A");

  // this turns on the motor if temp is above threshold (105.8°F ≈ 41°C)
  if (f1 >= 82)
  {
    motorState = true;
  }
  else
  {
    motorState = false;
  }
  if (f2 >= 82)
  {
    motorState = false;
  }
  /*if(f1 < 105.8 || f2 < 105.8){
     digitalWrite(BUZZ_SW, LOW);
   }  */
}

void lcdPrintTask()
{
  Serial.print("LCD UPDATE START\n");
  /// LCD STATUS UPDATE ///
  static uint32_t last = 0;
  static bool toggle = false;
  uint32_t now = millis();

  if (now - last >= 1000)
  { // update every 1 second
    last += 1000;
    toggle = !toggle;
  }

  /*if (toggle) {
    printLine(0, "Pump: ON        ");
    printLine(1, "System: READY   ");
  } else {
    printLine(0, "Pump: STANDBY   ");
    printLine(1, "System: IDLE    ");
  }*/

  Serial.print("LCD UPDATE STOP\n");
}

void currentCheck_task()
{
  Serial.print("CURRENT CHECK START\n");
  unsigned int x = 0;
  float AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0;
  AcsValueF = 0.0;
  for (int x = 0; x < 150; x++)
  {                                   // Get 150 samples
    AcsValue = analogRead(sensorPin); // Read current sensor values
    Samples = Samples + AcsValue;     // Add samples together
    delay(3);                         // let ADC settle before next sample 3ms
  }
  AvgAcs = Samples / 150.0; // Taking Average of Samples
  Serial.print("AcsValue: ");
  Serial.print(AcsValue, 4);
  Serial.println(" V");
  //((AvgAcs * (5.0 / 1024.0)) is converting the read voltage in 0-5 volts
  // 2.5 is offset (assumed that Arduino is working on 5v so the Vout at no current comes
  // out to be 2.5 which is offset. If your Arduino is working on different voltage then
  // you must change the offset according to the input voltage)
  // 0.066v(66mV) is rise in output voltage when 1A current flows at input
  AcsValueF = (0.312 - (AvgAcs * (5.1 / 4095.0))) / 0.044;
  Serial.print("Current: ");
  Serial.print(AcsValueF, 2); // 2 decimal places
  Serial.println(" A");
  // Serial.println(AcsValueF);//Print the read current on Serial monitor
  delay(50);
  // Take multiple samples and average to reduce ADC noise
}
// turns alarm on/off based on current

void alarmCheck_task()
{
  Serial.print("ALARM CHECK START\n");
  if (currentValue >= 10.0)
  { // threshold current for alarm
    alarmState = true;
  }
  else
  {
    alarmState = false;
  }

  if (alarmState)
  {
    Serial.print("ALARM ON \n");
    digitalWrite(BUZZ_SW, HIGH);
    delay(1000);
    digitalWrite(BUZZ_SW, LOW);
    delay(500); // debounce
  }
  else
  {
    Serial.print("ALARM OFF \n");
    // turn buzzer OFF
    digitalWrite(BUZZ_SW, LOW);
    delay(1000);
  }

  Serial.print("ALARM CHECK STOP\n");
}

//
void motorCheck_task()
{
  Serial.print("MOTOR CHECK START\n"); // logging start to console

  // control motor based on motorState boolean from tempCheck_task
  if (motorState)
  {
    digitalWrite(IN1_PIN, HIGH);
    Serial.println("Motor ON");
  }
  else
  {
    digitalWrite(IN1_PIN, LOW);
    Serial.println("Motor OFF");
  }

  Serial.print("MOTOR CHECK STOP\n"); // logging end to console
}

void loop()
{
  if (tempCheck_timer >= tempCheck_MS)
  {
    tempCheck_timer = 0;
    tempCheck_task();
  }
  if (lcdPrint_timer >= lcdPrint_MS)
  {
    lcdPrint_timer = 0;
    lcdPrintTask();
  }
  if (currentCheck_timer >= currentCheck_MS)
  {
    currentCheck_timer = 0;
    currentCheck_task();
  }
  if (alarmCheck_timer >= alarmCheck_MS)
  {
    alarmCheck_timer = 0;
    alarmCheck_task();
  }
  if (motorCheck_timer >= motorCheck_MS)
  {
    motorCheck_timer = 0;
    motorCheck_task();
  }
}
