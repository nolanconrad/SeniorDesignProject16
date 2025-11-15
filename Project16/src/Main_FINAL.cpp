#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <elapsedMillis.h>

#define ONE_WIRE_BUS 4  // D4 = GPIO4 on your board

constexpr int IN1_PIN = 23; //for motor driver

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


// LCD pins (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

//current sensor setup
int sensorPin = 32;     // ACS712 output connected to GPIO 32
const float sensitivity = 0.066;  // 66 mV/A for ACS712-30A
const float adcMax = 4095.0;      // 12-bit ADC
const float vRef = 3.3;           // ESP32 ADC reference voltage
const float zeroOffset = 2.5;     // No-current output voltage (V)

//alarm buzzer setup
const int BUZZ_SW = 25; // the transistor/MOSFET control pin


// Print text to a specific row (no padding/trimming)
void printLine(int row, const String &text) {
  lcd.setCursor(0, row);
  lcd.print(text);
}

//timing 
elapsedMillis tempCheck_timer, lcdPrint_timer, currentCheck_timer, alarmCheck_timer, motorCheck_timer;
  // each starts at 0 ms since boot
const uint32_t tempCheck_MS = 1000;   // 1 s
const uint32_t lcdPrint_MS = 2000;   // 2 s
const uint32_t currentCheck_MS = 3000;   // 3 s
const uint32_t alarmCheck_MS = 4000;   // 4 s
const uint32_t motorCheck_MS = 5000; // 5 s

boolean motorState = false; // false = OFF, true = ON
boolean alarmState = false; // false = OFF, true = ON

float currentValue = 0.0; // to store current reading

void setup() {
  Serial.begin(115200);  
  // Initialize LCD (16 columns, 2 rows)
  lcd.begin(16, 2);
  lcd.clear();
  
  //preparing system
  printLine(0, "System Init...");
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);   // start OFF
  delay(5000);
  lcd.clear();

  printLine(0, "System Ready!");
  printLine(1, "Pump: OFF      ");
  delay(3000);
  lcd.clear();

  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW); // LOW = off for low-side switch

}

void tempCheck_task() {
  Serial.print("TEMPERATURE CHECK \n");
  sensors.requestTemperatures();               // start conversion
  float c = 0;
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    c = sensors.getTempCByIndex(i);      // read each device
    Serial.print("Sensor ");
    String line = "T" + String(i+1) + " = " + String(c, 2) + " C"; //might need to fix this line
    printLine(0, line);
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c);
    Serial.println(" °C");
  }
  
  //this turns on the motor if temp is above threshold
  if(c >= 99){
    motorState = true;
  } else {
    motorState = false;
  }
} 

void lcdPrintTask() {
  Serial.print("LCD UPDATE START\n");
  /// LCD STATUS UPDATE ///
  static uint32_t last = 0;
  static bool toggle = false;
  uint32_t now = millis();

  if (now - last >= 1000) {        // update every 1 second
    last += 1000;
    toggle = !toggle;
  }

  if (toggle) {
    printLine(0, "Pump: ON        ");
    printLine(1, "System: READY   ");
  } else {
    printLine(0, "Pump: STANDBY   ");
    printLine(1, "System: IDLE    ");
  }
  
  Serial.print("LCD UPDATE STOP\n");
}

void currentCheck_task() {
  Serial.print("CURRENT CHECK START\n");
  int adcValue = analogRead(sensorPin);
  float voltage = (adcValue / adcMax) * vRef;
  float currentValue = (voltage - zeroOffset) / sensitivity;  // in Amps

  Serial.print("Current: ");
  Serial.print(currentValue, 3);   // 3 decimal places
  Serial.println(" A");
}
  //turns alarm on/off based on current


void alarmCheck_task() {
  Serial.print("ALARM CHECK START\n");
  if (currentValue >= 10.0) { //threshold current for alarm
      alarmState = true;
    } else {
      alarmState = false;  
  }

  if (alarmState) {
    Serial.print("ALARM ON \n");
    digitalWrite(BUZZ_SW, HIGH);
    delay(1000);
    digitalWrite(BUZZ_SW, LOW);
    delay(500); //debounce
  } else {
    Serial.print("ALARM OFF \n");
    // turn buzzer OFF
    digitalWrite(BUZZ_SW, LOW);
    delay(1000);
  }
  
  Serial.print("ALARM CHECK STOP\n");
}

//
void motorCheck_task(){
  Serial.print("MOTOR CHECK START\n"); //logging start to console

  //control motor based on motorState boolean from tempCheck_task
  if (motorState) {
    digitalWrite(IN1_PIN, HIGH);
    Serial.println("Motor ON");
  } else {
    digitalWrite(IN1_PIN, LOW);
    Serial.println("Motor OFF");
  }

  Serial.print("MOTOR CHECK STOP\n"); //logging end to console
}

void loop() {
  if (tempCheck_timer >= tempCheck_MS) { tempCheck_timer = 0; tempCheck_task(); }
  if (lcdPrint_timer >= lcdPrint_MS) { lcdPrint_timer = 0; lcdPrintTask(); }
  if (currentCheck_timer >= currentCheck_MS) { currentCheck_timer = 0; currentCheck_task(); }
  if (alarmCheck_timer >= alarmCheck_MS) { alarmCheck_timer = 0; alarmCheck_task(); }
  if (motorCheck_timer >= motorCheck_MS) { motorCheck_timer = 0; motorCheck_task(); }
}
