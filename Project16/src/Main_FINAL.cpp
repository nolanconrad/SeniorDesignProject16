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
elapsedMillis tempCheck_task, lcdPrint_task, currentCheck_task, alarmCheck;     // each starts at 0 ms since boot
const uint32_t A_MS = 1000;   // 1 s
const uint32_t B_MS = 2000;   // 2 s
const uint32_t C_MS = 5000;   // 5 s


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


void loop() {

  int adcValue = analogRead(sensorPin);
  float voltage = (adcValue / adcMax) * vRef;
  float current = (voltage - zeroOffset) / sensitivity;  // in Amps


  static uint32_t last = 0;
  static bool toggle = false;

  /// LCD STATUS UPDATE ///
  uint32_t now = millis();
  if (now - last >= 1000) {        // update every 1 second
    last += 1000;
    toggle = !toggle;

    if (toggle) {
      printLine(0, "Pump: ON        ");
      printLine(1, "System: READY   ");
    } else {
      printLine(0, "Pump: STANDBY   ");
      printLine(1, "System: IDLE    ");
    }
    
/// TEMPERATURE SENSOR ///
  sensors.requestTemperatures();               // start conversion
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    float c = sensors.getTempCByIndex(i);      // read each device
    Serial.print("Sensor ");
    String line = "T" + String(i+1) + " = " + String(c, 2) + " C"; //might need to fix this line
    printLine(0, line);
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c);
    Serial.println(" °C");
  }
  delay(1000);

/// MOTOR DRIVER ///
    // Turn ON
  digitalWrite(IN1_PIN, HIGH);
  Serial.println("Motor ON");
  delay(3000);                  // ON for 3 seconds

  // Turn OFF
  digitalWrite(IN1_PIN, LOW);
  Serial.println("Motor OFF");
  delay(3000);  

/// CURRENT SENSOR ///
  Serial.print("Current: ");
  Serial.print(current, 3);   // 3 decimal places
  Serial.println(" A");

  delay(500);  // wait 0.5 seconds

//BUZZER ALARM ///
    // turn buzzer ON
  digitalWrite(BUZZ_SW, HIGH);
  delay(1000);

  // turn buzzer OFF
  digitalWrite(BUZZ_SW, LOW);
  delay(1000);


}
