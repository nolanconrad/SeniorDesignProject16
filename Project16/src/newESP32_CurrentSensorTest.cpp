#include <Arduino.h>  

int sensorPin = 32;     // ACS712 output connected to GPIO 32
const float sensitivity = 0.066;  // 66 mV/A for ACS712-30A
const float adcMax = 4095.0;      // 12-bit ADC
const float vRef = 3.3;           // ESP32 ADC reference voltage
const float zeroOffset = 2.5;     // No-current output voltage (V)

void setup() {
  Serial.begin(9600);
}

void loop() {
  int adcValue = analogRead(sensorPin);
  float voltage = (adcValue / adcMax) * vRef;
  float current = (voltage - zeroOffset) / sensitivity;  // in Amps

  Serial.print("Current: ");
  Serial.print(current, 3);   // 3 decimal places
  Serial.println(" A");

  delay(500);  // wait 0.5 seconds
}
