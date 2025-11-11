// DRV8871 one-direction sanity test
// Wire ESP32 GPIO25 → DRV8871 IN1
// Tie DRV8871 IN2 → GND
// Make sure VM (battery +) is on DRV8871 VM, and grounds are common.
#include <arduino.h>
#include <math.h>
#define IN1 18

void setup() {
  pinMode(IN1, OUTPUT);
}

void loop() {
  digitalWrite(IN1, HIGH);   // full ON
  Serial.println("Motor ON for 5 seconds");
  delay(5000);
  digitalWrite(IN1, LOW);
  Serial.println("Motor OFF for 5 seconds"); 
}