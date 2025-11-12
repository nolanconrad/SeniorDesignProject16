// DRV8871 ON/OFF (no PWM)
// IN1 on GPIO18, IN2 tied to GND

#include <Arduino.h>
constexpr int IN1_PIN = 23;

void setup() {
  Serial.begin(115200);
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);   // start OFF
  Serial.println("Ready: motor OFF");
}

void loop() {
  // Turn ON
  digitalWrite(IN1_PIN, HIGH);
  Serial.println("Motor ON");
  delay(3000);                  // ON for 3 seconds

  // Turn OFF
  digitalWrite(IN1_PIN, LOW);
  Serial.println("Motor OFF");
  delay(3000);                  // OFF for 3 seconds
}
