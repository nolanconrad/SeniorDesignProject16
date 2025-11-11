#include <Arduino.h>

const int BUZZ_SW = 25; // the transistor/MOSFET control pin

void setup() {
  pinMode(BUZZ_SW, OUTPUT);
  digitalWrite(BUZZ_SW, LOW); // LOW = off for low-side switch
}

void loop() {
  // turn buzzer ON
  digitalWrite(BUZZ_SW, HIGH);
  delay(1000);

  // turn buzzer OFF
  digitalWrite(BUZZ_SW, LOW);
  delay(1000);
}
