#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

void setup() {
  lcd.begin(16, 2);
}

void loop() {
    lcd.clear();
    delay(1000);
    lcd.setCursor(0,0); lcd.print("Hello, 1602A!");
    lcd.setCursor(0,1); lcd.print("ESP32 online :)");
    delay(3000);
}
