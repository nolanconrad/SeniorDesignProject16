#include <Arduino.h>
#include <LiquidCrystal.h>

// LCD pins: RS=14, E=27, D4=26, D5=25, D6=33, D7=17
// (Wire RW to GND on the hardware.)
LiquidCrystal lcd(14, 27, 26, 25, 33, 17);

void setup() {
  lcd.begin(16, 2);        // 16 columns, 2 rows
  lcd.clear();
  lcd.setCursor(0, 0);     // column 0, row 0 (top)
  lcd.print("Hello, 1602A!");
  lcd.setCursor(0, 1);     // column 0, row 1 (bottom)
  lcd.print("ESP32 online :)");
}

void loop() {
  // nothing needed
}
