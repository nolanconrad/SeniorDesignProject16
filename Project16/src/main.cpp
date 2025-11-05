#include <arduino.h>
#include <math.h>

//A0 setup
int analogValue = analogRead(A0);

//pin setup 
const int greenledPin = 2;
const int redledPin = 9;          // LED output

//calculating voltage via A0
float voltage = analogValue * (5.0 / 1023.0);

// --- Constants ---
const float R_FIXED = 10000.0; // 10k resistor
const float BETA = 3950.0;     // Beta value of thermistor
const float T0 = 298.15;       // 25°C in Kelvin
const float R0 = 10000.0;      // Resistance at 25°C

// --- Temperature threshold (in °F) ---
const float THRESHOLD_F = 90.0;  // LED turns on above 86°F (~30°C)

void setup() {
  Serial.begin(9600);
  pinMode(greenledPin, OUTPUT);
  pinMode(redledPin, OUTPUT);
  Serial.println("THIS IS A TEST.`");
}

void loop() {
  int analogValue = analogRead(A0);
  float voltage = analogValue * (5.0 / 1023.0);

  // Calculate thermistor resistance
  float R_therm = R_FIXED * (voltage / (5.0 - voltage));


  // Convert resistance to temperature (Kelvin → Celsius → Fahrenheit)
  float tempK = 1.0 / (1.0/T0 + (1.0/BETA) * log(R_therm / R0));
  float tempC = tempK - 273.15;
  float tempF = (tempC * 9.0 / 5.0) + 32.0;

  // Print temperature
  Serial.print("Temperature: ");
  Serial.print(tempF);
  Serial.println(" °F");

  // Print temperature
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

// If voltage is nearly 0V or 5V, assume sensor is disconnected
  if (voltage < 0.1 || voltage > 4.9) {
    Serial.println("Thermistor disconnected!");
    digitalWrite(redledPin, HIGH);   // alert LED
    delay(5000);
    digitalWrite(redledPin, LOW);
  } 

  // --- LED control ---
  if (tempF > THRESHOLD_F) {
    digitalWrite(greenledPin, HIGH);  // Turn LED on
  } else {
    digitalWrite(greenledPin, LOW);   // Turn LED off
  }

  delay(1000);  // update every 1 second

  /* 
  if (sensorValue < 10) { //if the A0 value is basically 0, turn on the red LED
    digitalWrite(redledPin, HIGH);
  } else {
    digitalWrite(redledPin, LOW);
  }
  */

}
