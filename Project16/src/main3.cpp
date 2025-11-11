#include <Arduino.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4  // D4 = GPIO4 on your board
#define IN1 18

//these two lines make the temp sensors work as a bus
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  //setup for temp sensors  
    Serial.begin(115200); //baud rate of the serial monitor
    sensors.begin(); //starts the dallas temperature module
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount());
    Serial.println(" DS18B20 sensor(s).");
  
  //setup for current sensor

  //setup for motor driver
    pinMode(IN1, OUTPUT);
}

void loop() {

  //TEMPERATURE SENSOR MODULE - takes the temperature readings from the sensors and prints them to the serial monitor  
 
  sensors.requestTemperatures();               // start conversion
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
        float c = sensors.getTempCByIndex(i);      // read each device
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(c);
        Serial.println(" °C");
    }

  //CURRENT SENSOR MODULE

  //MOTOR DRIVER MODULE - activates motor while temperature is above 100F
    //this isnt going to work in current state
    if(temp = 1) { digitalWrite(IN1, HIGH);   // pump fully on
    } else {
        digitalWrite(IN1, LOW);    // pump off
    };

    delay(1000);
}