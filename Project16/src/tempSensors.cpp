#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4  // D4 = GPIO4 on your board


//TEMPERATURE SENSOR MODULE

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  sensors.begin();

  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount());
  Serial.println(" DS18B20 sensor(s).");
}

void loop() {
  sensors.requestTemperatures();               // start conversion
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    float c = sensors.getTempCByIndex(i);      // read each device
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c);
    Serial.println(" °C");
  }
  delay(1000);

  //CURRENT SENSOR MODULE
  
  //MOTOR DRIVER MODULE

}
