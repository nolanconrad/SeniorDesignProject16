#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  sensors.begin();
  sensors.setWaitForConversion(false);
}

void loop() {
  Serial.println("Scanning...");
  Serial.print("Device count: ");
  Serial.println(sensors.getDeviceCount());
  sensors.requestTemperatures();

  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    float c = sensors.getTempCByIndex(i);
    Serial.print("T");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c);
    Serial.println(" C");
  }
  delay(2000);
}
