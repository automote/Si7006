/* 
	Si7006-A20 library example sketch
	Lovelesh, thingTronics

This sketch shows how to use the Si7006 library to read the Silicon Labs Si7006-A20 temperature and humidity sensor.

Hardware connections:

3V3 to 3.3V
GND to GND

(WARNING: do not connect 3V3 to 5V or the sensor will be damaged!)

You will also need to connect the I2C pins (SCL and SDA) to your Arduino.
The pins are different on different Arduinos:

                    SDA    SCL
Any Arduino        "SDA"  "SCL"
Uno, Pro            A4     A5
Mega2560, Due       20     21
Leonardo            2      3
ESP8266				5      4

*/
 
#include <Wire.h>
#include <Si7006.h>

Si7006 tempHumi; 

void setup() {
  Serial.begin(9600);
  Serial.println("Si7006 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
}


void loop() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
  Serial.println();
  delay(1000);
}