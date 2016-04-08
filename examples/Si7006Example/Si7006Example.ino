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

Si7006 tempNHum; 

void setup() {
	
	// Initialize the Serial port:  
	Serial.begin(9600);
	Serial.println("Si7006-A20 example sketch");

	// Initialize the Si7006 library
	// You can pass nothing to light.begin() for the default I2C address (0x40)
	tempNHum.begin();
	
	if (!tempNHum.begin()) {
		Serial.println("Couldn't find Si7006");
		while (1)
			delay(1); // Do Nothing
	}
	
	// Get factory ID from sensor:
	// (Just for fun, you don't need to do this to operate the sensor)
	
	char ID[8];

	if (tempNHum.getDeviceID(ID)) {
		Serial.print("Got Sensor Part ID: 0X");
		// Default value of MSB 0x06
		for(int i = 0; i < sizeof(ID); i++) {
			Serial.print(ID[i],HEX);
		}
		Serial.println();
		
	}
	// Most library commands will return true if communications was successful,
	// and false if there was a problem. You can ignore this returned value,
	// or check whether a command worked correctly and retrieve an error code:
	else {
		byte error = tempNHum.getError();
		printError(error);
	}
	
	// Gets the Firmware Version of the chip
	// Default value is 0xFF for version 1.0 or 0x20 for version 2.0
	byte firmwareVer;
	
	if(tempNHum.getFirmwareVer(firmwareVer)) {
		Serial.print("Got Sensor Firmware Version: 0X");
		Serial.println(firmwareVer,HEX);
	}
	else {
		byte error = tempNHum.getError();
		printError(error);
	}
	
	// Gets the contents RH/Temp User Register of the sensor
	byte resolution;
	boolean voltage, heaterStatus;
	tempNHum.getTempControl(resolution, voltage, heaterStatus);
	Serial.print("Resolution is: ");
	Serial.println(resolution);
	
	// Setting the resolution and heater disable
	resolution = 0x00;
	heaterStatus = false;
	tempNHum.setTempControl(resolution, heaterStatus);
	
	// Getting heater current
	byte heaterCurrent;
	tempNHum.getHeaterControl(heaterCurrent);
	Serial.print("Heater Current is ");
	Serial.println(heaterCurrent);
	
	// Setting heater current
	tempNHum.setHeaterControl(heaterCurrent);
}


void loop() {
	float temp;
	float humidity;
	boolean mode = false;

	// Read temperature
	if(tempNHum.getTemperature(temp, mode)) {
		Serial.print("Temp *C = ");
		Serial.println(temp);
	}
	else{
		Serial.println("Failed to read temperature");
		byte error = tempNHum.getError();
		printError(error);
	}

	// Read humidity
	if(tempNHum.getHumidity(humidity, mode)) {
		Serial.print("Humidity = ");
		Serial.print(humidity);
		Serial.println("%");
	}
	else {
		Serial.println("Failed to read humidity");
		byte error = tempNHum.getError();
		printError(error);
	}

  delay(1000);
}

void printError(byte error) {
  // If there's an I2C error, this function will
  // print out an explanation.

  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error) {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}
