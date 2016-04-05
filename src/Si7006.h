/*
	Si7006 Temperature and humidity sensor library for Arduino
	Lovelesh, thingTronics
	
The MIT License (MIT)

Copyright (c) 2015 thingTronics Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

version 0.1
*/

#ifndef Si7006_h
#define Si7006_h

#include "Arduino.h"

#define Si7006_ADDR   0x40 // default address

// Si7006 register addresses
#define Si7006_MEAS_REL_HUMIDITY_MASTER_MODE    0xE5
#define Si7006_MEAS_REL_HUMIDITY_NO_MASTER_MODE 0xF5
#define Si7006_MEAS_TEMP_MASTER_MODE            0xE3
#define Si7006_MEAS_TEMP_NO_MASTER_MODE         0xF3
#define Si7006_READ_TEMP                        0xE0
#define Si7006_RESET 							0xFE
#define Si7006_WRITE_HUMIDITY_TEMP				0xE6						
#define Si7006_READ_HUMIDITY_TEMP				0xE7
#define Si7006_WRITE_HEATER_CONTR				0x51
#define Si7006_READ_HEATER_CONTR				0x11
#define Si7006_READ_ID_LOW_0					0xFA
#define Si7006_READ_ID_LOW_1					0x0F
#define Si7006_READ_ID_HIGH_0					0xFC
#define Si7006_READ_ID_HIGH_1					0xC9
#define Si7006_FIRMWARE_0						0x84
#define Si7006_FIRMWARE_1						0xB8


class Si7006 {
	public:
		Si7006(void);
			// Si7006 object
			
		boolean begin();
			// Initialize Si7006 library with default address (0x40)
			// Always returns true
			
		uint16_t readStatus(void);
			// Returns the status of the sensor
			
		void reset(void);
			// SW Reset the sensor
			
		void heater(boolean);
			// Initializes the internal heater
			
		float readTemperature(void);
			// Returns the temperature from the sensor
			
		float readHumidity(void);
			// Returns the relative humidity from the sensor
			
		uint8_t crc8(const uint8_t *data, int len);
			// Returns the CRC byte generated from the data
		
	private:
		boolean readTempHum(void);
			// Reads the temperature and relative humidity from the sensor
			
		void writeCommand(uint16_t cmd);
			// Writes command bytes to sensor

		byte _i2c_address;
		float humidity, temp;
};

#endif