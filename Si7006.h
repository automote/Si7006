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
#define Si7006_READ_OLD_TEMP                    0xE0
#define Si7006_RESET 							0xFE
#define Si7006_WRITE_HUMIDITY_TEMP_CONTR		0xE6						
#define Si7006_READ_HUMIDITY_TEMP_CONTR 		0xE7
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
			
		boolean reset(void);
			// SW Reset the sensor
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean getTempControl(byte &res, boolean voltage, boolean heater);
			// Gets the contents RH/Temp User Register of the sensor
			// res uses D7 and D0 bit
			// If res = 0, RH is set to 12 bit & temp 14 bit resolution (default)
			// If res = 1, RH is set to 8 bit & temp 12 bit resolution
			// If res = 2, RH is set to 10 bit & temp 13 bit resolution
			// If res = 4, RH is set to 11 bit & temp 11 bit resolution
			//----------------------------------------------------------
			// If voltage = false(0), VDD OK (default)
			// If voltage = true(1), VDD LOW
			//----------------------------------------------------------
			// If heater = false(0), On-chip Heater is disabled (default)
			// If heater = true(1), On-chip Heater is disabled
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean setTempControl(byte res, boolean heater);
			// Sets the contents RH/Temp User Register of the sensor
			// Gets the contents RH/Temp User Register of the sensor
			// res uses D7 and D0 bit
			// If res = 0, RH is set to 12 bit & temp 14 bit resolution (default)
			// If res = 1, RH is set to 8 bit & temp 12 bit resolution
			// If res = 2, RH is set to 10 bit & temp 13 bit resolution
			// If res = 4, RH is set to 11 bit & temp 11 bit resolution
			//----------------------------------------------------------
			// If heater = false(0), On-chip Heater is disabled (default)
			// If heater = true(1), On-chip Heater is disabled
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean getHeaterControl(byte &heaterCurrent);
			// Gets the Heater current of the On-chip Heater
			// If heaterCurrent = 0, Heater current is 3.09mA (default)
			// If heaterCurrent = 15, Heater current is 94.20mA
			// heaterCurrent is in multiples of 3.09mA
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean setHeaterControl(byte heaterCurrent);
			// Sets the Heater current of the On-chip Heater
			// If heaterCurrent = 0, Heater current is 3.09mA (default)
			// If heaterCurrent = 15, Heater current is 94.20mA
			// heaterCurrent is in multiples of 3.09mA
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
		

		boolean getDeviceID(char (&deviceID)[8]);
			// Gets the Device ID of the chip
			// Default value of MSB 0x06
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)

		boolean getFirmwareVer(byte &firmware);
			// Gets the Firmware Version of the chip
			// Default value is 0xFF for version 1.0
			// or 0x20 for version 2.0
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean getTemperature(float &temperature, boolean mode);
			// Gets the Temperature data from the sensor
			// If mode = true(1), Hold Master Mode is used
			// If mode = false(0), No Hold Master Mode is used
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean getHumidity(float &humidity, boolean mode);
			// Gets the Humidity data from the sensor
			// If mode = true(1), Hold Master Mode is used
			// If mode = false(0), No Hold Master Mode is used
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		boolean getOldTemperature(float &temperature);
			// Gets the Old Temperature data from the sensor
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() below)
			
		uint8_t crc8(const uint8_t *data, int len);
			// Returns the CRC byte generated from the data
			
		byte getError(void);
			// If any library command fails, you can retrieve an extended
			// error code using this command. Errors are from the wire library: 
			// 0 = Success
			// 1 = Data too long to fit in transmit buffer
			// 2 = Received NACK on transmit of address
			// 3 = Received NACK on transmit of data
			// 4 = Other error
		
	private:
		
		boolean readByte(byte address, byte &value);
			// Reads a byte from a LTR303 address
			// Address: LTR303 address (0 to 15)
			// Value will be set to stored byte
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() above)
	
		boolean writeByte(byte address, byte value);
			// Write a byte to a LTR303 address
			// Address: LTR303 address (0 to 15)
			// Value: byte to write to address
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() above)

		boolean readUInt(byte address, unsigned int &value);
			// Reads an unsigned integer (16 bits) from a LTR303 address (low byte first)
			// Address: LTR303 address (0 to 15), low byte first
			// Value will be set to stored unsigned integer
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() above)

		boolean read1ByteData(byte address1, byte address2, byte &value);
			// Reads a byte from a Si7006 sensor when provided with 2 addresses
			// Address: Si7006 address (0 to 15)
			// Value will be set to stored byte
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() above)
			
		boolean read4ByteData(byte address1, byte address2, char (&value)[4]);
			// Reads an unsigned long (32 bits) from a Si7006 address (high byte first)
			// Address: Si7006 register address (0 to 15), high byte first
			// Value will be set to stored unsigned long
			// Returns true (1) if successful, false (0) if there was an I2C error
			// (Also see getError() above)
		
		byte _i2c_address;
		byte _error;
};

#endif