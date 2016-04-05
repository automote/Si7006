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


#include <Si7006.h>
#include <Wire.h>

Si7006::Si7006() {
	// Si7006 object
}


boolean Si7006::begin(uint8_t i2caddr) {
	// Initialize Si7006 library with default address (0x40)
	// Always returns true
	
	Wire.begin();
	_i2caddr = i2caddr;
	reset();
	//return (readStatus() == 0x40);
	return true;
}

uint16_t Si7006::readStatus(void) {
	// Returns the status of the sensor
	
	writeCommand(SHT31_READSTATUS);
	Wire.requestFrom(_i2caddr, (uint8_t)3);
	uint16_t stat = Wire.read();
	stat <<= 8;
	stat |= Wire.read();
	Serial.println(stat, HEX);
	return stat;
}

void Si7006::reset(void) {
	// SW Reset the sensor
	
	writeCommand(SHT31_SOFTRESET);
	delay(10);
}

void Si7006::heater(boolean h) {
	// Initializes the internal heater
	
	if (h)
	writeCommand(SHT31_HEATEREN);
	else
	writeCommand(SHT31_HEATERDIS);
}

float Si7006::readTemperature(void) {
	// Returns the temperature from the sensor
	if (! readTempHum()) return NAN;
	return temp;
}  

float Si7006::readHumidity(void) {
	// Returns the relative humidity from the sensor

	if (! readTempHum()) return NAN;
	return humidity;
}

uint8_t Si7006::crc8(const uint8_t *data, int len) {
	// Returns the CRC byte generated from the data
	/*
	 *
	 * CRC-8 formula from page 14 of SHT spec pdf
	 *
	 * Test data 0xBE, 0xEF should yield 0x92
	 *
	 * Initialization data 0xFF
	 * Polynomial 0x31 (x8 + x5 +x4 +1)
	 * Final XOR 0x00
	*/

	const uint8_t POLYNOMIAL(0x31);
	uint8_t crc(0xFF);

	for ( int j = len; j; --j ) {
	  crc ^= *data++;

	  for ( int i = 8; i; --i ) {
	crc = ( crc & 0x80 )
	  ? (crc << 1) ^ POLYNOMIAL
	  : (crc << 1);
	  }
	}
	return crc;
}

// Private Functions

boolean Si7006::readTempHum(void) {
	// Reads the temperature and relative humidity from the sensor
	
	uint8_t readbuffer[6];

	writeCommand(SHT31_MEAS_HIGHREP);

	delay(500);
	Wire.requestFrom(_i2caddr, (uint8_t)6);
	if (Wire.available() != 6) 
	return false;
	for (uint8_t i=0; i<6; i++) {
	readbuffer[i] = Wire.read();
	//  Serial.print("0x"); Serial.println(readbuffer[i], HEX);
	}
	uint16_t ST, SRH;
	ST = readbuffer[0];
	ST <<= 8;
	ST |= readbuffer[1];

	if (readbuffer[2] != crc8(readbuffer, 2)) return false;

	SRH = readbuffer[3];
	SRH <<= 8;
	SRH |= readbuffer[4];

	if (readbuffer[5] != crc8(readbuffer+3, 2)) return false;

	// Serial.print("ST = "); Serial.println(ST);
	double stemp = ST;
	stemp *= 175;
	stemp /= 0xffff;
	stemp = -45 + stemp;
	temp = stemp;

	//  Serial.print("SRH = "); Serial.println(SRH);
	double shum = SRH;
	shum *= 100;
	shum /= 0xFFFF;

	humidity = shum;

	return true;
}

void Si7006::writeCommand(uint16_t cmd) {
	// Writes command bytes to sensor
	
	Wire.beginTransmission(_i2caddr);
	Wire.write(cmd >> 8);
	Wire.write(cmd & 0xFF);
	Wire.endTransmission();  
}