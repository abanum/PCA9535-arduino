/*************************************************** 
 This is a library for the PCA9535 i2c port expander

 These displays use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

  This code is modified from Adafruit_MCP23017.

  Written by Manabu Ishihara.  
 BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifdef __AVR_ATtiny85__
  #include <TinyWireM.h>
  #define Wire TinyWireM
#else
  #include <Wire.h>
#endif


#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include "Arduino_PCA9535.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// minihelper to keep Arduino backward compatibility
static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
	Wire.write((uint8_t) x);
#else
	Wire.send(x);
#endif
}

static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
	return Wire.read();
#else
	return Wire.receive();
#endif
}

/**
 * Bit number associated to a give Pin
 */
uint8_t Arduino_PCA9535::bitForPin(uint8_t pin){
	return pin%8;
}

/**
 * Register address, port dependent, for a given PIN
 */
uint8_t Arduino_PCA9535::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin<8) ?portAaddr:portBaddr;
}

/**
 * Reads a given register
 */
uint8_t Arduino_PCA9535::readRegister(uint8_t addr){
	// read the current GPINTEN
	Wire.beginTransmission(PCA9535_ADDRESS | i2caddr);
	wiresend(addr);
	Wire.endTransmission();
	Wire.requestFrom(PCA9535_ADDRESS | i2caddr, 1);
	return wirerecv();
}


/**
 * Writes a given register
 */
void Arduino_PCA9535::writeRegister(uint8_t regAddr, uint8_t regValue){
	// Write the register
	Wire.beginTransmission(PCA9535_ADDRESS | i2caddr);
	wiresend(regAddr);
	wiresend(regValue);
	Wire.endTransmission();
}


/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void Arduino_PCA9535::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
	uint8_t bit=bitForPin(pin);
	regValue = readRegister(regAddr);

	// set the value for the particular bit
	bitWrite(regValue,bit,pValue);

	writeRegister(regAddr,regValue);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the PCA9535 given its HW selected address, see datasheet for Address selection.
 */
void Arduino_PCA9535::begin(uint8_t addr) {
	if (addr > 7) {
		addr = 7;
	}
	i2caddr = addr;

	Wire.begin();

	// set defaults!
	// all inputs on port A and B
	writeRegister(PCA9535_CONFIGPORT0,0xff);
	writeRegister(PCA9535_CONFIGPORT1,0xff);
}

/**
 * Initializes the default PCA9535, with 000 for the configurable part of the address
 */
void Arduino_PCA9535::begin(void) {
	begin(0);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void Arduino_PCA9535::pinMode(uint8_t p, uint8_t d) {
	updateRegisterBit(p,(d==INPUT),PCA9535_INPUTPORT0,PCA9535_INPUTPORT1);
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t Arduino_PCA9535::readGPIOAB() {
	uint16_t ba = 0;
	uint8_t a;

	// read the current GPIO output latches
	Wire.beginTransmission(PCA9535_ADDRESS | i2caddr);
	wiresend(PCA9535_INPUTPORT0);
	Wire.endTransmission();

	Wire.requestFrom(PCA9535_ADDRESS | i2caddr, 2);
	a = wirerecv();
	ba = wirerecv();
	ba <<= 8;
	ba |= a;

	return ba;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t Arduino_PCA9535::readGPIO(uint8_t b) {

	// read the current GPIO output latches
	Wire.beginTransmission(PCA9535_ADDRESS | i2caddr);
	if (b == 0)
		wiresend(PCA9535_INPUTPORT0);
	else {
		wiresend(PCA9535_INPUTPORT1);
	}
	Wire.endTransmission();

	Wire.requestFrom(PCA9535_ADDRESS | i2caddr, 1);
	return wirerecv();
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void Arduino_PCA9535::writeGPIOAB(uint16_t ba) {
	Wire.beginTransmission(PCA9535_ADDRESS | i2caddr);
	wiresend(PCA9535_OUTPUTPORT0);
	wiresend(ba & 0xFF);
	wiresend(ba >> 8);
	Wire.endTransmission();
}

void Arduino_PCA9535::digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit=bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr=regForPin(pin,PCA9535_INPUTPORT0,PCA9535_INPUTPORT1);
	gpio = readRegister(regAddr);

	// set the pin and direction
	bitWrite(gpio,bit,d);

	// write the new GPIO
	regAddr=regForPin(pin,PCA9535_OUTPUTPORT0,PCA9535_OUTPUTPORT1);
	writeRegister(regAddr,gpio);
}

void Arduino_PCA9535::pinInvert(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit=bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr=regForPin(pin,PCA9535_POLARITYINVPORT0,PCA9535_POLARITYINVPORT1);
	gpio = readRegister(regAddr);

	// set the pin and direction
	bitWrite(gpio,bit,d);

	// write the new GPIO
	regAddr=regForPin(pin,PCA9535_POLARITYINVPORT1,PCA9535_POLARITYINVPORT1);
	writeRegister(regAddr,gpio);
}


uint8_t Arduino_PCA9535::digitalRead(uint8_t pin) {
	uint8_t bit=bitForPin(pin);
	uint8_t regAddr=regForPin(pin,PCA9535_INPUTPORT0,PCA9535_INPUTPORT1);
	return (readRegister(regAddr) >> bit) & 0x1;
}

