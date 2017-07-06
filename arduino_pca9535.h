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

#ifndef _Arduino_PCA9535_H_
#define _Arduino_PCA9535_H_

// Don't forget the Wire library
#ifdef __AVR_ATtiny85__
#include <TinyWireM.h>
#else
#include <Wire.h>
#endif

class Arduino_PCA9535 {
public:
  void begin(uint8_t addr);
  void begin(void);

  void pinMode(uint8_t p, uint8_t d);
  void digitalWrite(uint8_t p, uint8_t d);
  void pinInvert(uint8_t p, uint8_t d);
  uint8_t digitalRead(uint8_t p);

  void writeGPIOAB(uint16_t);
  uint16_t readGPIOAB();
  uint8_t readGPIO(uint8_t b);

//  void setupInterrupts(uint8_t mirroring, uint8_t open, uint8_t polarity);
//  void setupInterruptPin(uint8_t p, uint8_t mode);
//  uint8_t getLastInterruptPin();
//  uint8_t getLastInterruptPinValue();

 private:
  uint8_t i2caddr;

  uint8_t bitForPin(uint8_t pin);
  uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);

  uint8_t readRegister(uint8_t addr);
  void writeRegister(uint8_t addr, uint8_t value);

  /**
   * Utility private method to update a register associated with a pin (whether port A/B)
   * reads its value, updates the particular bit, and writes its value.
   */
//  void updateRegisterBit(uint8_t p, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr);

};

#define PCA9535_ADDRESS 0x20

// registers
#define PCA9535_INPUTPORT0 0x00
#define PCA9535_INPUTPORT1 0x01
#define PCA9535_OUTPUTPORT0 0x02
#define PCA9535_OUTPUTPORT1 0x03
#define PCA9535_POLARITYINVPORT0 0x04
#define PCA9535_POLARITYINVPORT1 0x05
#define PCA9535_CONFIGPORT0 0x06
#define PCA9535_CONFIGPORT1 0x07

#endif
