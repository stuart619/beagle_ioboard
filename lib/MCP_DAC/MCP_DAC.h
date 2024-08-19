#pragma once
//
//    FILE: MCP4921.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.2.0
//    DATE: 2021-02-03
// PURPOSE: Arduino library for MCP4921
//     URL: https://github.com/RobTillaart/MCP4921


#include "Arduino.h"
#include "SPI.h"



class MCP4921
{
public:

  MCP4921(uint8_t dataOut = 255, uint8_t clock = 255, SPIClass *mySPI = &SPI);

  //       if only select is given ==> HW SPI
  void     begin(uint8_t select);

  //       0 or 1
  uint8_t  channels();
  //       255 (8 bit) or 1023 (10 bit) or 4095 (12 bit)
  uint16_t maxValue();

  //       gain = 1 or 2
  bool     setGain(uint8_t gain = 1);
  uint8_t  getGain();

  bool     analogWrite(uint16_t value, uint8_t channel = 0);
  uint16_t lastValue(uint8_t channel = 0);
  void     fastWriteA(uint16_t value);
  void     fastWriteB(uint16_t value);

  bool     increment(uint8_t channel = 0);
  bool     decrement(uint8_t channel = 0);

  //       convenience wrappers
  //       percentage = 0..100.0%
  void     setPercentage(float percentage, uint8_t channel = 0);
  float    getPercentage(uint8_t channel = 0);

  //       trigger LDAC = LatchDAC pin - if not set it does nothing
  void     setLatchPin( uint8_t latchPin);
  void     triggerLatch();

  //       shutDown - Page 21  ==> write will wake up.
  void     shutDown();
  bool     isActive();

  //       speed in Hz
  void     setSPIspeed(uint32_t speed);
  uint32_t getSPIspeed();

  //
  //       MCP49xxx series only
  //
  //       see page 20 ==> not functional for MCP48xx series.
  void     setBufferedMode(bool mode = false);
  bool     getBufferedMode();

  // debugging
  void     reset();
  bool     usesHWSPI();


#if defined(ESP32)                    // ESP32 specific

  void     selectHSPI();
  void     selectFSPI();
  bool     usesHSPI();
  bool     usesFSPI();

  // to overrule the ESP32s default hardware pins
  void     setGPIOpins(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t select);
#endif


protected:
  uint8_t  _dataOut;              //  Data out Pin (MOSI)
  uint8_t  _clock;                //  Clock Pin (SCK)
  uint8_t  _select;               //  Chip Select Pin (CS)
  uint8_t  _latchPin = 255;       //  Latch-DAC Pin (LDAC)
  bool     _hwSPI;                //  Hardware SPI (true) or Software SPI (false)
  uint32_t _SPIspeed = 16000000;  //  SPI-Bus Frequency

  uint8_t  _channels;             //  Number of DAC-Channels of a given Chip
  uint16_t _maxValue;             //  Maximum value of a given Chip
  uint16_t _value[2];             //  Current value  (cache for performance)
  uint8_t  _gain;                 //  Programmable Gain Amplifier variable
  bool     _buffered = false;     //  Buffer for the Reference Voltage of the MCP49XX Series Chips
  bool     _active   = true;      //  Indicates shutDown mode.

  void     transfer(uint16_t data);
  uint8_t  swSPI_transfer(uint8_t d);


  SPIClass    * mySPI;

  SPISettings _spi_settings;

#if defined(ESP32)

  bool     _useHSPI = true;

#endif
};


//  -- END OF FILE --

