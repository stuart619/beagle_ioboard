/*
Library modified to work with SPI peripheral pointers
Autor: Siddharth Kothari
*/


//    FILE: MCP4921.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.2.0
//    DATE: 2021-02-03
// PURPOSE: Arduino library for MCP4921
//     URL: https://github.com/RobTillaart/MCP4921

#include "MCP_DAC.h"

MCP4921::MCP4921(uint8_t dataOut, uint8_t clock, SPIClass *inSPI)
{
  mySPI = inSPI;
  _dataOut = dataOut;
  _clock = clock;
  _select = 0;
  _hwSPI = (dataOut == 255) || (clock == 255);
  _channels = 1;
  _maxValue = 4095;
  reset();
}

void MCP4921::reset()
{
  _gain = 1;
  _value[0] = 0;
  _value[1] = 0;
  _buffered = false;
  _active = true;
}

void MCP4921::begin(uint8_t select)
{
  _select = select;
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);

  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

  if (_hwSPI)
  {
#if defined(LOL)
    if (_useHSPI) //  HSPI
    {
      mySPI = new SPIClass(HSPI);
      mySPI->end();
      mySPI->begin(14, 12, 13, select); //  CLK=14  MISO=12  MOSI=13
    }
    else //  FSPI
    {
      mySPI = new SPIClass(FSPI);
      mySPI->end();
      mySPI->begin(18, 19, 23, select); //  CLK=18  MISO=19  MOSI=23
    }

#else //  generic hardware SPI
    mySPI->end();
    mySPI->begin();
#endif
  }
  else //  software SPI
  {
    pinMode(_dataOut, OUTPUT);
    pinMode(_clock, OUTPUT);
    digitalWrite(_dataOut, LOW);
    digitalWrite(_clock, LOW);
  }
}

uint8_t MCP4921::channels()
{
  return _channels;
}

uint16_t MCP4921::maxValue()
{
  return _maxValue;
}

#if defined(ESP32) or defined(ARDUINO_ARCH_RP2040)
void MCP4921::setGPIOpins(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t select)
{
  _clock = clk;
  _dataOut = mosi;
  _select = select;
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);

  mySPI->end(); //  disable SPI
#endif
#if defined(ESP32)

  mySPI->begin(clk, miso, mosi, select);
}
#endif

bool MCP4921::setGain(uint8_t gain)
{
  if ((0 == gain) || (gain > 2))
    return false;
  _gain = gain;
  return true;
}

uint8_t MCP4921::getGain()
{
  return _gain;
}

bool MCP4921::analogWrite(uint16_t value, uint8_t channel)
{
  if (channel >= _channels)
    return false;

  //  CONSTRAIN VALUE
  uint16_t _val = value;
  if (_val > _maxValue)
    _val = _maxValue;
  _value[channel] = value;

  //  PREPARING THE DATA TRANSFER
  uint16_t data = 0x1000;
  if (channel == 1)
    data |= 0x8000;
  if (_buffered)
    data |= 0x4000;
  if (_gain == 1)
    data |= 0x2000;

  if (_maxValue == 4095)
    data |= _val;
  else if (_maxValue == 1023)
    data |= (_val << 2);
  else
    data |= (_val << 4);
  transfer(data);
  return true;
}

uint16_t MCP4921::lastValue(uint8_t channel)
{
  return _value[channel];
}

void MCP4921::fastWriteA(uint16_t value)
{
  transfer(0x3000 | value);
}

void MCP4921::fastWriteB(uint16_t value)
{
  transfer(0xB000 | value);
}

bool MCP4921::increment(uint8_t channel)
{
  if (channel >= _channels)
    return false;
  if (_value[channel] == _maxValue)
    return false;
  return analogWrite(_value[channel] + 1, channel);
}

bool MCP4921::decrement(uint8_t channel)
{
  if (channel >= _channels)
    return false;
  if (_value[channel] == 0)
    return false;
  return analogWrite(_value[channel] - 1, channel);
}

void MCP4921::setPercentage(float perc, uint8_t channel)
{
  if (perc < 0)
    perc = 0;
  if (perc > 100)
    perc = 100;
  analogWrite((0.01 * perc * _maxValue), channel);
}

float MCP4921::getPercentage(uint8_t channel)
{
  return (_value[channel] * 100.0) / _maxValue;
}

void MCP4921::setLatchPin(uint8_t latchPin)
{
  _latchPin = latchPin;
  pinMode(_latchPin, OUTPUT);
  digitalWrite(_latchPin, HIGH);
}

void MCP4921::triggerLatch()
{
  if (_latchPin != 255)
  {
    digitalWrite(_latchPin, LOW);
    //  delay needed == 100 ns - Page 7
    //  on "slow" devices the next delay can be commented
    delayMicroseconds(1);
    digitalWrite(_latchPin, HIGH);
  }
}

void MCP4921::shutDown()
{
  _active = false;
  transfer(0x0000); //  a write will reset the values.
}

bool MCP4921::isActive()
{
  return _active;
}

void MCP4921::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
};

uint32_t MCP4921::getSPIspeed()
{
  return _SPIspeed;
}

void MCP4921::setBufferedMode(bool mode)
{
  _buffered = mode;
}

bool MCP4921::getBufferedMode()
{
  return _buffered;
}

bool MCP4921::usesHWSPI()
{
  return _hwSPI;
}

#if defined(ESP32)

void MCP4921::selectHSPI()
{
  _useHSPI = true;
}

void MCP4921::selectFSPI()
{
  _useHSPI = false;
}

bool MCP4921::usesHSPI()
{
  return _useHSPI;
}

bool MCP4921::usesFSPI()
{
  return !_useHSPI;
}

#endif

//////////////////////////////////////////////////////////////////
//
//  PROTECTED
//
void MCP4921::transfer(uint16_t data)
{
  //  DATA TRANSFER
  digitalWrite(_select, LOW);
  if (_hwSPI)
  {
    //  mySPI->beginTransaction(SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0));
    mySPI->beginTransaction(_spi_settings);
    mySPI->transfer((uint8_t)(data >> 8));
    mySPI->transfer((uint8_t)(data & 0xFF));
    mySPI->endTransaction();
  }
  else //  Software SPI
  {
    swSPI_transfer((uint8_t)(data >> 8));
    swSPI_transfer((uint8_t)(data & 0xFF));
  }
  digitalWrite(_select, HIGH);
}

//  MSBFIRST
uint8_t MCP4921::swSPI_transfer(uint8_t val)
{
  uint8_t clk = _clock;
  uint8_t dao = _dataOut;
  for (uint8_t mask = 0x80; mask; mask >>= 1)
  {
    digitalWrite(dao, (val & mask));
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);
  }
  return 0;
}


