
/*!
 * @file INA237.h
 * @section INA237 Description
 * Arduino Library for accessing the INA237 current sensor\n
 * 
 * Author: Siddharth Kothari
 */

#pragma once

#include "Arduino.h"
#include "../../include/hardwarePins.h"
#include <Wire.h>



/************************************************************************************************
** Declare constants used in the class                                                         **
************************************************************************************************/
#ifndef INA_I2C_MODES                               // I2C related constants
#define INA_I2C_MODES                               // Guard code to prevent multiple defs
const uint32_t INA_I2C_STANDARD_MODE{100000};       ///< Default normal I2C 100KHz speed
const uint32_t INA_I2C_FAST_MODE{400000};           ///< Fast mode
const uint32_t INA_I2C_FAST_MODE_PLUS{1000000};     ///< Really fast mode
const uint32_t INA_I2C_HIGH_SPEED_MODE{3400000};    ///< Turbo mode
#endif      

/**************************************************************************************************
 * Register addresses for the INA237 sensor                                                        *
**************************************************************************************************/
const uint16_t REGISTER_CONFIG{0x00};
const uint16_t REGISTER_ADC_CONFIG{0x01};
const uint16_t REGISTER_SHUNT_CAL{0x02};
const uint16_t REGISTER_VSHUNT{0x04};
const uint16_t REGISTER_VBUS{0x05};
const uint16_t REGISTER_DIETEMP{0x06};
const uint16_t REGISTER_CURRENT{0x07};
const uint16_t REGISTER_POWER{0x08};
const uint16_t REGISTER_DIAG_ALRT{0xB};
const uint16_t REGISTER_SOVL{0xC};
const uint16_t REGISTER_SUVL{0x0D};
const uint16_t REGISTER_BOVL{0x0E};
const uint16_t REGISTER_BUVL{0xF};
const uint16_t REGISTER_TEMP_LIMIT{0x10};
const uint16_t REGISTER_PWR_LIMIT{0x11};
const uint16_t REGISTER_MANUFACTURER_ID{0x3E};

const uint8_t  I2C_DELAY{10}; // Delay in us between I2C read/write operations

/**************************************************************************************************
 * Bit masks for the CONFIG register                                                                *
**************************************************************************************************/

#define BIT_CONFIG_RST 15
#define BIT_CONFIG_CONVDLY 6 //13-6, 8 bit value
#define BIT_CONFIG_ADCRANGE 4 //0 for 163.84mV, 1 for 40.96mV

#define INA327_BUS_VOLTAGE_LSB 3125 // 3.125mV x 1000 uV 
#define INA327_CURRENT_LSB 9155273 //  9.155273 ×10^-4 mA (assuming max current= 30mA) (divide entire calc by 10^-10 for mA)




class INA237 {
public:
  INA237(TwoWire *twi, uint8_t sensorAddress);
  bool begin();
  void setConfiguration();
  void setCalibration();
  uint16_t getCurrent();
  uint16_t getBusMilliVolts();

protected:
  TwoWire *wire;

private:
  int16_t readWord(const uint8_t addr, const uint8_t deviceAddress);
  void writeWord(const uint8_t addr, const uint16_t data, const uint8_t deviceAddress);
  uint8_t deviceAddress;
};