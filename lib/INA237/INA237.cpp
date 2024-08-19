/*!
 * @file INA237.cpp
 * @section INA237.cpp Description
 * Arduino Library for accessing the INA237 current sensor\n
 * 
 * Author: Siddharth Kothari
 */

#include "INA237.h"

INA237::INA237(TwoWire *twi = &Wire, uint8_t sensorAddress = 0x40) {
    wire = twi;
    deviceAddress = sensorAddress;
}

/*
Calculation for shunt cal
SHUNT_CAL = 819.2 x 10^6 x CURRENT_LSB x RSHUNT
CURRENT_LSB = max_expected_current/32768
MAX_EXPECTED_CURRENT = 30mA
RSHUNT = 4.99

so, SHUNT_CAL= 819.2 x 10^6 x (max_expected_current/32768) x 4.99
    SHUNT_CAL= 819.2 x 10^6 x (0.03/32768) x 4.99
    = 3742.5 (0xE9E)

*/


void INA237::writeWord(const uint8_t addr, const uint16_t data, const uint8_t deviceAddress)
{
    /*! @brief     Write 2 bytes to the specified I2C address
        @details   Standard I2C protocol is used, but a delay of I2C_DELAY microseconds has been
                   added to let the INAxxx devices have sufficient time to process the data
        @param[in] addr I2C address to write to
        @param[in] data 2 Bytes to write to the device
        @param[in] deviceAddress Address on the I2C device to write to */
    wire->beginTransmission(deviceAddress); // Address the I2C device
    wire->write(addr);                      // Send register address to write
    wire->write((uint8_t)(data >> 8));      // Write the first (MSB) byte
    wire->write((uint8_t)data);             // and then the second byte
    wire->endTransmission();                // Close transmission and actually send data
    delayMicroseconds(I2C_DELAY);          // delay required for sync
}

int16_t INA237::readWord(const uint8_t addr, const uint8_t deviceAddress)
{
    /*! @brief     Read one word (2 bytes) from the specified I2C address
        @details   Standard I2C protocol is used, but a delay of I2C_DELAY microseconds has been
                   added to let the INAxxx devices have sufficient time to get the return data ready
        @param[in] addr I2C address to read from
        @param[in] deviceAddress Address on the I2C device to read from
        @return    integer value read from the I2C device */
    wire->beginTransmission(deviceAddress);       // Address the I2C device
    wire->write(addr);                            // Send register address to read
    wire->endTransmission();                      // Close transmission
    delayMicroseconds(I2C_DELAY);                // delay required for sync
    wire->requestFrom(deviceAddress, (uint8_t)2); // Request 2 consecutive bytes
    return ((uint16_t)wire->read() << 8) | wire->read();
}

void INA237::setConfiguration()
{
    uint16_t config = 0x0000;
    config &= ~(1 << BIT_CONFIG_RST);      // Reset
    config |= 1 << BIT_CONFIG_CONVDLY;     // Conversion Delay
    config &= ~(1 << BIT_CONFIG_ADCRANGE); // ADC Range
    writeWord(REGISTER_CONFIG, config, deviceAddress);
}

void INA237::setCalibration()
{
    uint16_t calibration = 0x0000;
    calibration |= 0xE9E; // Shunt Calibration
    calibration &= ~(1 << 15); //reserved
    writeWord(REGISTER_SHUNT_CAL, calibration, deviceAddress);
}

uint16_t INA237::getCurrent()
{
    uint16_t rawCurrent = readWord(REGISTER_CURRENT, deviceAddress);
    float calc = (float)(((float)rawCurrent * INA327_CURRENT_LSB)/10000000000.0); 
    return uint16_t(calc);
}

uint16_t INA237::getBusMilliVolts()
{
    uint16_t rawVoltage = readWord(REGISTER_VBUS, deviceAddress);
    rawVoltage = (uint16_t)((rawVoltage * INA327_BUS_VOLTAGE_LSB)/1000);
    return rawVoltage;
}



bool INA237::begin()
{
    /*! @brief     Initialize the I2C bus and the INA237 device
        @details   This function initializes the I2C bus and the INA237 device. It returns true if
                     the device is found on the I2C bus, false otherwise
        @param[in] deviceAddress Address on the I2C bus where the INA237 device is located
        @return    true if the device is found on the I2C bus, false otherwise */
    wire->begin(I2C_SDA, I2C_SCL, INA_I2C_STANDARD_MODE);
    // Additional initialization code for your sensor, if needed
    setConfiguration();
    setCalibration();

    return true;
}

