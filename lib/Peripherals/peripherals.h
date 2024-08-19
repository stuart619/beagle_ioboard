#pragma once

#include "Arduino.h"
#include "../../include/hardwarePins.h"
#include "SPI.h"
#include "Wire.h"

// SPI Peripheral objects
extern SPIClass hspi;
extern SPIClass fspi;

extern TwoWire i2cPort;

