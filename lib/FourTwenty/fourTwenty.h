#pragma once


#include "MCP_DAC.h"
#include "INA237.h"
#include "../../include/hardwarePins.h"
#include "peripherals.h"

/**
 * initFourTwenty() - Initializes the 4-20 pins transmitters and receivers
 * Call this function in void setup() in main.cpp
 * @param none
 * @return none
 */
void initFourTwenty();

/**
 * setFourTwentyDAC() - Sets the value of the DAC for a given channel
 *
 * @param channel - the putput channel of the DAC to set (1,2,3 or 4)
 * @param value - the value to set the DAC to (0-4095) (0 corresponds to 0mA, 4095 corresponds to 20mA)
 * @return none
 */
void setFourTwentyDAC(int channel, int value);


/**
 * getFourTwentyInputCurrent() - Gets the current value of the 4-20 input for a given channel
 *
 * @param channel - the input channel of the 4-20 input to read (1,2,3 or 4)
 * @return the current value of the 4-20 input in mA
 */
uint16_t getFourTwentyInputCurrent(int channel);


