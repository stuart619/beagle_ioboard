/* Relay.h
*  
*  Contains functions that can be called to control onboard relays
*/

#include "Arduino.h"
#include "../../include/hardwarePins.h"

/**
 * initRelays() - Initializes the relay pins as outputs
 * Call this function in void setup() in main.cpp
 * @param none
 * @return none
 */
void initRelays();

/**
 * setRelay() - Sets the state of a relay
 *
 * @param relay_channel - the channel of the relay to set (1,2 3,4 or 5)
 * @param state - the state to set the relay to (HIGH or LOW)
 * @return sum of `values`, or 0.0 if `values` is empty.
 */
void setRelay(uint8_t relay_channel, uint8_t state);