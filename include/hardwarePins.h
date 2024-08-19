#pragma once

#include "Arduino.h"

// SPI-2 related
#define SPI_2_MOSI 35 
#define SPI_2_CLOCK 36
#define OUTPUT_CS_1 11 // 4-20mA output channel 1
#define OUTPUT_CS_2 12 // 4-20mA output channel 2
#define OUTPUT_CS_3 13 // 4-20mA output channel 3

// SPI-3 related
#define SPI_3_MOSI 48
#define SPI_3_CLOCK 47
#define SPI_3_MISO 21
#define SD_CS 37 // SD card chip select
#define OUTPUT_CS_4 14 // 4-20mA output channel 4
#define OUTPUT_CS_5 2 // 4-20mA output channel 5

// CAN bus
#define CAN_RX 39 // CAN bus RX
#define CAN_TX 38 // CAN bus TX

// I2C
#define I2C_SDA 8 // I2C SDA
#define I2C_SCL 9 // I2C SCL

// Relays
#define RELAY_1 1 // Relay 1
#define RELAY_2 46 // Relay 2
#define RELAY_3 10 // Relay 3
#define RELAY_4 17 // Relay 4
#define RELAY_5 18 // Relay 5

// Leds
#define LED_RED 41
#define LED_GREEN 42
#define LED_BLUE 40

/*---------------------------MICROCONTROLLER PERIPHERAL POINTERS-----------------------------*/
/*---------------------------DO NOT TOUCH---------------------------------------------------*/

/*---------------------------DO NOT TOUCH---------------------------------------------------*/
/*---------------------------MICROCONTROLLER PERIPHERAL POINTERS-----------------------------*/