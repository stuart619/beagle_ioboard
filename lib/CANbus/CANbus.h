#include "../../include/hardwarePins.h"
#include "Arduino.h"
#include "driver/twai.h"

#define RX_PIN CAN_RX
#define TX_PIN CAN_TX

void CANinit();
int CANsend(uint32_t identifier, uint8_t dataArray[], uint8_t datalen );

void CANrecv(void);
