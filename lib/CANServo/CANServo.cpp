#include "CANServo.h"
#include "CANbus.h"

void HAXTongYiCanServo::ServoEnable() {}

void HAXTongYiCanServo::ReadStatusWord() {
  uint8_t CanMsg[8];
  uint16_t id = 0x0C << 7 + NodeId;
  int index = 0;
  CanMsg[index] = 0x40;
  CanMsg[++index] = 0x41;
  CanMsg[++index] = 0x60;
  CANsend(id, CanMsg, 3);
}