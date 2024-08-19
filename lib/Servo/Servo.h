#pragma once

#include <Arduino.h>
#include "CANbus.h"

const uint32_t SERVO_POSITIVE_MAX = 0x00230000; //35 rounds * 65536
const uint32_t SERVO_POSITIVE_MIN = 0x00000000; //0 rounds * 65536
const uint32_t SERVO_NEGTIVE_MIN = 0XFFDD0000; //0x100000000 - 35 rounds * 65536
const uint32_t SERVO_NEGTIVE_MAX = 0xFFFFFFFF; 

#define SERVO_SCALING_FACTOR         0.03 // scaling factor for controlling servo

// can comm for servo motor control, PDO mode
const uint16_t RPDO1 = 0x200;
const uint16_t RPDO2 = 0x300;
const uint16_t RPDO3 = 0x400;
const uint16_t RPDO4 = 0x500;
const uint16_t ServoNodeID = 0x0A;
const uint16_t TPDO1 = 0x180;
const uint16_t TPDO2 = 0x280;
const uint16_t TPDO3 = 0x380;
const uint16_t TPDO4 = 0x480;
const uint16_t SDOADDRESS = 0x0580;

const uint16_t ThisNodeId = 0x00;
const uint16_t HB_FUNC = 0x700;


enum class HBSTATE {
  HB_BOOTUP = 0,
  HB_STOPPED = 4,
  HB_OPERATIONAL = 5,
  HB_PREOPERATINAL = 127
};

enum PPCOMMAND {
  PPPAUSE = 0x03,
  PPSTOP = 0x05,
  PPRESUME = 0x0F,
  PPSTART = 0x1F
};

void initServo();

void can_heartbeat_publish(HBSTATE hbState);
void can_nmt_start_publish();
void can_set_pp_mode();
void setServoTarget(uint32_t position, uint32_t speed);
//  void can_request_status_word();

void runPPcommand(PPCOMMAND cmd);
void startPP();
void pausePP();
void resumePP();
void stopPP();

void can_change_servo_mode();
void CAN_TPDO_CALLBACK(twai_message_t message);
// void getMotorRounds();

void setServoposition(int cmd);

void zero_return_mode();

uint32_t getCurrentPosition();
uint32_t getServoPosition();