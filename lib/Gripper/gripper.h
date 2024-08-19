#pragma once

#include "cylinder.h"


#define SEQ_START 0
#define SEQ_TRACKING 1
#define SEQ_CUTTING 2
#define SEQ_COMPLETE 3
#define SEQ_STOPPED 4
#define STARTUP_POS 100

// #define DELAY_DONTROL_HEIGHT 3000
// #define DELAY_ROTATE_BACK 3000
// #define DELAY_GRAB 3000
// #define DELAY_CLOSE 3000
// #define DELAY_SECOND_CUTTER_EXTEND 3000
// #define DELAY_SECOND_CUTTER_RETRACT 3000
// #define DELAY_RETRACT 3000
// #define DELAY_ROTATE 3000
// #define DELAY_RELEASE 3000
// #define DELAY_RESET 3000
// #define DELAY_COMPLETE 3000

#define DELAY_ROTATE_BACK 200
#define DELAY_GRAB 550 //100
#define DELAY_CLOSE 1300 //800
#define DELAY_SECOND_CUTTER_EXTEND 600
#define DELAY_SECOND_CUTTER_RETRACT 300
#define DELAY_RETRACT 500
#define DELAY_ROTATE 300
#define DELAY_RELEASE 300//400
#define DELAY_RESET 1000 //500
#define DELAY_COMPLETE 1200//1000

enum GripperStates {
  STANDBY = 0,
  CONTROL_VERTICAL_HEIGHT = 1,
  ROTATE_GRIPPER_BACK = 2,
  NC_AND_EXTEND = 3,
  CLOSE_GRIPPER = 4,
  SECOND_CUTTER_EXTEND =5,
  RETRACT = 6,
  ROTATE_GRIPPER = 7,
  OPEN_GRIPPER = 8,
  RESET_POS = 9,
  RESET_SYS = 10
};

class Gripper {
private:
  uint8_t pin_gripper_open;
  uint8_t pin_gripper_close;
  uint8_t pin_rotation;
  CylinderPneumatic *ptr_hor_cyl;
  CylinderPneumatic *ptr_ver_cyl;

  unsigned long timestamp;
  int state;
  bool gripper_mode;
  int original_pos;
  int trig;
  float ver_cmd_val;

public:
  int seqRet;

  Gripper(uint8_t pin_open, uint8_t pin_close, uint8_t pin_rotate, CylinderPneumatic *ptr_hor, CylinderPneumatic *ptr_ver);

  void init();
  void gripper(int action);
  int seq();

  // Getters for internals of gripper
  int getSeqTimestamp();
  int getGripperState();
  int getOriginalPosition();
  int getTrigVal();
  float getVerCmdVal();

  // Setters for internals of gripper
  void setGripperState(int new_state);
  void setOriginalPosition(int pos_val);
  void setTrigValue(int trig_val);
  void setVerCmdVal(float ver_val);
  void setGripperMode(float mode_val);
};

class GripperMotor{
private:
  uint8_t pin_gripper_open;
  uint8_t pin_gripper_close;
  uint8_t pin_scissors;
  uint8_t pin_ver_extend;
  uint8_t pin_hor_extend;
  uint8_t pin_cutter;

  unsigned long timestamp;
  int state;
  bool gripper_mode;
  int original_pos;
  int trig;
  float ver_cmd_val;

  // uint32_t servospeed = 0x000007D0;// speed is 200r/min
  uint32_t servospeed = 0x000061A8;// speed is 2500r/min
  // uint32_t servospeed = 0x00007530;// speed is 3000r/min

  int ServoGripper_closed = 2100;// the maximunm value is 2100
  uint8_t pin_ServoGripper = 2 ;

  uint32_t first_position = 0xFFE60000;// set the stroke is -26cm,command is -26 rounds
  // uint32_t offset_position = 0.1* 0x10000;
  // uint32_t second_position = first_position - 0x00030000 - offset_position;

  uint32_t target_position;
  uint32_t Servoposition;

public:
  int seqRet;

  GripperMotor(uint8_t Scissors, uint8_t RotationalValve, uint8_t Extend_cylinder, uint8_t hor_cutter);

  void init();
  void gripper(int action);
  int seq(bool trigger_flag=false);
  void gripperReset();

  // Getters for internals of gripper
  int getSeqTimestamp();
  int getGripperState();

  // Setters for internals of gripper
  void setGripperState(int new_state);
};
