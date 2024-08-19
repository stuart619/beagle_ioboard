#ifndef _CAN_SERVO_H
#define _CAN_SERVO_H
#include <Arduino.h>
class HAXTongYiCanServo {
 public:
  void ServoEnable();
  void Servo_Stop();
  void Servo_ERRStop();
  void SetHomePosition();
  void ServoRunParaSet();
  void ServoRun();
  void ServoHandRunParaSet();

 private:
  uint8_t ControlMode;
  uint16_t StatusWord;
  void ReadStatusWord();
  int NodeId = 0;
};
#endif
