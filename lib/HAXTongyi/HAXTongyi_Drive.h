#ifndef _TONGYI_DRIVE_H
#define _TONGYI_DRIVE_H
#include "sys.h"

void Servo_Enable(u8 axis);
void Servo_Stop(u8 axis);
void Servo_ERRStop(u8 axis);
void SetHomePosition(u8 axis);
void ServoRunParaSet(u8 axis);
void ServoRun(u8 axis);
void ServoHandRunParaSet(u8 axis);
#endif
