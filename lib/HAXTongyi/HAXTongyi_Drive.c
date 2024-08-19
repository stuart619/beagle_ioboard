#include "Tongyi_Drive.h"
#include "can.h"
#include "GlobalParameter.h"
/*********** HAX HMI Controller**************/

void Servo_Enable(u8 axis)
{
	if(SysParameter.ControlMode[axis] != 0x01)
	{
		//��������λ��ģʽ����
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x01;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x030A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 1;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
		//����ģʽ��ȡָ��
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	}else
	{
		if(SysParameter.DriverState[axis] & 0x0040)  //Switch on disable
		{
			//����Ready to switch on
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x06;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		}
		
		if((SysParameter.DriverState[axis] & 0x006F) == 0x0021)  //Ready to switch on
		{
			//����switch on
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x07;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		}
		
		if((SysParameter.DriverState[axis] & 0x006F) == 0x0023)  //Switched on
		{
			//����Operation enabled
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x0F;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		}
		
	}
		
}

void Servo_Stop(u8 axis)
{
	if(SysParameter.DriverState[axis] & 0x0400)
	{
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x0F;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}else
	{
		//����Quick stop
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x02;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}
}


void Servo_ERRStop(u8 axis)
{
	if((SysParameter.DriverState[axis] & 0x006F) == 0x0027)
	{
		//���������ʹ��
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x07;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	}else if((SysParameter.DriverState[axis] & 0x0088))
	{
		//�������״̬
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x80;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}
	
	SysParameter.ActuatorRunCommand[axis] = 16;  //�������״̬����Ϊֹͣ
}


void ServoRunParaSet(u8 axis)
{
	u16 acc;
	u32 position;
	//ȷ���ٶ�ֵ������ȷ
	position = (ROM_Parameter.PointPosition[axis][SysParameter.ActuatorRunCommand[axis]-1] + ROM_Parameter.HomeMotorPosition[axis]);
	if(SysParameter.PointSpeedBack[axis] != ROM_Parameter.PointSpeed[axis][SysParameter.ActuatorRunCommand[axis]-1])
  {
		//����λ��ģʽλ�ú��ٶ�ֵ  PDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = (u8) position;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = (u8)(position>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = (u8)(position>>16);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = (u8)(position>>24);
		
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8) ROM_Parameter.PointSpeed[axis][SysParameter.ActuatorRunCommand[axis]-1];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(ROM_Parameter.PointSpeed[axis][SysParameter.ActuatorRunCommand[axis]-1]>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x050A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
		//���ٶ�
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x81;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	//ȷ��λ��ֵ������ȷ
	} else if(SysParameter.PointPositionBack[axis] != position)
  {
		//����λ��ģʽλ�ú��ٶ�ֵ  PDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = (u8) position;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = (u8)(position>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = (u8)(position>>16);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = (u8)(position>>24);
		
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8) ROM_Parameter.PointSpeed[axis][SysParameter.ActuatorRunCommand[axis]-1];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(ROM_Parameter.PointSpeed[axis][SysParameter.ActuatorRunCommand[axis]-1]>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x050A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
		//���ٶ� SDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x7A;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}else if((SysParameter.SpeedAccBack[axis] * ROM_Parameter.SpeedAcc[SysParameter.ActuatorRunCommand[axis]-1])!= 125000)
	{
		//SDO ����0x6083
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x23;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x83;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;
   
		acc = 125000/ROM_Parameter.SpeedAcc[SysParameter.ActuatorRunCommand[axis]-1];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8)acc;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(acc>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	//�����ٶ� SDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x83;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}else if((SysParameter.SpeedDecBack[axis] * ROM_Parameter.SpeedAcc[SysParameter.ActuatorRunCommand[axis]-1])!= 125000)
	{
		//SDO ����0x6083
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x23;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x84;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;
   
		acc = 125000/ROM_Parameter.SpeedAcc[SysParameter.ActuatorRunCommand[axis]-1];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8)acc;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(acc>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	//�����ٶ� SDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x84;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}
	else
	{
		SysParameter.MotorRunState[axis] |= 0x01;  //���������
		
	}
}

void ServoHandRunParaSet(u8 axis)
{
	u16 acc;
	//ȷ���ٶ�ֵ������ȷ
	if(SysParameter.PointSpeedBack[axis] != SysParameter.MoveSpeedToMotor[axis])
  {
		//����λ��ģʽλ�ú��ٶ�ֵ  PDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = (u8) SysParameter.MovementToMotor[axis];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = (u8)(SysParameter.MovementToMotor[axis]>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = (u8)(SysParameter.MovementToMotor[axis]>>16);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = (u8)(SysParameter.MovementToMotor[axis]>>24);
		
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8) SysParameter.MoveSpeedToMotor[axis];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(SysParameter.MoveSpeedToMotor[axis]>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x050A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
		//���ٶ�
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x81;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	//ȷ��λ��ֵ������ȷ
	} else if(SysParameter.PointPositionBack[axis] != SysParameter.MovementToMotor[axis])
  {
		//����λ��ģʽλ�ú��ٶ�ֵ  PDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = (u8) SysParameter.MovementToMotor[axis];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = (u8)(SysParameter.MovementToMotor[axis]>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = (u8)(SysParameter.MovementToMotor[axis]>>16);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = (u8)(SysParameter.MovementToMotor[axis]>>24);
		
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8) SysParameter.MoveSpeedToMotor[axis];
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(SysParameter.MoveSpeedToMotor[axis]>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x050A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
		//���ٶ� SDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x7A;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	}else if((SysParameter.SpeedAccBack[axis] * 1250)!= 125000)
	{
		//SDO ����0x6083
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x23;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x83;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;
   
		acc = 100;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8)acc;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(acc>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	//�����ٶ� SDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x83;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}
	else if((SysParameter.SpeedDecBack[axis] * 1250)!= 125000)
	{
		//SDO ����0x6083
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x23;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x84;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;
   
		acc = 100;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = (u8)acc;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = (u8)(acc>>8);
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		
	//�����ٶ� SDO
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x40;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x84;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[2] = 0x60;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[3] = 0x00;

		Can_TXData.DataBuff[Can_TXData.InPtr].Data[4] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[5] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[6] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].Data[7] = 0x00;
		Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x060A+axis;
		Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 8;
		Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
	}
	else
	{
		//��������ʹ�ܱ���  ����ִ��
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x3F;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
		SysParameter.MotorRunState[axis] |= 0x01;  //���������
	}
}


void ServoRun(u8 axis)
{
	//��������ʹ�ܱ���  ����ִ��
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[0] = 0x3F;
			Can_TXData.DataBuff[Can_TXData.InPtr].Data[1] = 0x00;
			Can_TXData.DataBuff[Can_TXData.InPtr].id   = 0x020A+axis;
			Can_TXData.DataBuff[Can_TXData.InPtr].Leng = 2;
			Can_TXData.InPtr = (Can_TXData.InPtr+1) % CanBuffLength;
}

void SetHomePosition(u8 axis)
{
	
}


