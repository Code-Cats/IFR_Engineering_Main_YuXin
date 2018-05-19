#ifndef __UPHILL_AUTO_LIFT_H__
#define __UPHILL_AUTO_LIFT_H__

#include "bsp.h"

#define PITCH 0
#define ROLL 1
#define YAW 2

void AutoChassisAttitude_Lift_V2(float chassis_pitch_raw);	//自动调整姿态	//pitch正方向为前上	//注意放在lift_task前面
void Chassis_Attitude_Angle_Convert(void);	//综合得出底盘姿态

extern float Chassis_GYRO[3];

#endif
