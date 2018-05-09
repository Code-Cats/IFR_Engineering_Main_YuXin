#ifndef __LIFT_H_
#define __LIFT_H_

#include "bsp.h"

void Lift_Task(void);
u8 Lift_Cali(void);	//新版升降标定
void Lift_Calibration(void);	//旧版升降电机上电标定


#endif
