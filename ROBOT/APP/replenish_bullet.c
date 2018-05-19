#include "replenish_bullet.h"

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;

extern u8 Trailer_statu;

u8 Replenish_Bullet_Statu=0;
void Replenish_Bullet_Task(u8 key_r_state)
{
	static u8 key_r_state_last;
	static u8 replenish_bullet_statu_last=0;
	if(Trailer_statu==0)	//与拖车互斥
	{
		if(key_r_state_last==0&&key_r_state==1)	//按下R键
		{
			Replenish_Bullet_Statu=!Replenish_Bullet_Statu;
		}
	}
	
	if(Replenish_Bullet_Statu==1)
	{
		SetCheck_GripBulletLift(0);
		ViceControlData.valve[VALVE_BULLET_STORAGE]=1;	//将补弹气缸伸出
//		if(RC_Ctl.rc.ch3-1024>80)
		if(KeyBoardData[KEY_CTRL].value==1)
		{
			ViceControlData.servo[SERVO_BULLET_DOWN]=1;
		}
	}
	else
	{
		SetCheck_GripBulletLift(1);
		ViceControlData.servo[SERVO_BULLET_DOWN]=0;		//舵机执行处反馈处在自动取弹文件，因时间关系未整理分离
		ViceControlData.valve[VALVE_BULLET_STORAGE]=0;	//将补弹气缸伸出
	}
	
	if(replenish_bullet_statu_last==0&&Replenish_Bullet_Statu==1)
	{
		SetCheck_FrontLift(1);
		SetCheck_BackLift(1);
	}
	else if(replenish_bullet_statu_last==1&&Replenish_Bullet_Statu==0)
	{
		SetCheck_FrontLift(0);
		SetCheck_BackLift(0);
	}
	
	replenish_bullet_statu_last=Replenish_Bullet_Statu;
	key_r_state_last=key_r_state;
}


