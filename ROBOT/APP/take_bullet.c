#include "take_bullet.h"

#define STEER_UP_L_INIT 1210	//2500
#define STEER_UP_R_INIT 1850	//500
#define STEER_DOWN_L_INIT 1650	//1000
#define STEER_DOWN_R_INIT 1550	//2180

#define STEER_UP_L_REVERSAL 2500	//2500
#define STEER_UP_R_REVERSAL 500	//500
#define STEER_DOWN_L_REVERSAL 1000	//1000
#define STEER_DOWN_R_REVERSAL 2180	//2180

u16 Steer_Send[4]={STEER_UP_L_INIT,STEER_UP_R_INIT,STEER_DOWN_L_INIT,STEER_DOWN_R_INIT};
extern RC_Ctl_t RC_Ctl;
void Take_Bullet_Task(void)
{
	if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
	{
		Steer_Send[UP_L]=STEER_UP_L_REVERSAL;
		Steer_Send[UP_R]=STEER_UP_R_REVERSAL;
		Steer_Send[DOWN_L]=STEER_DOWN_L_INIT;
		Steer_Send[DOWN_R]=STEER_DOWN_R_INIT;
	}
	else
	{
		Steer_Send[UP_L]=STEER_UP_L_INIT;
		Steer_Send[UP_R]=STEER_UP_R_INIT;
		Steer_Send[DOWN_L]=STEER_DOWN_L_REVERSAL;
		Steer_Send[DOWN_R]=STEER_DOWN_R_REVERSAL;
	}
	PWM3_1=Steer_Send[UP_L];
	PWM3_2=Steer_Send[UP_R];
	PWM3_3=Steer_Send[DOWN_L];
	PWM3_4=Steer_Send[DOWN_R];
}

