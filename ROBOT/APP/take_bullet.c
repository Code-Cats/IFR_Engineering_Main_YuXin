#include "take_bullet.h"

//#define VALVE_ISLAND 0		//电磁阀控制位定义
//#define VALVE_BULLET_PROTRACT 1	//前伸
//#define VALVE_BULLET_CLAMP 2	//夹紧

TakeBulletState_e TakeBulletState=BULLET_ACQUIRE;	//(自动)取弹标志位


#define STEER_UP_L_INIT 500//1210	//2500
#define STEER_UP_R_INIT 2500//1950	//500
#define STEER_DOWN_L_INIT 1600//1650	//1000小-内
#define STEER_DOWN_R_INIT 1430//1550	//2180小-外
//2018.5.4
#define STEER_UP_L_REVERSAL 1500//2500	//2500
#define STEER_UP_R_REVERSAL 1500//630	//500
#define STEER_DOWN_L_REVERSAL 1100	//1000
#define STEER_DOWN_R_REVERSAL 1930	//2180
u16 Steer_Send[4]={STEER_UP_L_INIT,STEER_UP_R_INIT,STEER_DOWN_L_INIT,STEER_DOWN_R_INIT};

u8 valve_fdbstate[6]={0};	//记录是否伸出的反馈标志
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={300,1200,700,1000,1000,1000};	//待加入，延时参数
const u32 valve_POORdelay[6]={300,1200,700,1000,1000,1000};	//待加入，延时参数
const u32 servo_GOODdelay[2]={2000,1000};	//延时参数	//第一段为2000是将子弹落下的延时也加进去了，因为舵机翻转和子弹下落必须是连在一体的
const u32 servo_POORdelay[2]={1000,1000};	//延时参数


extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern u32 time_1ms_count;
extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];

u8 auto_takebullet_statu=0;
void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//右拨杆
	
	static u8 valve_last[6]={0};	//记录上一次数值	//保持与工程车兼容性
	static u8 servo_last[2]={0};	//记录上一次数值	//保持与工程车兼容性
	
	static u32 valve_startGOOD_time[6]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 servo_startGOOD_time[2]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 valve_startPOOR_time[6]={0};	//记录逆向触发时间	//保持与工程车兼容性
	static u32 servo_startPOOR_time[2]={0};	//记录逆向触发时间	//保持与工程车兼容性
	
	
	if(GetWorkState()==TAKEBULLET_STATE&&RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	//取弹升降给DOWN-MID，前伸出发-夹紧一套给DOWN-MID-->DOWN-DOWN;舵机旋转给DOWN-MID-->DOWN-UP
	{
		if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
		{
			if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
				auto_takebullet_statu=!auto_takebullet_statu;
				TakeBulletState=BULLET_ACQUIRE;
			}
			else if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
			}
			
			if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//这句话放在bullet_lift.c里
			{
				
			}

		}

		if(auto_takebullet_statu==1)	//自动取弹
		{
			switch(TakeBulletState)	//自动取弹过程
			{
				case BULLET_ACQUIRE:	//前伸、夹紧、抬起动作	称之为获得过程
				{
					if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)	//如果未伸出，就检查升降高度到抓取位置，然后伸出
					{
						SetCheck_GripBulletLift(1);	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
						SetCheck_LiftAll_To_bullet(1);	//取弹时底盘升至固定高度，1为升，0为降
						if(SetCheck_GripBulletLift(1)==1&&SetCheck_LiftAll_To_bullet(1)==1)
						{
							ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;	//前伸函数
						}
					}
					
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//若未夹紧则执行，已夹紧则不执行，达到阶段性运作效果
					{
						if(valve_fdbstate[VALVE_BULLET_PROTRACT]==1&&SetCheck_GripBulletLift(1)==1)	//如果已经前伸，就向下落到抓取高度
						{
							ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
						}
					}
					
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)//如果前伸到位腿升起函数
					{
						if(SetCheck_GripBulletLift(0)==1)	//上升至可旋转高度
						TakeBulletState=BULLET_POUROUT;//直接切换到下一状态
					}
					break;
				}
				case BULLET_POUROUT:	//车身倾斜、舵机旋转	称之为倒弹过程
				{
					ViceControlData.servo[0]=1;
					//略作延时，切换到下一状态	//此处比英雄少一倾斜过程
						if(servo_fdbstate[0]==1)
						TakeBulletState=BULLET_THROWOUT;
					break;
				}
				case BULLET_THROWOUT:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
				{
					ViceControlData.servo[0]=0;
					if(servo_fdbstate[0]==0)	//先让舵机归位的原因是以便让弹药箱能够顺利回位
					{
						if(SetCheck_GripBulletLift(0)==1)//车身回复到抬起（扔）弹药箱高度，扔掉弹药箱并准备下一次平移，工程实际无需此函数
						{
							ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
						//	ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;	//注释以便平移取下一颗弹
						}//如果车身抬起且舵机到位，则松开夹紧，至此一个完整取弹结束
					}
					break;
				}
			}
		}
	}
	else	//GetWorkState()==TAKEBULLET_STATE&&RC_Ctl.rc.switch_left==RC_SWITCH_DOWN的else
	{
		if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
		SetCheck_TakeBullet_TakeBack();
	}



	
//	if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//给登岛腾位置
//	{
//		Steer_Send[UP_L]=STEER_UP_L_REVERSAL;
//		Steer_Send[UP_R]=STEER_UP_R_REVERSAL;
//		Steer_Send[DOWN_L]=STEER_DOWN_L_INIT;
//		Steer_Send[DOWN_R]=STEER_DOWN_R_INIT;
//	}
//	else
//	{
//		Steer_Send[UP_L]=STEER_UP_L_INIT;
//		Steer_Send[UP_R]=STEER_UP_R_INIT;
//		Steer_Send[DOWN_L]=STEER_DOWN_L_REVERSAL;
//		Steer_Send[DOWN_R]=STEER_DOWN_R_REVERSAL;
//	}
	
//	if(auto_takebullet_statu==1)	//1/4自动程序
//	{
//		ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;
//		if(valve_fdbstate[VALVE_BULLET_PROTRACT]==1)
//		{
//			ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
//		}
//	}
//	else
//	{
//		ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
//		if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)
//		{
//			ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
//		}
//	}
	
	if(ViceControlData.servo[0]==1)
	{
		Steer_Send[UP_L]=STEER_UP_L_REVERSAL;
		Steer_Send[UP_R]=STEER_UP_R_REVERSAL;
	}
	else
	{
		Steer_Send[UP_L]=STEER_UP_L_INIT;
		Steer_Send[UP_R]=STEER_UP_R_INIT;
	}
	
	/******************************************************************
以下三个for为反馈假设检测方案
分别为：
1.上升，下降沿的触发时间记录
2.根据触发时间的推演反馈值计算
3.数据迭代
******************************************************************/

		for(int i=0;i<6;i++)	//触发时间块
		{
			if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//伸出触发
			{
				valve_startGOOD_time[i]=time_1ms_count;
			}
			else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//收回触发
			{
				valve_startPOOR_time[i]=time_1ms_count;
			}
			
			if(i<2)
			{
				if(servo_last[i]==0&&ViceControlData.servo[i]==1)
				{
					servo_startGOOD_time[i]=time_1ms_count;
				}
				else if(servo_last[i]==1&&ViceControlData.servo[i]==0)
				{
					servo_startPOOR_time[i]=time_1ms_count;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//反馈计算位
		{
			if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
			{
				valve_fdbstate[i]=1;
			}
			else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
			{
				valve_fdbstate[i]=0;
			}
			
			if(i<2)
			{
				if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
				{
					servo_fdbstate[i]=1;
				}
				else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
				{
					servo_fdbstate[i]=0;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//迭代块
		{
			valve_last[i]=ViceControlData.valve[i];
			if(i<2)	servo_last[i]=ViceControlData.servo[i];
		}
////////////////////////////////////////////////////////////////////////////////////////////块结束标志
	swicth_Last_state=RC_Ctl.rc.switch_right;	//迭代
	
	
	PWM3_1=Steer_Send[UP_L];
	PWM3_2=Steer_Send[UP_R];
	PWM3_3=Steer_Send[DOWN_L];
	PWM3_4=Steer_Send[DOWN_R];
}

u8 SetCheck_TakeBullet_TakeBack(void)	//取弹回位
{
	ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
	ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
	
	if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)
	{
		SetCheck_GripBulletLift(1);
		if(SetCheck_LiftAll_To_bullet(0)==1)
		return 1;
	}
	return 0;
}




//#define LIFT_DISTANCE_FALL 30
#define LIFT_DISTANCE_GRIPBULLET	900	//夹弹药箱时底盘高度900-810	890-807  880-803
#define BULLETLIFT_DISTANCE_GRIPBULLET	330	//夹弹药箱时夹持升降高度	//经过实验300为极限可容许前伸高度
#define BULLETLIFT_DISTANCE_DISGRIPBULLET	1070	//拔起弹药箱夹持机构高度
extern LIFT_DATA lift_Data;

u8 SetCheck_GripBulletLift(u8 grip_state)	//夹持机构升降函数//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
{
	bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET);
	bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET);
	
	return (abs(bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbP+bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP-2*(BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET)))<30);	//这里是仅以前两腿为反馈传回的
}

u8	SetCheck_LiftAll_To_bullet(u8 bullet_state)	//底盘升降函数	//取弹时底盘升至固定高度，1为升，0为降
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET)))<30);	//这里是仅以前两腿为反馈传回的
}
