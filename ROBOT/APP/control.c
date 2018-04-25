#include "control.h"

WorkState_e workState=PREPARE_STATE;


LIFT_DATA lift_Data={0};


PID_GENERAL PID_Lift_Position[4]={PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT};
PID_GENERAL PID_Lift_Speed[4]={PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT};



extern RC_Ctl_t RC_Ctl;
extern LIFT_POSITION_ENCODER lift_position_encoder[4];
extern GYRO_DATA Gyro_Data;

extern CHASSIS_DATA chassis_Data;
extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];


extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern s16 Chassis_Vw;
extern u8 cali_state_Entirety_PID;

s16 remote_tem=0;
s16 lift_tem=0;
s16 LIFT_tarP=0;

u8 t_lift_time_start=0;
///////////////////////////////////////remoteData
s16 Vw_tem=0;
///////////////////////////////////////

u32 time_1ms_count=0;
void Control_Task(void)	//2ms
{
	time_1ms_count++;

	Lift_Time_Gauge(&t_lift_time_start);
	
	Check_Task();
	
	if(time_1ms_count%50==0)
	{
		Debug_Send_OSC();
	}
Take_Bullet_Task();
	Vw_tem=Chassis_Attitude_Correct(Chassis_GYRO[2],Gyro_Data.angvel[2]+2);
  Chassis_Vw+=Vw_tem;
	Work_State_Change();
	switch (GetWorkState())	//2018.3.15
	{
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 //此时外设刚刚开启，需等待一段时间全局自检未检测到异常（2-3个自检触发周期以上），又因为时间计算起点为定时器启动点，故无需进行时间差记录
			if(time_1ms_count>300)	//若从LOST状态回到CHECK模式，则执行计数清零操作
			{	//若能执行到这里说明LOSTCHECK通过，进行数值检测
				RC_Calibration();	//self check
				if(1)	//selfcheck标志
				{
					SetWorkState(PREPARE_STATE);	//此步意味自检通过，一切硬件模块正常
					//数据初始化↓
				}
			}
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	//等待车身状态稳定，并设置初值
			SetWorkState(CALI_STATE);
			break;
		}
		case CALI_STATE:	//标定模式
		{
			if(Lift_Cali()==1&&BulletLift_Cali()==1)
			{
				SetWorkState(NORMAL_STATE);
			}
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			Take_Bullet_Task();
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			Ascend_Control_Center();
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			Descend_Control_Center();
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			break;
		}
		case LOST_STATE:	//错误模式
		{
			break;
		}
		case STOP_STATE:	//停止状态
		{
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			break;
		}
	}
	
	LED_Indicate();
	
	Chassis_Attitude_Angle_Convert();
	
	Motor_Send();
}



extern AscendState_e AscendState;
extern DescendState_e DescendState;
/*************************************
RC或PC对机器状态的切换
*************************************/
void Work_State_Change(void)
{
	static u8 Switch_Right_Last=0;
	static WorkState_e State_Record=CHECK_STATE;	
	State_Record=GetWorkState();
	switch (GetWorkState())	//2018.3.15
	{
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 
			
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	
			
			break;
		}
		case CALI_STATE:	//标定模式
		{
			
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	//左中
			{
				SetWorkState(STOP_STATE);
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
//				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
//				SetWorkState(DESCEND_STATE);
			}
			
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	//左中
			{
				AscendState=FULLRISE_GO1;	//重置防止下一次异常
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	
			{
				DescendState=FULLFALL_DOWN1;	//重置防止下一次异常
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			break;
		}
		case LOST_STATE:	//错误模式
		{
			SetWorkState(CHECK_STATE);
			time_1ms_count=0;	//进入初始状态重新自检
			break;
		}
		case STOP_STATE:	//停止状态
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP||RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			if(Error_Check.statu[LOST_DBUS]==0||abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
			{
				SetWorkState(STOP_STATE);
			}
			break;
		}
	}
	Switch_Right_Last=RC_Ctl.rc.switch_right;
}



extern s16 t_error_record;
void LED_Indicate(void)
{
	if(time_1ms_count%BLINK_CYCLE==0)
	{
		switch (GetWorkState())	//2018.3.15
		{
			case CHECK_STATE:	//自检模式
			{	//板载外设初始化后便进入自检模式 
				LED_Blink_Set(10,10);
				break;
			}
			case PREPARE_STATE:	//预备模式
			{	
				LED_Blink_Set(9,9);
				break;
			}
			case CALI_STATE:	//标定模式	红开绿闪
			{
				LED_Blink_Set(9,9);
				break;
			}
			case NORMAL_STATE:	//正常操作模式	红关绿闪
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ASCEND_STATE:	//自动上岛模式	绿闪
			{
				LED_Blink_Set(1,0);
				break;
			}
			case DESCEND_STATE:	//自动下岛模式	绿闪
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ERROR_STATE:	//错误模式
			{
				if(t_error_record==LOST_BULLETLIFT1)	//前取弹升降
				{
					LED_Blink_Set(3,10);
				}
				else if(t_error_record==LOST_BULLETLIFT2)	//后取弹升降
				{
					LED_Blink_Set(4,10);
				}
				else if(t_error_record==LOST_CM1||t_error_record==LOST_CM2||t_error_record==LOST_CM3||t_error_record==LOST_CM4)	//底盘电机
				{
					LED_Blink_Set(2,10);
				}
				else if(t_error_record==LOST_LIFT1||t_error_record==LOST_LIFT2||t_error_record==LOST_LIFT3||t_error_record==LOST_LIFT4)	//升降
				{
					LED_Blink_Set(1,10);
				}
				else
				{
					LED_Blink_Set(6,10);
				}
				
				break;
			}
			case LOST_STATE:	//错误模式
			{
				LED_Blink_Set(1,1);
				break;
			}
			case STOP_STATE:	//停止状态	红闪
			{
				if(time_1ms_count%BLINK_INTERVAL==0)
				{
					LED_Blink_Set(0,10);
				}
				else if((time_1ms_count+BLINK_INTERVAL/2)%BLINK_INTERVAL==0)
				{
					LED_Blink_Set(10,0);
				}
				
				break;
			}
			case PROTECT_STATE:	//自我保护模式	双闪
			{
				LED_Blink_Set(1,1);
				break;
			}
		}
LED_Blink_Run();
	}
}


s16 lift_k_tem=0;
#define LIFT_K 0.8
void Lift_Task(void)
{
	///////////////////////////////////////////////////////////////////
//	lift_Data.lf_lift_tarP=LIFT_tarP;
//	lift_Data.rf_lift_tarP=LIFT_tarP;
//	lift_Data.lb_lift_tarP=LIFT_tarP;
//	lift_Data.rb_lift_tarP=LIFT_tarP;
	if(GetWorkState()==NORMAL_STATE)
	{
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	//左上
		{
			if(time_1ms_count%10==0)
			{
				switch (RC_Ctl.rc.switch_right)
				{
					case 1:
					{
						lift_Data.lf_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rf_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.lb_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rb_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						break;
					}
					case 2:
					{
						lift_Data.lf_lift_tarP+=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rf_lift_tarP+=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
							
						lift_Data.lb_lift_tarP-=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rb_lift_tarP-=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						break;
					}
					case 3:
					{
						
					}
				}
			}
			
		}
	}
	
	
	if(GetWorkState()!=CALI_STATE&&GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CHECK_STATE)	//标定状态下不限制行程
	{
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<FALL?FALL:lift_Data.lf_lift_tarP;	//限制行程
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>ISLAND?ISLAND:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<FALL?FALL:lift_Data.rf_lift_tarP;	//限制行程
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>ISLAND?ISLAND:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<FALL?FALL:lift_Data.lb_lift_tarP;	//限制行程
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>ISLAND?ISLAND:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<FALL?FALL:lift_Data.rb_lift_tarP;	//限制行程
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>ISLAND?ISLAND:lift_Data.rb_lift_tarP;
	}
	
	lift_Data.lf_lift_tarV=(int32_t)PID_General(lift_Data.lf_lift_tarP,lift_Data.lf_lift_fdbP,&PID_Lift_Position[LF]);	//位置环PID计算
	lift_Data.rf_lift_tarV=(int32_t)PID_General(lift_Data.rf_lift_tarP,lift_Data.rf_lift_fdbP,&PID_Lift_Position[RF]);
	lift_Data.lb_lift_tarV=(int32_t)PID_General((lift_Data.lb_lift_tarP),lift_Data.lb_lift_fdbP,&PID_Lift_Position[LB]);	//+13该补偿下向右正常
	lift_Data.rb_lift_tarV=(int32_t)PID_General((lift_Data.rb_lift_tarP),lift_Data.rb_lift_fdbP,&PID_Lift_Position[RB]);	//-5
	
	lift_Data.lf_lift_output=PID_General(lift_Data.lf_lift_tarV,lift_Data.lf_lift_fdbV,&PID_Lift_Speed[LF]);	//速度环PID计算
	lift_Data.rf_lift_output=PID_General(lift_Data.rf_lift_tarV,lift_Data.rf_lift_fdbV,&PID_Lift_Speed[RF]);
	lift_Data.lb_lift_output=PID_General(lift_Data.lb_lift_tarV,lift_Data.lb_lift_fdbV,&PID_Lift_Speed[LB]);
	lift_Data.rb_lift_output=PID_General(lift_Data.rb_lift_tarV,lift_Data.rb_lift_fdbV,&PID_Lift_Speed[RB]);
	
//	CAN_Lift_SendMsg(lift_Data.lf_lift_output,lift_Data.rf_lift_output,lift_Data.lb_lift_output,lift_Data.rb_lift_output);
	
}


/*****************************************第三代标定程序**********************************************/
LiftCaliState_e liftcaliState=UP_STATE;
u8 Lift_Cali(void)
{
	switch (liftcaliState)
	{
		case UP_STATE:
		{
			PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
			
			lift_Data.lf_lift_tarP=70;
			lift_Data.rf_lift_tarP=70;
			lift_Data.lb_lift_tarP=70;
			lift_Data.rb_lift_tarP=70;

			if(lift_Data.lf_lift_fdbP>30&&lift_Data.rf_lift_fdbP>30&&lift_Data.lb_lift_fdbP>30&&lift_Data.rb_lift_fdbP>30)
			{
				liftcaliState=WAIT_STATE;
				cali_state_Entirety_PID=1;
			}
			break;
		}
		case WAIT_STATE:
		{
			PID_Lift_Speed[LF].k_i=0;
			PID_Lift_Speed[RF].k_i=0;
			PID_Lift_Speed[LB].k_i=0;
			PID_Lift_Speed[RB].k_i=0;
				
			lift_Data.lf_lift_tarP=-10000;
			lift_Data.rf_lift_tarP=-10000;
			lift_Data.lb_lift_tarP=-10000;
			lift_Data.rb_lift_tarP=-10000;
			if((lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<-25)	//即已过速度零点
			{
				liftcaliState=DOWN_STATE;
			}
			break;
		}
		case DOWN_STATE:
		{
			if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<10)	//上一次这一次位置值之差//改为了
			{
				lift_position_encoder[LF].turns=0;
				lift_position_encoder[RF].turns=0;
				lift_position_encoder[LB].turns=0;
				lift_position_encoder[RB].turns=0;
				
				lift_Data.lf_lift_tarP=0;
				lift_Data.rf_lift_tarP=0;
				lift_Data.lb_lift_tarP=0;
				lift_Data.rb_lift_tarP=0;
				
				PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[RF].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[LB].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
				
				cali_state_Entirety_PID=0;	//清零
				liftcaliState=OK_STATE;
			}
			break;
		}
		case OK_STATE:
		{
			return 1;
		}
	}
	return 0;
}



u8 cali_state_Entirety_PID=0;	//整体PID在标定过程中的触发时机标志位
u8 Calibration_state=0;
s16 t_fdbV_sum=0;
u16 t_cali_count=0;
/*****************************************第二代标定程序**********************************************/
void Lift_Calibration(void)	//升降电机上电标定
{
//	u32 Record_Last=0;	//标定程序中上一次角度纪录值
	u8 Calibration_Time_Count=0;	//计时
	
	
	SetWorkState(CALI_STATE);
	PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
	
	lift_Data.lf_lift_tarP=100;
	lift_Data.rf_lift_tarP=100;
	lift_Data.lb_lift_tarP=100;
	lift_Data.rb_lift_tarP=100;
	while(lift_Data.lf_lift_fdbP<40||lift_Data.rf_lift_fdbP<40||lift_Data.lb_lift_fdbP<40||lift_Data.rb_lift_fdbP<40)	//等待上升到一定高度
	{;}

	
	PID_Lift_Speed[LF].k_i=0;
	PID_Lift_Speed[RF].k_i=0;
	PID_Lift_Speed[LB].k_i=0;
	PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
		
	lift_Data.lf_lift_tarP=-10000;
	lift_Data.rf_lift_tarP=-10000;
	lift_Data.lb_lift_tarP=-10000;
	lift_Data.rb_lift_tarP=-10000;
		
	cali_state_Entirety_PID=1;
	
	Calibration_Time_Count=time_1ms_count;	//	记录当前系统时间
		
	while(Calibration_state!=1||lift_position_encoder[LF].turns!=0||lift_position_encoder[RF].turns!=0||lift_position_encoder[LB].turns!=0||lift_position_encoder[RB].turns!=0)
	{
		if((time_1ms_count-Calibration_Time_Count)>1000)	//此延时作用是让升降电机过原点
		{
			
			if(time_1ms_count%500==0)
			{
				t_cali_count++;
				t_fdbV_sum=abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV);
				if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<15)	//上一次这一次位置值之差//改为了
				{
					
					Calibration_state=1;
					lift_position_encoder[LF].turns=0;
					lift_position_encoder[RF].turns=0;
					lift_position_encoder[LB].turns=0;
					lift_position_encoder[RB].turns=0;
					
					lift_Data.lf_lift_tarP=0;
					lift_Data.rf_lift_tarP=0;
					lift_Data.lb_lift_tarP=0;
					lift_Data.rb_lift_tarP=0;
					
					PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[RF].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[LB].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
					
					cali_state_Entirety_PID=0;	//清零
					SetWorkState(NORMAL_STATE);	//3.16暂时加
				}
				//Record_Last= lift_position_encoder[LF].calc+lift_position_encoder[RF].calc+lift_position_encoder[LB].calc+lift_position_encoder[RB].calc;
			}

		}
		Calibration_Time_Count=0;
	}
}

float lift_calisend[4]={0};
float bulletlift_calisend[2]={0};

void Motor_Send(void)
{
	switch (GetWorkState())	//2018.3.15
	{	
		case CHECK_STATE:	//自检模式
		{	//板载外设初始化后便进入自检模式 //此时外设刚刚开启，需等待一段时间全局自检未检测到异常（2-3个自检触发周期以上），又因为时间计算起点为定时器启动点，故无需进行时间差记录
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		case PREPARE_STATE:	//预备模式
		{	//等待车身状态稳定，并设置初值
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		case CALI_STATE:	//标定模式
		{
			Lift_Cali_Output_Limit(lift_Data.lf_lift_output,&lift_calisend[LF]);
			Lift_Cali_Output_Limit(lift_Data.rf_lift_output,&lift_calisend[RF]);
			Lift_Cali_Output_Limit(lift_Data.lb_lift_output,&lift_calisend[LB]);
			Lift_Cali_Output_Limit(lift_Data.rb_lift_output,&lift_calisend[RB]);
//		Entirety_PID(&lift_Data,cali_send);  	//整体PID补偿
//			Lift_Cali_GYRO_Compensate(cali_send);	//陀螺仪补偿.存在问题3.14
			
			BulletLift_Cali_Output_Limit(bulletlift_Motor_Data[BULLETLIFT_FRONTID].output,&bulletlift_calisend[BULLETLIFT_FRONTID]);
			BulletLift_Cali_Output_Limit(bulletlift_Motor_Data[BULLETLIFT_BACKID].output,&bulletlift_calisend[BULLETLIFT_BACKID]);
			

			CAN2_Chassis_SendMsg(0,0,0,0);
//		CAN1_Lift_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg((s16)lift_calisend[LF],(s16)lift_calisend[RF],(s16)lift_calisend[LB],(s16)lift_calisend[RB]);
//			CAN2_BulletLift_SendMsg(0,0);
			CAN2_BulletLift_SendMsg(bulletlift_calisend[BULLETLIFT_FRONTID],bulletlift_calisend[BULLETLIFT_BACKID]);
			break;
		}
		case NORMAL_STATE:	//正常操作模式
		{
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN_Chassis_SendMsg(0,0,0,0);
//    CAN_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_BulletLift_SendMsg((s16)bulletlift_Motor_Data[BULLETLIFT_FRONTID].output,(s16)bulletlift_Motor_Data[BULLETLIFT_BACKID].output);
			break;
		}
		case ERROR_STATE:	//错误模式
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		case STOP_STATE:	//停止状态
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
		default:
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletLift_SendMsg(0,0);
			break;
		}
	}
	
}


void Lift_Cali_Output_Limit(float cm_out,float * cali_out)
{
	if(cm_out>LIFT_CALI_OUTPUT_MAX+3000)
	{
		*cali_out=LIFT_CALI_OUTPUT_MAX+3000;
	}
	else if(cm_out<-LIFT_CALI_OUTPUT_MAX)
	{
		*cali_out=-LIFT_CALI_OUTPUT_MAX;
	}
	else
	{
		*cali_out=cm_out;
	}
}


#define PITCH 0
#define ROLL 1
#define YAW 2
float Chassis_GYRO[3]={0};	//pitch roll yaw
/*************************************************
功能：经数据融合给出底盘姿态供其他模块调用
数据单位：度 Gyro_Data.angle
云台陀螺仪数据正方向：pitch:下 roll:  左高右低 yaw：逆时针
电机位置正方向：pitch：下  yaw:逆时针
电机中值：YAW_INIT PITCH_INIT
**************************************************/
void Chassis_Attitude_Angle_Convert(void)	//综合得出底盘姿态
{
//	float deviation_pitch=PITCH_GYRO_INIT-yunMotorData.pitch_fdbP;	//对于底盘来说，云台中值即是底盘在云台坐标系上的位置
//	float deviation_yaw=YAW_INIT-yunMotorData.yaw_fdbP;
//	//对yaw轴进行限制，标准过零（-180——+180）
//	Chassis_GYRO[PITCH]=-Gyro_Data.angle[PITCH]-deviation_pitch*360.0f/8192;	//因为云台电机位置反馈正方向与陀螺仪正方向相反pitch？-2
//	Chassis_GYRO[ROLL]=Gyro_Data.angle[ROLL]+2;	//roll
//	Chassis_GYRO[YAW]=Gyro_Data.angle[YAW]+deviation_yaw*360.0f/8192;	//因为云台电机位置反馈正方向与陀螺仪正方向相同
// 
//	//限制-180_+180
//	Chassis_GYRO[YAW]=Chassis_GYRO[YAW]>180?Chassis_GYRO[YAW]-360:Chassis_GYRO[YAW];
//	Chassis_GYRO[YAW]=Chassis_GYRO[YAW]<-180?Chassis_GYRO[YAW]+360:Chassis_GYRO[YAW];
}



s32 t_entirety_lf=0;
s32 t_entirety_rf=0;
s32 t_entirety_lb=0;
s32 t_entirety_rb=0;
s32 t_out_lf=0;
s32 t_out_rf=0;
s32 t_out_lb=0;
s32 t_out_rb=0;
#define ENTIRETY_LIFT_P 8	//整体PID参数
void Entirety_PID(const LIFT_DATA * pliftdata,float cali_send[4])	//整体PID补偿		//2018.2.26DEBUG版
{
	s32 lift_average=(s32)(pliftdata->lf_lift_fdbP+pliftdata->rf_lift_fdbP+pliftdata->lb_lift_fdbP+pliftdata->rb_lift_fdbP)/4;
	if(cali_state_Entirety_PID==1)
	{
		t_entirety_lf=(lift_average-pliftdata->lf_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_rf=(lift_average-pliftdata->rf_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_lb=(lift_average-pliftdata->lb_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_rb=(lift_average-pliftdata->rb_lift_fdbP)*ENTIRETY_LIFT_P;
		
		cali_send[LF]+=t_entirety_lf;
		cali_send[RF]+=t_entirety_rf;
		cali_send[LB]+=t_entirety_lb;
		cali_send[RB]+=t_entirety_rb;
		
		t_out_lf=cali_send[LF];
		t_out_rf=cali_send[RF];
		t_out_lb=cali_send[LB];
		t_out_rb=cali_send[RB];
	}
}

#define LIFT_GYRO_CALI_K 65
s16 t_cali_gyro_lf=0;
s16 t_cali_gyro_rf=0;
s16 t_cali_gyro_lb=0;
s16 t_cali_gyro_rb=0;
void Lift_Cali_GYRO_Compensate(float cali_send[4])	//基于陀螺仪的底盘标定输出补偿3.13晚存在问题
{	//当chassis_pitch>0时前高后低  需要减前两个LF RF，加后两个LB RB
	//当roll>0时左高右低	需要减左两个LF LB,加右两个RF RB
	if(cali_state_Entirety_PID==1)
	{
		cali_send[LF]+=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		cali_send[RF]+=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		cali_send[LB]+=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		cali_send[RB]+=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		t_cali_gyro_lf=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		t_cali_gyro_rf=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		t_cali_gyro_lb=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		t_cali_gyro_rb=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
	}
	  
}


u16 lift_time_gauge_count=0;
void Lift_Time_Gauge(u8 *trigger)	//升降时间自测量
{
	if(*trigger==1)
	{
		lift_time_gauge_count++;
		if(SetCheck_FrontLift(1)==1||SetCheck_BackLift(1)==1)
		{
			*trigger=0;
			SetCheck_FrontLift(0);
			SetCheck_BackLift(0);
		}
	}
}



void RC_Calibration(void)	//上电检测遥控器接收值并与默认参数比较，判断是否正常，否则软复位
{													//注：必须放在遥控器接收初始化后
	if(abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
	{
		NVIC_SystemReset();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************主动保护状态***********************************************/
RC_Ctl_t RC_DATA_ERROR={0};	//记录错误帧数据
void Teleconltroller_Data_protect(void)	//遥控器数据自保护 
{
	u8 protect_state=0xC0;	//按位表示当前遥控器数据是否正常	//最高2位为保留位，常为1	//364-1024-1684
	protect_state|=(abs(RC_Ctl.rc.ch0-1024)<=662);
	protect_state|=(abs(RC_Ctl.rc.ch1-1024)<=662)<<1;
	protect_state|=(abs(RC_Ctl.rc.ch2-1024)<=662)<<2;
	protect_state|=(abs(RC_Ctl.rc.ch3-1024)<=662)<<3;
	protect_state|=(RC_Ctl.rc.switch_left==1||RC_Ctl.rc.switch_left==2||RC_Ctl.rc.switch_left==3)<<4;
	protect_state|=(RC_Ctl.rc.switch_right==1||RC_Ctl.rc.switch_right==2||RC_Ctl.rc.switch_right==3)<<5;
	
	if(protect_state!=0xFF)	{SetWorkState(PROTECT_STATE); RC_DATA_ERROR=RC_Ctl;}
}




////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

/***********************--工作状态--**********************/
void SetWorkState(WorkState_e state)
{
    workState = state;
}


WorkState_e GetWorkState()
{
	return workState;
}


