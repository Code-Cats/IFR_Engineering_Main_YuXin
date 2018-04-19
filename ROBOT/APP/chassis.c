#include "chassis.h"

CHASSIS_DATA chassis_Data={0};

PID_GENERAL PID_Chassis_Speed[4]={PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT};
PID_GENERAL PID_Chassis_Follow=PID_CHASSIS_FOLLOW_DEFAULT;

FIRST_ORDER_FILTER Yaw_Follow_Filter=YAW_FOLLOW_FILTER_DEFAULT;

s16 Chassis_Vx=0;
s16 Chassis_Vy=0;
s16 Chassis_Vw=0;

extern RC_Ctl_t RC_Ctl;
extern GYRO_DATA Gyro_Data;


#define K_SPEED 8
s32 t_Vw_PID=0;
void Remote_Task(void)
{
	if(GetWorkState()==NORMAL_STATE)
	{
		Chassis_Vx=RC_Ctl.rc.ch1-1024;
	}
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)	//仅在正常情况下遥控器可驱动电机，(自动)登岛模式下交由程序自动控制
	{
		Chassis_Vw=RC_Ctl.rc.ch2-1024;
	}
	Chassis_Vy=RC_Ctl.rc.ch0-1024;

	chassis_Data.lf_wheel_tarV=(Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rf_wheel_tarV=(-Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.lb_wheel_tarV=(Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rb_wheel_tarV=(-Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	

	
///////////////////////////////////////////////////////////////// 
//	chassis_Data.lf_wheel_tarV=remote_tem;
//	chassis_Data.rf_wheel_tarV=remote_tem;
//	chassis_Data.lb_wheel_tarV=remote_tem;
//	chassis_Data.rb_wheel_tarV=remote_tem;
	
	chassis_Data.lf_wheel_output=PID_General(chassis_Data.lf_wheel_tarV,chassis_Data.lf_wheel_fdbV,&PID_Chassis_Speed[LF]);
	chassis_Data.rf_wheel_output=PID_General(chassis_Data.rf_wheel_tarV,chassis_Data.rf_wheel_fdbV,&PID_Chassis_Speed[RF]);
	chassis_Data.lb_wheel_output=PID_General(chassis_Data.lb_wheel_tarV,chassis_Data.lb_wheel_fdbV,&PID_Chassis_Speed[LB]);
	chassis_Data.rb_wheel_output=PID_General(chassis_Data.rb_wheel_tarV,chassis_Data.rb_wheel_fdbV,&PID_Chassis_Speed[RB]);
	if((GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)&&RC_Ctl.rc.switch_left==RC_SWITCH_UP)
	{
		Extended_Integral_PID(&chassis_Data);
	}
	
//	CAN_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
}


#define CHASSIS_INTEGRAL_PID_KP 3
#define CHASSIS_INTEGRAL_PID_KI 0.01
#define CHASSIS_INTEGRAL_PID_I_SUM_LIM 1000
void Extended_Integral_PID(CHASSIS_DATA* chassis_data)	//扩展型整体PID，适用于任意动作场景	2018.4.19
{
	float tarv_sum=abs(chassis_data->lf_wheel_tarV)+abs(chassis_data->rf_wheel_tarV)+abs(chassis_data->lb_wheel_tarV)+abs(chassis_data->rb_wheel_tarV);
	float fdbv_sum=abs(chassis_data->lf_wheel_fdbV)+abs(chassis_data->rf_wheel_fdbV)+abs(chassis_data->lb_wheel_fdbV)+abs(chassis_data->rb_wheel_fdbV);
	float expect[4]={0};
	float error[4]={0};
	static float inte[4];
	expect[LF]=fdbv_sum*chassis_data->lf_wheel_tarV/tarv_sum;
	expect[RF]=fdbv_sum*chassis_data->rf_wheel_tarV/tarv_sum;
	expect[LB]=fdbv_sum*chassis_data->lb_wheel_tarV/tarv_sum;
	expect[RB]=fdbv_sum*chassis_data->rb_wheel_tarV/tarv_sum;
	error[LF]=expect[LF]-chassis_data->lf_wheel_fdbV;
	error[RF]=expect[RF]-chassis_data->rf_wheel_fdbV;
	error[LB]=expect[LB]-chassis_data->lb_wheel_fdbV;
	error[RB]=expect[RB]-chassis_data->rb_wheel_fdbV;
	
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	
	for(int id=0;id<4;id++)
	{
		inte[id]=inte[id]>CHASSIS_INTEGRAL_PID_I_SUM_LIM?CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
		inte[id]=inte[id]<-CHASSIS_INTEGRAL_PID_I_SUM_LIM?-CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
	}
	
	chassis_data->lf_wheel_output+=(s32)(error[LF]*CHASSIS_INTEGRAL_PID_KP+inte[LF]);
	chassis_data->rf_wheel_output+=(s32)(error[RF]*CHASSIS_INTEGRAL_PID_KP+inte[RF]);
	chassis_data->lb_wheel_output+=(s32)(error[LB]*CHASSIS_INTEGRAL_PID_KP+inte[LB]);
	chassis_data->rb_wheel_output+=(s32)(error[RB]*CHASSIS_INTEGRAL_PID_KP+inte[RB]);
}


