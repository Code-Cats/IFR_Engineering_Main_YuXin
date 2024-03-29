#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

#define LF 0
#define RF 1
#define LB 2
#define RB 3

/************标定输出**************/
#define LIFT_CALI_OUTPUT_MAX 1700//1700	//2018.4.12
////////////////////////////////////

/************行程限制**************/
#define LIFT_DISTANCE_FALL 30
#define LIFT_DISTANCE_ISLAND 1170



#define LIFT_POSITION_PID_P	50	//405升降电机PID
#define LIFT_POSITION_PID_I 0
#define LIFT_POSITION_PID_D	0.015f //	0.15f
#define LIFT_POSITION_MAXOUT 8500	//位置环输出的最大速度
#define LIFT_POSITION_MAXINPUT 99999999
#define LIFT_POSITION_PID_MER	180 //	20
#define LIFT_POSITION_PID_DEAD	0	// 0
//经过测试，3510最大转速瞬时值8500，一般限制在8000

#define LIFT_SPEED_PID_P	 5
#define LIFT_SPEED_PID_I	 0.05f
#define LIFT_SPEED_PID_D	 0.06f
#define LIFT_SPEED_MAXOUT 8500	//速度环输出的最大电流
#define LIFT_SPEED_MAXINPUT 8400
#define LIFT_SPEED_PID_MER 	6000
#define LIFT_SPEED_PID_DEAD	 4
#define LIFT_SPEED_PID_I_DEAD 	4
#define LIFT_SPEED_PID_I_MAX 	3000/LIFT_SPEED_PID_I


//升降电机位置环PID参数
#define PID_LIFT_POSITION_DEFAULT \
{\
	LIFT_POSITION_PID_P,\
	LIFT_POSITION_PID_I,\
  LIFT_POSITION_PID_D,\
	LIFT_POSITION_PID_MER,\
	-LIFT_POSITION_MAXINPUT,\
	LIFT_POSITION_MAXINPUT,\
	-LIFT_POSITION_MAXOUT,\
	LIFT_POSITION_MAXOUT,\
	0,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	0,\
}\

//升降电机速度环PID参数
#define PID_LIFT_SPEED_DEFAULT \
{\
	LIFT_SPEED_PID_P,\
	LIFT_SPEED_PID_I,\
  LIFT_SPEED_PID_D,\
	LIFT_SPEED_PID_MER,\
	-LIFT_SPEED_MAXINPUT,\
	LIFT_SPEED_MAXINPUT,\
	-LIFT_SPEED_MAXOUT,\
	LIFT_SPEED_MAXOUT,\
	LIFT_SPEED_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	LIFT_SPEED_PID_DEAD,\
}\


typedef struct
{
	 int32_t lf_lift_fdbV; //201 
	 int32_t rf_lift_fdbV; //202
	 int32_t lb_lift_fdbV; //203
	 int32_t rb_lift_fdbV; //204	  
	
   int32_t lf_lift_tarV;
	 int32_t rf_lift_tarV;
	 int32_t lb_lift_tarV;
	 int32_t rb_lift_tarV;
	  
   int32_t lf_lift_fdbP;
	 int32_t rf_lift_fdbP;
	 int32_t lb_lift_fdbP;
	 int32_t rb_lift_fdbP;
	 
	 int32_t lf_lift_tarP;
	 int32_t rf_lift_tarP;
	 int32_t lb_lift_tarP;
	 int32_t rb_lift_tarP;
	 
	 float lf_lift_output;
	 float rf_lift_output;	
	 float lb_lift_output;
	 float rb_lift_output;
	
}LIFT_DATA;	//CAN2

typedef struct
{
	u16 state;
}LO;

extern float Chassis_GYRO[3];

void Control_Task(void);
void Work_State_Change(void);	//RC,PC对机器状态的控制
void Work_State_Change_Gaming(void);	//战场版控制状态切换
void Work_Execute_LastVersion(void);	//之前版本的执行
void Work_Execute_Gaming(void);	//战场版switch工作执行
void Work_State_Change_BackProtect(void);	//当从某一状态退出时，确保该状态的一切遗留控制都归位
void LED_Indicate(void);	//灯光指示


void RC_Calibration(void);

void Motor_Send(void);
void Lift_Cali_Output_Limit(float cm_out,float * cali_out);

void Entirety_PID(const LIFT_DATA * pliftdata,float send[]);	//整体PID补偿		//2018.2.26DEBUG版

void Lift_Cali_GYRO_Compensate(float cali_send[]);	//基于陀螺仪的底盘标定输出补偿

void Teleconltroller_Data_protect(void);	//遥控器数据自保护 

void Lift_Time_Gauge(u8 *trigger);	//升降时间自测量

void KeyboardRetset(void);	//如果战场发生意外，就进行复位处理
void Data_Init(void);	//内核复位后数据重置

/***********************--标定状态--**********************/
typedef enum
{
	UP_STATE,	//初始向上过程
	WAIT_STATE,	//中间的等待过零过程
	DOWN_STATE,	//向下标定过程
	OK_STATE,	//标定结束
}LiftCaliState_e;


/***********************--工作状态--**********************/
typedef enum
{
    PREPARE_STATE,    //上电后初始化状态 1s钟左右
    CHECK_STATE,			//自检状态 在此阶段能够通过内核重启解决的问题发生时将会自动内核重启
		CALI_STATE,    		//校准状态标定
		LOST_STATE,				//硬件帧率丢失
		ERROR_STATE,			//硬件错误状态，包括但不限于检测出传感器失效，数据失常等重启无法解决的问题
    NORMAL_STATE,			//正常输入状态	即底盘跟随云台模式
    STOP_STATE,       //停止运动状态(由遥控器控制的保护状态)
		ASCEND_STATE,			//登岛状态
		DESCEND_STATE,		//下岛状态
		TAKEBULLET_STATE,	//取弹状态
		SEMI_ASCEND_STATE,	//半自动、手动上岛
		SEMI_DESCEND_STATE,	//半自动、手动下岛
		PROTECT_STATE,		//保护状态，在程序循环时执行，当检测到程序发生异常或者传感器异常时进入该模式
}WorkState_e;

extern WorkState_e workState;
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);

#endif
