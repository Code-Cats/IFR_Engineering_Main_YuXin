#include "control.h"
#include "semi_automatic_landing.h"

WorkState_e workState=PREPARE_STATE;


extern LIFT_DATA lift_Data;

extern RC_Ctl_t RC_Ctl;
extern LIFT_POSITION_ENCODER lift_position_encoder[4];
extern GYRO_DATA Gyro_Data;

extern CHASSIS_DATA chassis_Data;
extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];
extern ViceControlDataTypeDef ViceControlData;
extern SensorDataTypeDef SensorData;

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
	
	KeyboardRetset();
	
	if(time_1ms_count%50==0)
	{
//		Debug_Send_OSC();	//待完善发数逻辑
	}
	
	if(time_1ms_count%1==0)	//1000hz
	{
		for(int keyid=0;keyid<KEY_NUMS;keyid++)	//放在定时器里
		{
			ButtonStatu_Verdict(&KeyBoardData[keyid]);	//键位信息高级处理
		}
	}
	
//Take_Bullet_Task();
////////////	Vw_tem=Chassis_Attitude_Correct(Chassis_GYRO[2],Gyro_Data.angvel[2]+2);	//暂时还没加陀螺仪
////////////  Chassis_Vw+=Vw_tem;
	Work_State_Change_Gaming();	//战场版
	//
	Work_State_Change_BackProtect();
	//
	Work_Execute_Gaming();	//战场版
	//
	
	LED_Indicate();
	
	Chassis_Attitude_Angle_Convert();
	
	Motor_Send();
	
	if(time_1ms_count%2==0)
	{
		ViceBoard_SendDataRefresh();
		ViceBoard_SendDataRun();
	}
}


extern TakeBulletState_e TakeBulletState;	//(自动)取弹标志位
extern AscendState_e AscendState;
extern DescendState_e DescendState;
/*************************************
RC或PC对机器状态的切换
*************************************/
void Work_State_Change(void)
{
	static u8 Switch_Right_Last=0;
	static WorkState_e State_Record=CHECK_STATE;	
	
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
				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
				SetWorkState(DESCEND_STATE);
//				SetWorkState(TAKEBULLET_STATE);
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
		case TAKEBULLET_STATE:	//取弹模式
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	//左中
			{
				TakeBulletState=BULLET_ACQUIRE;	//(自动)取弹标志重置
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case SEMI_ASCEND_STATE:	//半自动、手动上岛
		{

			break;
		}
		case SEMI_DESCEND_STATE:	//半自动、手动下岛
		{

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
	State_Record=GetWorkState();
}

void Work_State_Change_Gaming(void)	//战场版控制状态切换
{	//战场版
	static u8 Switch_Right_Last=0;
	static u8 Switch_Left_Last=0;
	if(GetWorkState()!=CHECK_STATE&&GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE&&GetWorkState()!=LOST_STATE&&GetWorkState()!=ERROR_STATE&&GetWorkState()!=PROTECT_STATE)	//这三种初始状态+3种保护状态不进状态切换（不受控）
	{
		switch(RC_Ctl.rc.switch_left)
		{
			case RC_SWITCH_UP:
			{
				switch(RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:	//UP-UP	一般状态
					{
						SetWorkState(NORMAL_STATE);
						break;
					}
					case RC_SWITCH_MIDDLE:
					{
						
						break;
					}
					case RC_SWITCH_DOWN:
					{
						break;
					}
				}
				break;
			}
			case RC_SWITCH_MIDDLE:
			{
				switch(RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:
					{
						
						break;
					}
					case RC_SWITCH_MIDDLE:	//MID-MID	全自动下岛
					{
						if(Switch_Left_Last!=RC_SWITCH_MIDDLE)	//保护一开始未把拨杆置到最上
						SetWorkState(DESCEND_STATE);
						break;
					}
					case RC_SWITCH_DOWN:	//MID-DOWN	手动下岛
					{
						SetWorkState(SEMI_DESCEND_STATE);
						break;
					}
				}
				break;
			}
			case RC_SWITCH_DOWN:
			{
				switch(RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:	//DOWN-UP	全自动上岛
					{
						if(Switch_Left_Last!=RC_SWITCH_DOWN)	//保护一开始未把拨杆置到最上
						SetWorkState(ASCEND_STATE);
						break;
					}
					case RC_SWITCH_MIDDLE:	//DOWN-MID	半自动上岛
					{
						SetWorkState(SEMI_ASCEND_STATE);
						break;
					}
					case RC_SWITCH_DOWN:	//DOWN-DOWN	取弹
					{
						SetWorkState(TAKEBULLET_STATE);
						break;
					}
				}
				break;
			}
		}
	}
	
	switch (GetWorkState())	//2018.5.9
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
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{

			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{

			break;
		}
		case TAKEBULLET_STATE:	//取弹模式
		{

			break;
		}
		case SEMI_ASCEND_STATE:	//半自动、手动上岛
		{

			break;
		}
		case SEMI_DESCEND_STATE:	//半自动、手动下岛
		{

			break;
		}
		case ERROR_STATE:	//错误模式
		{
			if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)
			{
				SetWorkState(NORMAL_STATE);
			}
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
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case PROTECT_STATE:	//自我保护模式
		{
			static u32 time_count=0;
			time_count++;
			if(Error_Check.statu[LOST_DBUS]==0&&abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
			{
				SetWorkState(NORMAL_STATE);
				time_count=0;
			}
			
			if(Error_Check.statu[LOST_DBUS]==0&&time_count>8000)	//有反馈认为无法恢复
			{
				NVIC_SystemReset();
			}
			break;
		}
	}

	Switch_Right_Last=RC_Ctl.rc.switch_right;
	Switch_Left_Last=RC_Ctl.rc.switch_left;
}

void Work_Execute_LastVersion(void)	//之前版本的执行
{
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
			TakeBullet_Control_Center();	//暂时把让位给登岛
			if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)
			{
				semi_auto_landing_center();
			}
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
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
		case TAKEBULLET_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			TakeBullet_Control_Center();	//取弹控制中心
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case SEMI_ASCEND_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)
			{
				semi_auto_landing_center();
			}
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case SEMI_DESCEND_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
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
}

extern u8 Replenish_Bullet_Statu;	//补弹状态位

u8 t_trailer_sensor_data_simu=0;	//传感器数据仿真
extern u8 SetCheck_TakeBullet_TakeBack_statu;	//切出取弹保护执行标志位	//加在这里是让半自动下岛能有下降的前提条件	//这个statu为0都是在一次执行完成后才有
void Work_Execute_Gaming(void)	//战场版switch工作执行
{
	static WorkState_e State_Record=CHECK_STATE;
	switch (GetWorkState())	//2018.5.9
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
//			ViceControlData.valve[VALVE_ISLAND]=0;//放在跳变
			Teleconltroller_Data_protect();	//遥控器数据保护
			TakeBullet_Control_Center();	//加上这个是因为关于舵机、气缸的假想反馈计算在这里面，切出取弹归位保护需要它，其内部已经做了仅在TAKEBULLET下做逻辑处理
			
			Replenish_Bullet_Task(KeyBoardData[KEY_R].value);	//给补给站补弹
			Trailer_Task(SensorData.Infrare[5]);	//拖车
			
			if(ViceControlData.valve[VALVE_ISLAND]==0&&Replenish_Bullet_Statu==0)
			{
				AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);
			}
			
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case ASCEND_STATE:	//自动上岛模式
		{
			if(State_Record!=ASCEND_STATE)	//2018.5.9
			{
				AscendState=Island_State_Recognize();	//自动辨识当前状态
			}
			
			Teleconltroller_Data_protect();	//遥控器数据保护
			Ascend_Control_Center();
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case DESCEND_STATE:	//自动下岛模式
		{
			TakeBullet_Control_Center();	//加上这个是因为关于舵机、气缸的假想反馈计算在这里面，切出取弹归位保护需要它，其内部已经做了仅在TAKEBULLET下做逻辑处理
			if(State_Record!=DESCEND_STATE)
			{
				DescendState=OutIsland_State_Recognize();
			}
			
			Teleconltroller_Data_protect();	//遥控器数据保护
			Descend_Control_Center();
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case TAKEBULLET_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			TakeBullet_Control_Center();	//取弹控制中心
			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case SEMI_ASCEND_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护

			semi_auto_landing_center();

			Remote_Task();	//执行移动
			Lift_Task();	//开启升降
			BulletLift_Task();
			break;
		}
		case SEMI_DESCEND_STATE:
		{
			Teleconltroller_Data_protect();	//遥控器数据保护
			
			TakeBullet_Control_Center();	//加上这个是因为关于舵机、气缸的假想反馈计算在这里面，切出取弹归位保护需要它，其内部已经做了仅在TAKEBULLET下做逻辑处理
			if(SetCheck_TakeBullet_TakeBack_statu==0)	//只有当取弹状态完全退出时，statu才会被置0
			{
				semi_auto_outlanding_center();
			}
			
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
	Image_Cut_Task();
	State_Record=GetWorkState();
}



extern u8 descend_valve_prepare_state;	//自动下岛电磁阀到位保护
extern u32 descend_valve_prepare_state_count;
//extern u8 SetCheck_TakeBullet_TakeBack_statu;	//切出取弹保护执行标志位	//放在前面extern
void Work_State_Change_BackProtect(void)	//当从某一状态退出时，确保该状态的一切遗留控制都归位
{
	static WorkState_e State_Record=CHECK_STATE;
	
	if(State_Record!=DESCEND_STATE&&GetWorkState()==DESCEND_STATE)
	{
		descend_valve_prepare_state=0;
		descend_valve_prepare_state_count=0;
	}
	
	if(State_Record!=NORMAL_STATE&&GetWorkState()==NORMAL_STATE)
	{
		ViceControlData.valve[VALVE_ISLAND]=0;	//取弹收回导轮
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//退出取弹模式
	{
		SetCheck_TakeBullet_TakeBack_statu=1;	//刷新处
	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		SetCheck_GripBulletLift(1);
		ViceControlData.valve[VALVE_ISLAND]=0;	//取弹收回导轮
	}
	SetCheck_TakeBullet_TakeBack();	//执行处
	State_Record=GetWorkState();
}

extern s16 t_error_i_record;
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
			case TAKEBULLET_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case SEMI_ASCEND_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case SEMI_DESCEND_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ERROR_STATE:	//错误模式
			{
				if(t_error_i_record==LOST_BULLETLIFT1)	//前取弹升降
				{
					LED_Blink_Set(3,10);
				}
				else if(t_error_i_record==LOST_BULLETLIFT2)	//后取弹升降
				{
					LED_Blink_Set(4,10);
				}
				else if(t_error_i_record==LOST_CM1||t_error_i_record==LOST_CM2||t_error_i_record==LOST_CM3||t_error_i_record==LOST_CM4)	//底盘电机
				{
					LED_Blink_Set(2,10);
				}
				else if(t_error_i_record==LOST_LIFT1||t_error_i_record==LOST_LIFT2||t_error_i_record==LOST_LIFT3||t_error_i_record==LOST_LIFT4)	//升降
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

/////////////////////////////////原来放lift task的地方

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
//			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
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
		case TAKEBULLET_STATE:
		{
			CAN2_BulletLift_SendMsg((s16)bulletlift_Motor_Data[BULLETLIFT_FRONTID].output,(s16)bulletlift_Motor_Data[BULLETLIFT_BACKID].output);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case SEMI_ASCEND_STATE:
		{
			CAN2_BulletLift_SendMsg((s16)bulletlift_Motor_Data[BULLETLIFT_FRONTID].output,(s16)bulletlift_Motor_Data[BULLETLIFT_BACKID].output);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case SEMI_DESCEND_STATE:
		{
			CAN2_BulletLift_SendMsg((s16)bulletlift_Motor_Data[BULLETLIFT_FRONTID].output,(s16)bulletlift_Motor_Data[BULLETLIFT_BACKID].output);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
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
	if(cm_out>LIFT_CALI_OUTPUT_MAX+3300)
	{
		*cali_out=LIFT_CALI_OUTPUT_MAX+3300;
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


void KeyboardRetset(void)	//如果战场发生意外，就进行复位处理
{
	if(KeyBoardData[KEY_CTRL].value==1&&KeyBoardData[KEY_SHIFT].value==1&&KeyBoardData[KEY_Z].value==0&&KeyBoardData[KEY_X].value==0&&KeyBoardData[KEY_C].value==1&&KeyBoardData[KEY_V].value==1)	//后面的是防止初始化时全部为0
	{
		time_1ms_count=0;
//		RC_Ctl={1024,1024,1024,1024,3,3};
		NVIC_SystemReset();
	}
}

void Data_Init(void)	//内核复位后数据重置
{
	RC_Ctl.rc.ch0=1024;
	RC_Ctl.rc.ch1=1024;
	RC_Ctl.rc.ch2=1024;
	RC_Ctl.rc.ch3=1024;
	RC_Ctl.rc.switch_left=3;
	RC_Ctl.rc.switch_right=3;
	time_1ms_count=0;
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


