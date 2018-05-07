#include "semi_automatic_landing.h"
#include "auto_lift.h"

/***************************************
引用：u8 SetCheck_FrontLift(u8 rise_state);	//前升降轮升起/落下并检查	//0表示FALL，1表示ISLAND
u8 SetCheck_BackLift(u8 rise_state);

		位置示意图
		 上岛方向
			3			2	//定义后
			1			0	//定义前
****************************************/
extern SensorDataTypeDef SensorData;
extern u32 time_1ms_count;

void semi_auto_landing_center(void)
{
	if(SensorData.Limit[2]==1&&SensorData.Limit[3]==1)	//关联后腿的收起
	{
		SetCheck_BackLift(0);
	}
	if(SensorData.Limit[0]==1&&SensorData.Limit[1]==1)	//关联前腿的升起
	{
		SetCheck_FrontLift(0);
	}
}

