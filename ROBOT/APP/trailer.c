#include "trailer.h"
/*
拖车任务
*/

#define LIFT_DISTANCE_TRAILER 300

extern LIFT_DATA lift_Data;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern ViceControlDataTypeDef ViceControlData;
extern SensorDataTypeDef SensorData;
extern u8 valve_fdbstate[6];	//记录是否伸出的反馈标志
extern u8 servo_fdbstate[2];

/*
#define VALVE_ISLAND 0		//电磁阀控制位定义
#define VALVE_BULLET_PROTRACT 1	//前伸
#define VALVE_BULLET_CLAMP 2	//夹紧
#define VALVE_BULLET_STORAGE 3	//弹药舱
#define VALVE_TRAILER 5	//拖车
*/

//红外的近距离为0，远距离为1
u8 Trailer_statu=0;	//拖车标志位
void Trailer_Task(u8 sensor_data)	//传入函数为测距传感器数据
{
	static u8 sensor_data_last=0;
	static u8 key_f_last=0;
	if(key_f_last==0&&KeyBoardData[KEY_F].value==1)	//待加入图传自动转向看向拖车
	{
		Trailer_statu=!Trailer_statu;	//翻转
	}
	
	if(Trailer_statu==1)
	{
		ViceControlData.valve[VALVE_TRAILER]=1;
		if(sensor_data_last==1&&sensor_data==0)
		{
			//添加升降函数
			lift_Data.lf_lift_tarP=LIFT_DISTANCE_TRAILER;
			lift_Data.rf_lift_tarP=LIFT_DISTANCE_TRAILER;
			lift_Data.lb_lift_tarP=LIFT_DISTANCE_TRAILER;
			lift_Data.rb_lift_tarP=LIFT_DISTANCE_TRAILER;
		}
	}
	else
	{
		ViceControlData.valve[VALVE_TRAILER]=0;
		lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
		lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
		lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
		lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
	}
	key_f_last=KeyBoardData[KEY_F].value;	//自己记录last而非直接调用keydata中的last可以避免因为刷新频率不同导致变值点检测窗口的丢失
	sensor_data_last=sensor_data;
}


