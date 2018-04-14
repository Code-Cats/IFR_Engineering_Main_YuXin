#include "usart6_wifi_Debug_OSC.h"
#include "control.h"

extern float tar_attitude;
extern float Attitude_error;
extern s16 t_Vw_correct;
extern s16 Chassis_Vw;
extern WorkState_e workState;

u8 Bufsize=10;//数据字节长度
u8 t=0;//计数用
u8 OSC_Data[10];//存储要发送的数据
s16 tar_att_tem=0;
s16 att_err=0;
void Debug_Send_OSC(void)
{
	tar_att_tem=(s16)tar_attitude;
	att_err=(s16)Attitude_error;
	OSC_Data[0]=tar_att_tem>>8;//Pitch_speed_pid.output>>8;
	OSC_Data[1]=tar_att_tem;//Pitch_speed_pid.output;
	OSC_Data[2]=att_err>>8;//MPU6050.x>>8;
	OSC_Data[3]=att_err;//MPU6050.x;
	OSC_Data[4]=t_Vw_correct>>8;
	OSC_Data[5]=t_Vw_correct;
	OSC_Data[6]=Chassis_Vw>>8;
	OSC_Data[7]=Chassis_Vw;
	OSC_Data[8]=workState;
	OSC_Data[9]=0xCC;
	for(t=0;t<Bufsize;t++)
	{
		USART_SendData(USART6,OSC_Data[t]);
		while(USART_GetFlagStatus(USART6,USART_FLAG_TC)!=SET);//等待发送结束
	}
}


