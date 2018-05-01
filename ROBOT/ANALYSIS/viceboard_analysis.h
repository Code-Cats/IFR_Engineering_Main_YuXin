#ifndef __VICEBOARD_ANALYSIS_H
#define __VICEBOARD_ANALYSIS_H

#include "bsp.h"


void ViceBoard_SendDataRun(void);
void ViceBoard_SendDataRefresh(void);//限制频率放在调用层
void Data_Receive(u8 data);	//从主板传过来的数据解析（主副板通用）
void SensorData_Deal(u8 *pData);

typedef struct
{
	u8 statu;
	u8 data[5];
	u8 count;
}ViceBoardSendTypeDef;


typedef struct
{
	u8 valve[6];
	u8 servo[2];
}ViceControlDataTypeDef;	//控制副板


#define MAINBOARD_SENDDATA_DEFAULT \
{\
	0,\
	{0x5A,0,0,0,0xA5},\
	0,\
}\

typedef struct
{
	u8 headOK_state;
	u8 valid_state;	//数据帧有效标志位
	u8 databuffer[5];
	u8 count;
}ReceiveDataTypeDef;

typedef struct
{
	u8 Infrare[4];
	u8 Limit[4];
}SensorDataTypeDef;

extern ViceBoardSendTypeDef SendData;
extern ReceiveDataTypeDef ReceiveData;


#endif

