#include "viceboard_analysis.h"

SensorDataTypeDef SensorData={0};
ViceBoardSendTypeDef SendData=MAINBOARD_SENDDATA_DEFAULT;

//副板给主板串口发送函数	//2ms执行一次，10ms更新一次结果，14400的波特率，一个字节最多传输11位，11/14400=0.76ms
void MainBoard_SendData(void)
{
	if(USART_GetFlagStatus(USART3,USART_FLAG_TC)== SET)	//如果上一帧发送完成
	{
		if(SendData.statu==1)
		{
			SendData.data[0]=0x5A;	//防止帧头帧尾被破坏
			SendData.data[4]=0xA5;	//防止帧头帧尾被破坏
			USART_SendData(USART3,SendData.data[SendData.count]);
			SendData.count++;
			if(SendData.count>4)
			{
				SendData.statu=0;
				SendData.count=0;
			}
		}
	}
	
}


ReceiveDataTypeDef ReceiveData={0};
u16 t_vice_count=0;
void Data_Receive(u8 data)	//从主板传过来的数据解析（主副板通用）
{
	t_vice_count++;
	if(data==0x5A&&ReceiveData.headOK_state==0)
	{
		ReceiveData.valid_state=0;	//数据接受期间不进行数据解析
		ReceiveData.headOK_state=1;	
		ReceiveData.count=0;	//重置count
	}
	
	if(ReceiveData.headOK_state==1)	//帧头已找到
	{
		ReceiveData.databuffer[ReceiveData.count]=data;
		ReceiveData.count++;
		if((data==0xA5&&ReceiveData.count!=5)||(ReceiveData.count>5))	//失效
		{
			ReceiveData.valid_state=0;
			ReceiveData.headOK_state=0;
			ReceiveData.count=0;	//重置count
		}
		else if(data==0xA5&&ReceiveData.count==5)
		{
			ReceiveData.valid_state=1;
			ReceiveData.headOK_state=0;
			ReceiveData.count=0;	//重置count
		}
	}
	
	//////////////////////////////这里放数据解析函数-->解析为真实数据
	SensorData_Deal(ReceiveData.databuffer);
}

void SensorData_Deal(u8 *pData)
{
	for(int i=0;i<4;i++)
	{
		SensorData.Limit[i]=*(pData+1)>>(7-i)&0x01;
	}
	
	for(int i=0;i<4;i++)
	{
		SensorData.Infrare[i]=*(pData+1)>>(3-i)&0x01;
	}
}

