#include "can2_analysis.h"

LIFT_POSITION_ENCODER chassis_position_encoder[4]={0};

extern CHASSIS_DATA chassis_Data;
extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];

/******************************************
函数名：CAN2_Feedback_Analysis
函数功能：对底盘电机以及云台电机进行数据解析
          得到反馈数据
函数参数：无
函数返回值：无
函数描述：无
*******************************************/
u32 t_cm_count=0;
void CAN2_Feedback_Analysis(CanRxMsg *rx_message)
{		
		CAN_Receive(CAN2, CAN_FIFO0, rx_message);//读取数据	
		switch(rx_message->StdId)
		{
			 case 0x201:
			{
				Speed_Data_deal(&chassis_Data.lf_wheel_fdbV,rx_message);
				Position_Data_deal(&chassis_Data.lf_wheel_fdbP,&chassis_position_encoder[LF],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM1]);
				break;
			}
			case 0x202:
			{
				t_cm_count++;
				Speed_Data_deal(&chassis_Data.rf_wheel_fdbV,rx_message);
				Position_Data_deal(&chassis_Data.rf_wheel_fdbP,&chassis_position_encoder[RF],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM2]);
				break;
			}
			case 0x203:
			{
				Speed_Data_deal(&chassis_Data.lb_wheel_fdbV,rx_message);
				Position_Data_deal(&chassis_Data.lb_wheel_fdbP,&chassis_position_encoder[LB],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM3]);
				break;
			}
			case 0x204:
			{
				Speed_Data_deal(&chassis_Data.rb_wheel_fdbV,rx_message);
				Position_Data_deal(&chassis_Data.rb_wheel_fdbP,&chassis_position_encoder[RB],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM4]);
				break;
			}
			case 0x205:	//shoot 下
			{
			  BulletLift_Feedback_Deal(&bulletlift_Motor_Data[BULLETLIFT_FRONTID],rx_message);
				LostCountFeed(&Error_Check.count[LOST_BULLETLIFT1]);
			  break;
			}
			case 0x206:	//shoot 下
			{
			  BulletLift_Feedback_Deal(&bulletlift_Motor_Data[BULLETLIFT_BACKID],rx_message);
				LostCountFeed(&Error_Check.count[LOST_BULLETLIFT2]);
			  break;
			}
			 default:
			 break;
		}
}


/****************************************************
函数名称：CAN_Chassis_SendMsg
函数功能：将底盘数据解析后发出
函数参数：motor_201*******底盘左前电机转速
          motor_202*******底盘右前电机转速
          motor_203*******底盘左后电机转速
          motor_204*******底盘右后电机转速
函数返回值： 无
描述：将数据存入TxMessage结构体再由CAN_Transmit发送
****************************************************/
void CAN2_Chassis_SendMsg(int motor_201,int motor_202,int motor_203,int motor_204)
{	
		CanTxMsg TxMessage;
	  TxMessage.StdId = 0x200;      //帧ID为传入参数的CAN_ID
    TxMessage.IDE = CAN_ID_STD;    //标准帧
    TxMessage.RTR = CAN_RTR_DATA;  //数据帧
    TxMessage.DLC = 0x08;          //帧长度为8
    
    TxMessage.Data[0] =(unsigned char)((motor_201>>8)&0xff);
    TxMessage.Data[1] = (unsigned char)(motor_201&0xff);
    TxMessage.Data[2] =(unsigned char)((motor_202>>8)&0xff);
    TxMessage.Data[3] = (unsigned char)(motor_202&0xff);
    TxMessage.Data[4] =(unsigned char)((motor_203>>8)&0xff);
    TxMessage.Data[5] = (unsigned char)(motor_203&0xff);
    TxMessage.Data[6] =(unsigned char)((motor_204>>8)&0xff);
    TxMessage.Data[7] = (unsigned char)(motor_204&0xff);
		 
    CAN_Transmit(CAN2,&TxMessage);
}


/****************************************************
函数名称：CAN2_Shoot_SendMsg
函数功能：将拨弹电机数据解析后发出
函数参数：motor_205*******前升降电机
          motor_206*******后升降电机

函数返回值： 无
描述：将数据存入tx_message结构体再由CAN_Transmit发送
****************************************************/
void CAN2_BulletLift_SendMsg(int16_t motor_205,int16_t motor_206)
{	  
    CanTxMsg tx_message;
    tx_message.StdId = 0x1ff;
    tx_message.IDE = CAN_Id_Standard;//标准帧
    tx_message.RTR = CAN_RTR_Data;   //数据帧
    tx_message.DLC = 0x08;           //帧长度为8
    
    tx_message.Data[0] = (char)(motor_205>>8);
    tx_message.Data[1] = (char)motor_205;
    tx_message.Data[2] = (char)(motor_206>>8);
    tx_message.Data[3] = (char)motor_206;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN2,&tx_message);
}


