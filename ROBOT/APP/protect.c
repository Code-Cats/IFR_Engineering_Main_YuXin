#include "protect.h"
#include "control.h"
/*
该文件用途：提供传感器冗余算法，传感器切换，以及车体实时状态检测，保护状态切换
预定义功能：
1.云台姿态，底盘姿态多来源冗余算法
2.车体误操作检测及有效制止
3.车体各模块在线检测检错
4.防疯防抢防盗
5.待续...
*/
#define LOST_THRESHOLD 5

Error_check_t Error_Check={LOST_CYCLE,{0},{0}};


void LostCountAdd(u16* lostcount)
{
	if(*lostcount<0xFFFE)
	(*lostcount)++;
}

void LostCountFeed(u16* lostcount)
{
	*lostcount=0;
}

u8 LostCountCheck(u16 lostcount,u8* statu,const u16 cycle)
{
	if(lostcount>LOST_THRESHOLD*cycle)
		*statu=1;
	else
		*statu=0;
	return *statu;
}


void Check_Task(void)
{
	for(int i=0;i<LOST_TYPE_NUM;i++)
	{
		LostCountAdd(&Error_Check.count[i]);
		LostCountCheck(Error_Check.count[i],&Error_Check.statu[i],Error_Check.cycle[i]);
	}
	
//	if(Error_Check.statu[LOST_IMU]==1)	//工程车暂未加陀螺仪
//	{
//		SetWorkState(ERROR_STATE);
//	}
	
	for(int i=4;i<LOST_TYPE_NUM-1;i++)	//控在一切正常后检测
	{
		if(Error_Check.statu[i]==1)
			SetWorkState(ERROR_STATE);
	}
	
	if(Error_Check.statu[LOST_DBUS]==1)
	{
		if(GetWorkState()==CHECK_STATE)
		{
			SetWorkState(LOST_STATE);	//启动时没有遥控信号的选择
		}
		else if(GetWorkState()!=ERROR_STATE)
		{
			SetWorkState(PROTECT_STATE);
		}
	}
	

}



