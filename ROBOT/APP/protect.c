#include "protect.h"
#include "control.h"
/*
���ļ���;���ṩ�����������㷨���������л����Լ�����ʵʱ״̬��⣬����״̬�л�
Ԥ���幦�ܣ�
1.��̨��̬��������̬����Դ�����㷨
2.�����������⼰��Ч��ֹ
3.�����ģ�����߼����
4.�����������
5.����...
*/
extern RC_Ctl_t RC_Ctl;


#define LOST_THRESHOLD 5

Error_check_t Error_Check={LOST_CYCLE,{0},{0}};

u8 Error_check_workstate=1;

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

u8 t_error_i_record=0;
void Check_Task(void)
{
	for(int i=0;i<LOST_TYPE_NUM;i++)
	{
		LostCountAdd(&Error_Check.count[i]);
		LostCountCheck(Error_Check.count[i],&Error_Check.statu[i],Error_Check.cycle[i]);
	}
	
//	if(Error_Check.statu[LOST_IMU]==1)	//���̳���δ��������
//	{
//		SetWorkState(ERROR_STATE);
//	}
	if(Error_check_workstate==1)	//����״̬
	{
		for(int i=4;i<LOST_TYPE_NUM;i++)	//����һ����������
		{
			if(Error_Check.statu[i]==1)
			{
				t_error_i_record=i;
				SetWorkState(ERROR_STATE);
			}	
		}
		

	}
	
	if(Error_Check.statu[LOST_DBUS]==1)	//ң������������Ҫ��
	{
		if(GetWorkState()==CHECK_STATE)
		{
			SetWorkState(LOST_STATE);	//����ʱû��ң���źŵ�ѡ��
		}
		else if(GetWorkState()!=ERROR_STATE)
		{
			SetWorkState(PROTECT_STATE);
		}
	}
	
	if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)
	{
		Error_check_workstate=0;
	}

}



