#ifndef __TAKE_BULLET_H_
#define __TAKE_BULLET_H_
#include "bsp.h"

#define VALVE_ISLAND 0		//电磁阀控制位定义
#define VALVE_BULLET_PROTRACT 1	//前伸
#define VALVE_BULLET_CLAMP 2	//夹紧
#define VALVE_BULLET_STORAGE 3	//弹药舱补弹
#define VALVE_TRAILER 5	//拖车

#define UP_L 0
#define UP_R 1
#define DOWN_L 2
#define DOWN_R 3


typedef enum
{
    BULLET_ACQUIRE,  		//前伸、夹紧、抬起动作	称之为获得过程
    BULLET_POUROUT,			//车身倾斜、舵机旋转	称之为倒弹过程
		BULLET_THROWOUT,			//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
}TakeBulletState_e;


void TakeBullet_Control_Center(void);
u8 SetCheck_GripBulletLift(u8 grip_state);	//夹持机构升降函数//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
u8	SetCheck_LiftAll_To_bullet(u8 bullet_state);	//底盘升降函数	//取弹时底盘升至固定高度，1为升，0为降
void SetCheck_TakeBullet_TakeBack(void);	//切出取弹状态时回位

#endif

