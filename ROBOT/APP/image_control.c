#include "main.h"
#include "image_control.h"

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;

#define STEER_IMAGE_INIT	1700
#define STEER_IMAGE_REVERSAL	1100

#define IMAGE_START_DELAY	(1000*10)	//10s后开始
void Screen_Start(void)	//屏幕启动切换到AV信道
{
	if(time_1ms_count<IMAGE_START_DELAY)	//10s后开始
	{
		IMAGE_START=PWM_IO_ON;
	}
	else if(time_1ms_count>IMAGE_START_DELAY&&time_1ms_count<IMAGE_START_DELAY+1000)
	{
		IMAGE_START=PWM_IO_OFF;
	}
	else
	{
		IMAGE_START=PWM_IO_ON;
	}

}

u8 av_cut=0;
u8 Steer_Image_state=0;
u16 steer_image=STEER_IMAGE_REVERSAL;
void Image_Cut_Task(void)	//摄像头切换、舵机
{
	static u8 key_r_last=0;
	
	t_AV_CUT=av_cut*20000;

	if(key_r_last==0&&KeyBoardData[KEY_R].value==1)
	{
		Steer_Image_state=!Steer_Image_state;
	}
	STEER_IMAGE=STEER_IMAGE_INIT-Steer_Image_state*(STEER_IMAGE_INIT-STEER_IMAGE_REVERSAL);
	
	key_r_last=KeyBoardData[KEY_R].value;
}
