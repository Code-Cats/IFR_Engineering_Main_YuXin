#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"

#define PWM3_1  TIM3->CCR1    //
#define PWM3_2  TIM3->CCR2    //
#define PWM3_3  TIM3->CCR3    //Ä¦²ÁÂÖ1
#define PWM3_4  TIM3->CCR4    //Ä¦²ÁÂÖ2
//#define PWM5  TIM14->CCR1   //µ¯²Ö¸Ç¶æ»ú
#define PWM5_1 TIM5->CCR1
#define PWM5_2 TIM5->CCR2
#define PWM5_3 TIM5->CCR3
#define PWM5_4 TIM5->CCR4

#define STEER_UP_L PWM3_1
#define STEER_UP_R PWM3_2
#define STEER_DOWN_L PWM3_3
#define STEER_DOWN_R PWM3_4

#define STEER_IMAGE PWM5_4
#define STEER_RFID	PWM5_3
				
#define FRICTION_INIT      800

void PWM_Config(void);


#endif


