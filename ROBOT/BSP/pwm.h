#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"

#define PWM3_1  TIM3->CCR1    //
#define PWM3_2  TIM3->CCR2    //
#define PWM3_3  TIM3->CCR3    //Ä¦²ÁÂÖ1
#define PWM3_4  TIM3->CCR4    //Ä¦²ÁÂÖ2
//#define PWM5  TIM14->CCR1   //µ¯²Ö¸Ç¶æ»ú
				
				
#define FRICTION_INIT      800

void PWM_Config(void);


#endif


