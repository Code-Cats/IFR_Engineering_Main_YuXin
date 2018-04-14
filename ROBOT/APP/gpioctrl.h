#ifndef __GPIOCTRL_H__
#define __GPIOCTRL_H__
#include "main.h"

#define GREEN_LED_ON()      GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_OFF()     GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define RED_LED_ON()            GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define RED_LED_OFF()           GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOC, GPIO_Pin_0)

#define LASER_ON()  GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define LASER_OFF()  GPIO_ResetBits(GPIOC, GPIO_Pin_5)

#define DELAY_CNT  200000    //…¡À∏ ±º‰º‰∏Ù

void LED_Red_Blink(void);
void LED_Green_Blink(void);

#endif

