#include "gpioctrl.h"

/**************************
函数名：  LED_Red_Blink
函数功能：红灯闪烁
函数输入： 无
函数返回值： 无
**************************/
void LED_Red_Blink(void)
{
	 static int32_t red;
	 if(red<=DELAY_CNT)
	 {
		  red++;
		  RED_LED_ON();
	 }
	 else
	 {
		  red++;
		  RED_LED_OFF();
		  if(red==2*DELAY_CNT)red=0;
	 }
}
/**************************
函数名：  LED_Green_Blink
函数功能：绿灯闪烁
函数输入： 无
函数返回值： 无
**************************/
void LED_Green_Blink(void)
{
	 static int32_t green;
	 if(green<=DELAY_CNT)
	 {
		  green++;
		  GREEN_LED_ON();
	 }
	 else
	 {
		  green++;
		  GREEN_LED_OFF();
		  if(green==2*DELAY_CNT)green=0;
	 }
}

