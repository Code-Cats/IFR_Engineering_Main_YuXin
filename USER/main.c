#include "main.h"
u16 tim5_t_send[4];

int main(void)
{
   delay_ms(2000);
	 SetWorkState(CHECK_STATE);	//启动后默认自检状态	//后续状态切换不在主函数进行
   delay_ms(500);
	Data_Init();
	 BSP_Init();	//板载外设初始化	//此时定时器启动，开始计时
	Data_Init();	//。。。。。
	 SetWorkState(CHECK_STATE);	//进入自检状态
	 delay_ms(100);

	while(1)
	 {
//		 STEER_IMAGE=tim5_t_send[0];
//		 STEER_RFID=tim5_t_send[1];
		 Screen_Start();
	 }
}





//////////////////////////////////////////////////////////////////
//??????,??printf??,??????use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//??????????                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//??_sys_exit()??????????    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//???fputc?? 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//????,??????   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
