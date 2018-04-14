#include "usart3_viceboard.h"


void USART3_ViceBoard_Init(uint32_t baud_rate)
{

    GPIO_InitTypeDef gpio;
	  USART_InitTypeDef usart;
	  NVIC_InitTypeDef nvic;
//    DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);
    
    USART_DeInit(USART3);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
	  usart.USART_Parity = USART_Parity_No;
   // usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usart);
//		
//		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//		
//	  DMA_DeInit(DMA1_Stream1);
//    DMA_StructInit(&dma);
//    dma.DMA_Channel = DMA_Channel_4;
//    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART3_DMA_RX_BUF[0][0];
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//    dma.DMA_BufferSize = sizeof(_USART3_DMA_RX_BUF)/2;
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Circular;
//    dma.DMA_Priority = DMA_Priority_Medium;
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA1_Stream1, &dma);
//		
//		
//		    //配置Memory1,Memory0是第一个使用的Memory
//    DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)&_USART3_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//    DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
//    
//    DMA_Cmd(DMA1_Stream1, ENABLE);
		
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;	//1
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);
		
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);        //usart rx idle interrupt  enabled
    USART_Cmd(USART3, ENABLE);
}

u32 vice_count=0;
u8 USART3_Res=0;
void USART3_IRQHandler(void)
{
////		if(Vision_Flag==0)
////		{
//	static uint32_t this_time_rx_len = 0;

	vice_count++;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		
		USART3_Res=USART_ReceiveData(USART3);
		Data_Receive(USART3_Res);
		//clear the idle pending flag 
		(void)USART3->SR;
		(void)USART3->DR;
	}

}


