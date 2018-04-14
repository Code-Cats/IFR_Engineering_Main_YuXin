#ifndef __BSP_H__
#define __BSP_H__

#include "main.h"
#include "can1.h"
#include "can2.h"
#include "pwm.h"
#include "gpio.h"
#include "usart1_remote.h"
#include "heartbeat.h"
#include "usart3_viceboard.h"
#include "viceboard_analysis.h"
#include "usart6_wifi_Debug.h"

#include "uart4.h"
#include "imu_data_decode.h"
#include "packet.h"
#include "dma_uart4.h"

void BSP_Init(void);



#endif 

