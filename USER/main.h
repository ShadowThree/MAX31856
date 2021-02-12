// CPU		: STM32F103RCT6
// Flash	: 256 KB	(大容量)  (2KB/page, Range: 0x08000000 - 0x0803FFFF)
//				  page0 range: 0x08000000 - 0x080007FF
// SRAM		: 48 KB

#ifndef __MAIN__H__
#define __MAIN__H__

#include "sys.h"
#include "led.h"
#include "delay.h"
//#include "key.h"
#include "usart.h"
#include "lcd.h"
#include "Dis_Picture.h" 
#include "Text.h"	
#include "GBK_LibDrive.h"	
//#include "timer.h"
//#include "exti.h"
//#include "stmflash.h"
#include "stdio.h"
#include "global.h"
#include "max31856.h"

#define PWM_COUNT (1800-1)		// 40KHz

#define Motor_Stop();				TIM_SetCompare2(TIM5,0); TIM_SetCompare3(TIM5,0)
#define Motor_ClockRotate(n)		TIM_SetCompare3(TIM5,0); TIM_SetCompare2(TIM5,n)
#define Motor_AntiClockRotate(n)	TIM_SetCompare2(TIM5,0); TIM_SetCompare3(TIM5,n)
					
// 只要在需要重新烧写PCR参数的时候才置1，烧写完成后重新置0.
// 默认为0 （通过按键修改PCR参数不需要修改这个值）
#define KEEP_PCR_PARAM_IN_FLASH 	1

#if KEEP_PCR_PARAM_IN_FLASH
	// 设置正反转时间，单位为 s
	uint8_t timeClockRotate = 4;
	uint8_t timeAntiClockRotate = 4;
	uint16_t speedRotate = 0;
#else
	// 正反转时间变量
	uint8_t timeClockRotate;
	uint8_t timeAntiClockRotate;
#endif

extern keySta keySta1;

char text_buffer[255] = {0};
signed char duty_cycle = 0;
void itoa(char* buf, int i);
size_t lenOfTx = 0;												// 非负整数，能够表示系统中可能出现的最大整数

void RCC_Configuration(void);
void myInit(void);

#endif
