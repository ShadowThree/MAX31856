// CPU		: STM32F103RCT6
// Flash	: 256 KB	(������)  (2KB/page, Range: 0x08000000 - 0x0803FFFF)
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
					
// ֻҪ����Ҫ������дPCR������ʱ�����1����д��ɺ�������0.
// Ĭ��Ϊ0 ��ͨ�������޸�PCR��������Ҫ�޸����ֵ��
#define KEEP_PCR_PARAM_IN_FLASH 	1

#if KEEP_PCR_PARAM_IN_FLASH
	// ��������תʱ�䣬��λΪ s
	uint8_t timeClockRotate = 4;
	uint8_t timeAntiClockRotate = 4;
	uint16_t speedRotate = 0;
#else
	// ����תʱ�����
	uint8_t timeClockRotate;
	uint8_t timeAntiClockRotate;
#endif

extern keySta keySta1;

char text_buffer[255] = {0};
signed char duty_cycle = 0;
void itoa(char* buf, int i);
size_t lenOfTx = 0;												// �Ǹ��������ܹ���ʾϵͳ�п��ܳ��ֵ��������

void RCC_Configuration(void);
void myInit(void);

#endif
