#ifndef __DELAY_H
#define __DELAY_H 			   
//#include "sys.h"


typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef uint32_t  u32;
typedef uint16_t u16;

//////////////////////////////////////////////////////////////////////////////////	 
//STM32������
//ʹ��SysTick����ͨ����ģʽ���ӳٽ��й���
//����delay_us,delay_ms

////////////////////////////////////////////////////////////////////////////////// 	
void delay_init(void);
void delay_ms(u16 nms);
//void delay_us(u32 nus);

#endif





























