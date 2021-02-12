#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED PAout(8)	// PAout(8) 表示一个可以控制PA8管脚输出状态的地址。对这个地址赋值就可以改变管脚输出状态

void LED_Init(void);	//初始化

#endif
