#include "main.h"

int main(void)
{
	float temp;
	
	myInit();		// 一些模块和IO口初始化
	while(1)
	{
		DrawFont_GBK24B(0,0,0x0000,(u8*)"Cold Junction Temp: ");
		temp = readCJTemperature();
		sprintf(text_buffer, "%f", temp);
		DrawFont_GBK24B(8,24,0x0000,(u8*)text_buffer);
		
		DrawFont_GBK24B(0,48,0x0000,(u8*)"Thermocouple Temp: ");
		temp = readThermocoupleTemperature();
		sprintf(text_buffer, "%f", temp);
		DrawFont_GBK24B(8,72,0x0000,(u8*)text_buffer);
	} 
}

void myInit(void)
{	
	//RCC_Configuration();		// 在startup_stm32f10x_hd.s文件中已经调用了SystemInit函数对时钟进行初始化。
								// the clock already setting in startup_stm32f10x_hd.s, so not need set again.
	delay_init();	    	     //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
 	LED_Init();			      	// GPIO初始化
	LCD_Init();           		// 初始化LCD SPI 接口
	max31856_Init();
	
	switch (getThermocoupleType()) {
		case MAX31856_TCTYPE_B: DrawFont_GBK24B(0,200,0x0000,(u8*)"B Type"); break;
		case MAX31856_TCTYPE_E: DrawFont_GBK24B(0,200,0x0000,(u8*)"E Type"); break;
		case MAX31856_TCTYPE_J: DrawFont_GBK24B(0,200,0x0000,(u8*)"J Type"); break;
		case MAX31856_TCTYPE_K: DrawFont_GBK24B(0,200,0x0000,(u8*)"K Type"); break;
		case MAX31856_TCTYPE_N: DrawFont_GBK24B(0,200,0x0000,(u8*)"N Type"); break;
		case MAX31856_TCTYPE_R: DrawFont_GBK24B(0,200,0x0000,(u8*)"R Type"); break;
		case MAX31856_TCTYPE_S: DrawFont_GBK24B(0,200,0x0000,(u8*)"S Type"); break;
		case MAX31856_TCTYPE_T: DrawFont_GBK24B(0,200,0x0000,(u8*)"T Type"); break;
		case MAX31856_VMODE_G8: DrawFont_GBK24B(0,200,0x0000,(u8*)"Voltage x8 Gain mode"); break;
		case MAX31856_VMODE_G32: DrawFont_GBK24B(0,200,0x0000,(u8*)"Voltage x32 Gain mode"); break;
		default: DrawFont_GBK24B(0,200,0x0000,(u8*)"Unknown"); break;
	}
}
void RCC_Configuration(void)
{
	//----------使用外部RC晶振-----------
	RCC_DeInit();		// 初始化为缺省值
	RCC_HSEConfig(RCC_HSE_ON);	//使能外部的高速时钟 
	while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);	//等待外部高速时钟使能就绪
	
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	//Enable Prefetch Buffer
	FLASH_SetLatency(FLASH_Latency_2);		//Flash 2 wait state
	
	RCC_HCLKConfig(RCC_SYSCLK_Div1);		//HCLK = SYSCLK
	RCC_PCLK2Config(RCC_HCLK_Div1);			//PCLK2 =  HCLK
	RCC_PCLK1Config(RCC_HCLK_Div2);			//PCLK1 = HCLK/2
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);	//PLLCLK = 8MHZ * 9 =72MHZ
	RCC_PLLCmd(ENABLE);			//Enable PLLCLK
 
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}		//Wait till PLLCLK is ready
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	//Select PLL as system clock
	while(RCC_GetSYSCLKSource()!=0x08);		//Wait till PLL is used as system clock source
	
	//---------打开相应外设时钟--------------------
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//使能APB2外设的GPIOA的时钟
}

void itoa(char* buf, int i)
{
	buf[0] = i / 10000 + 0x30;
	buf[1] = i % 10000 / 1000 + 0x30;
	buf[2] = i % 1000 / 100 + 0x30;
	buf[3] = i % 100 / 10 + 0x30;
	buf[4] = i % 10 + 0x30;
	buf[5] = 0;
	while(buf[0] == 0x30)
	{
		buf[0] = buf[1];
		buf[1] = buf[2];
		buf[2] = buf[3];
		buf[3] = buf[4];
		buf[4] = buf[5];
	}
}
