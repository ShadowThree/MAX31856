#include "max31856.h"
#include "delay.h"
//#include "sys.h"

void SPI1_Init()
{
	GPIO_InitTypeDef GPIO_InitTypeStructure = {0};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;		// CS / CLK / MOSI
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitTypeStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_6;		// MISO
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitTypeStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_4);		// CS
	GPIO_SetBits(GPIOA, GPIO_Pin_5);		// CLK
}
void max31856_Init(void)
{
	SPI1_Init();
	
	writeRegister8(MAX31856_MASK_REG, 0xff);		// 0x02,0xff	// 故障屏蔽
	writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0 | FILTER_50HZ);		// 0x00, 0x10 | 0x01
	writeRegister8(MAX31856_CR1_REG, SAMPLE_POINT_8);
	setThermocoupleType(MAX31856_TCTYPE_K);
}

float max31856_ReadTemp(void)
{
	uint32_t result = 0;
	signed char ret = 1;
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	SPIx_WriteByte(SPI1, 0x0c);
	result = SPIx_ReadByte(SPI1);
	result = (result<<8) | SPIx_ReadByte(SPI1);
	result = (result<<8) | SPIx_ReadByte(SPI1);
	delay_ms(50);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	
	if(result & 0x800000)
	{
		result = ~result + 1; 
		ret = -1;
	}
	return (result>>5) / 128.0 * ret;
}

void writeRegister8(uint8_t addr, uint8_t data)
{
	addr |= 0x80;
	CS_LOW;			//GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	SPI1_ReadWriteByte(addr);
	SPI1_ReadWriteByte(data);
	CS_HIGH;		// GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n)
{
	addr &= 0x7F;
	// CLK_HIGH;
	CS_LOW;			//	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	SPI1_ReadWriteByte(addr);
	while(n--)
	{
		buffer[0] = SPI1_ReadWriteByte(0xFF);
		buffer++;
	}
	CS_HIGH;		// GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

uint8_t readRegister8(uint8_t addr)
{
	uint8_t ret = 0;
	readRegisterN(addr, &ret, 1);
	return ret;
}

uint16_t readRegister16(uint8_t addr)
{
	uint16_t ret;
	uint8_t buffer[2] = {0};
	readRegisterN(addr, buffer, 2);
	ret = buffer[0];
	ret <<= 8;
	ret |= buffer[1];
	return ret;
}

uint32_t readRegister24(uint8_t addr)
{
	uint32_t ret;
	uint8_t buffer[3] = {0};
	readRegisterN(addr, buffer, 3);
	
	ret = buffer[0];
	ret <<= 8;
	ret |= buffer[1];
	ret <<= 8;
	ret |= buffer[2];
	
	return ret;
}

void setThermocoupleType(max31856_thermocoupletype_t type)
{
	uint8_t t = readRegister8(MAX31856_CR1_REG);		// 0x01
	t &= 0xF0;
	t |= (uint8_t)type & 0x0F;
	writeRegister8(MAX31856_CR1_REG, t);
}

max31856_thermocoupletype_t getThermocoupleType(void)
{
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0x0F;
	return (max31856_thermocoupletype_t)(t);
}

void oneShotTemperature(void)
{
	uint8_t t;
	writeRegister8(MAX31856_CJTO_REG, 0x00);	// 冷端温度偏置
	
	t = readRegister8(MAX31856_CR0_REG);		// 0x00		// 读CR1寄存器的值
	t &= ~MAX31856_CR0_AUTOCONVERT;				// 0x80		// 关闭连续转换
	t |= MAX31856_CR0_1SHOT;					// 0x40		// 触发单次转换

	writeRegister8(MAX31856_CR0_REG, t);		// 重新写入
	delay_ms(500);								// 1shot 位置 1 ，CS 拉高后，开始转换。转换所需时间 <= (cnt_sample - 1)*40 + 185 ms
}	

float readCJTemperature(void)
{
	int16_t temp16;
	
	oneShotTemperature();
	
	temp16 = readRegister16(MAX31856_CJTH_REG);
	
	return temp16/256.0;
}

float readThermocoupleTemperature(void)
{
	int32_t temp24;
	float tempfloat;
	oneShotTemperature();
	temp24 = readRegister24(MAX31856_LTCBH_REG);
	if(temp24 & 0x800000)
		temp24 |= 0xFF000000;
	temp24 >>= 5;
	tempfloat = temp24;
	tempfloat /= 128.0;
	
	return tempfloat;
}

uint8_t readFault(void)
{
	return readRegister8(MAX31856_SR_REG);
}

// 调用前状态：确保 CS 为低，CLK 为高
// 退出前状态：确保 CS 为低，CLK 为高
static uint8_t SPI1_ReadWriteByte(uint8_t sByte)
{
	uint8_t rByte = 0, i;
	for(i = 8; i > 0 ; i--)
	{
		CLK_LOW;
		SET_MOSI((sByte & 0x80) >> 7);		// 注意这里是移动 7 位，而不算 8 位
		sByte <<= 1;
		CLK_HIGH;
		rByte <<= 1;				// 注意：必须先左移再读取值
		rByte |= READ_MISO;
	}
	return rByte;
}









