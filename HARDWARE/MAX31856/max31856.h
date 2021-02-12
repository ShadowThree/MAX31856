#ifndef _MAX31856__H__
#define _MAX31856__H__

#include "sys.h"

#define CS_HIGH			GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define CS_LOW			GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define CLK_HIGH		GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define CLK_LOW			GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define SET_MOSI(n) 	if((n) == 1) 								\
							GPIO_SetBits(GPIOA, GPIO_Pin_7);		\
						else if((n) == 0)							\
							GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define READ_MISO		GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)

// 寄存器 00H
#define W_ADDR_CR0			0x80
#define ONE_SHOUT_MODE		0x00
//#define CONTINUE_MODE		0x80
#define TRIGLE_CONVERT		0x40
#define FILTER_50HZ			0x01

// 寄存器 01H
#define W_ADDR_CR1			0x01
#define TEMP_SENSOR_TYPE_K	0x03		// 默认值
#define TEMP_SENSOR_TYPE_T	0x07
#define SAMPLE_POINT_8		0x30

#define MAX31856_CR0_REG           0x00    ///< Config 0 register
#define MAX31856_CR0_AUTOCONVERT   0x80    ///< Config 0 Auto convert flag
#define MAX31856_CR0_1SHOT         0x40    ///< Config 0 one shot convert flag
#define MAX31856_CR0_OCFAULT1      0x20    ///< Config 0 open circuit fault 1 flag
#define MAX31856_CR0_OCFAULT0      0x10    ///< Config 0 open circuit fault 0 flag
#define MAX31856_CR0_CJ            0x08    ///< Config 0 cold junction disable flag
#define MAX31856_CR0_FAULT         0x04    ///< Config 0 fault mode flag
#define MAX31856_CR0_FAULTCLR      0x02    ///< Config 0 fault clear flag

#define MAX31856_CR1_REG           0x01    ///< Config 1 register
#define MAX31856_MASK_REG          0x02    ///< Fault Mask register
#define MAX31856_CJHF_REG          0x03    ///< Cold junction High temp fault register
#define MAX31856_CJLF_REG          0x04    ///< Cold junction Low temp fault register
#define MAX31856_LTHFTH_REG        0x05    ///< Linearized Temperature High Fault Threshold Register, MSB
#define MAX31856_LTHFTL_REG        0x06    ///< Linearized Temperature High Fault Threshold Register, LSB
#define MAX31856_LTLFTH_REG        0x07    ///< Linearized Temperature Low Fault Threshold Register, MSB
#define MAX31856_LTLFTL_REG        0x08    ///< Linearized Temperature Low Fault Threshold Register, LSB
#define MAX31856_CJTO_REG          0x09    ///< Cold-Junction Temperature Offset Register 
#define MAX31856_CJTH_REG          0x0A    ///< Cold-Junction Temperature Register, MSB
#define MAX31856_CJTL_REG          0x0B    ///< Cold-Junction Temperature Register, LSB
#define MAX31856_LTCBH_REG         0x0C    ///< Linearized TC Temperature, Byte 2 
#define MAX31856_LTCBM_REG         0x0D    ///< Linearized TC Temperature, Byte 1
#define MAX31856_LTCBL_REG         0x0E    ///< Linearized TC Temperature, Byte 0
#define MAX31856_SR_REG            0x0F    ///< Fault Status Register

#define MAX31856_FAULT_CJRANGE     0x80    ///< Fault status Cold Junction Out-of-Range flag
#define MAX31856_FAULT_TCRANGE     0x40    ///< Fault status Thermocouple Out-of-Range flag
#define MAX31856_FAULT_CJHIGH      0x20    ///< Fault status Cold-Junction High Fault flag
#define MAX31856_FAULT_CJLOW       0x10    ///< Fault status Cold-Junction Low Fault flag
#define MAX31856_FAULT_TCHIGH      0x08    ///< Fault status Thermocouple Temperature High Fault flag
#define MAX31856_FAULT_TCLOW       0x04    ///< Fault status Thermocouple Temperature Low Fault flag
#define MAX31856_FAULT_OVUV        0x02    ///< Fault status Overvoltage or Undervoltage Input Fault flag
#define MAX31856_FAULT_OPEN        0x01    ///< Fault status Thermocouple Open-Circuit Fault flag

/** Noise filtering options enum. Use with setNoiseFilter() */
enum _max31856_noise_filter{
	MAX31856_NOISE_FILTER_50HZ,
	MAX31856_NOISE_FILTER_60HZ
};
typedef enum _max31856_noise_filter max31856_noise_filter_t;

/** Multiple types of thermocouples supported */
enum _max31856_thermocoupletype {
  MAX31856_TCTYPE_B = 0,
  MAX31856_TCTYPE_E = 1,
  MAX31856_TCTYPE_J = 2,
  MAX31856_TCTYPE_K = 3,
  MAX31856_TCTYPE_N = 4,
  MAX31856_TCTYPE_R = 5,
  MAX31856_TCTYPE_S = 6,
  MAX31856_TCTYPE_T = 7,
  MAX31856_VMODE_G8,		// 0b10xx
  MAX31856_VMODE_G32 = 12,	// 0b11xx
};
typedef enum _max31856_thermocoupletype max31856_thermocoupletype_t;

void max31856_Init(void);		// 初始化管脚，配置SPI参数，配置MAX31856工作模式，滤波频率，采样点以及热电偶类型
//void max31856_SetSenseType(uint8_t SenseType);
float max31856_ReadTemp(void);
void writeRegister8(uint8_t addr, uint8_t data);
void setThermocoupleType(max31856_thermocoupletype_t type);
max31856_thermocoupletype_t getThermocoupleType(void);
void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
uint8_t readRegister8(uint8_t addr);
uint16_t readRegister16(uint8_t addr);
uint32_t readRegister24(uint8_t addr);
max31856_thermocoupletype_t getThermocoupleType(void);
float readCJTemperature(void);
void oneShotTemperature(void);
float readThermocoupleTemperature(void);
uint8_t readFault(void);
void SPI1_Init(void);
static uint8_t SPI1_ReadWriteByte(uint8_t sByte);
static uint8_t SPIx_ReadByte(SPI_TypeDef * SPIx){return 0;}
static void SPIx_WriteByte(SPI_TypeDef * SPIx, uint8_t sByte){}

#endif
