#ifndef _MS5607B_H
#define _MS5607B_H

#include "HAL_MS5607B.h"

#define MS5607B_USE_I2C

typedef struct
{
	int32_t pressure;
	int32_t temperature;		
}MS5607B_DataType;

/*I2C Group*/
#define MS5607B_I2C_ADDRESS     0xEE
#define CMD_RESET 		0x1E // ADC reset command
#define CMD_ADC_READ 		0x00 // ADC read command
#define CMD_ADC_CONV 		0x40 // ADC conversion command
#define CMD_ADC_D1 		0x00 // ADC D1 conversion
#define CMD_ADC_D2 		0x10 // ADC D2 conversion
#define CMD_ADC_256 		0x00 // ADC OSR=256
#define CMD_ADC_512 		0x02 // ADC OSR=512
#define CMD_ADC_1024 		0x04 // ADC OSR=1024
#define CMD_ADC_2048 		0x06 // ADC OSR=2048
#define CMD_ADC_4096 		0x08 // ADC OSR=4096
#define CMD_PROM_RD 		0xA0 // Prom read command

void MS5607B_I2C_Init(void);
u8 MS5607B_I2C_WriteCmd(u8 slAddr, u8 cmd);
u8 MS5607B_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

//重置传感器
u8 MS5607B_Reset(void);
//读取校正参数
u8 MS5607B_GetCaliData(uint16_t *baroCali);
//触发气压测量
u8 MS5607B_StartPressureADC(void);
//触发温度测量
u8 MS5607B_StartTemperatureADC(void);
//获取数据
u8 MS5607B_GetData_I2C(uint32_t *Databuff);
//计算真实的温度
int32_t	MS5607B_GetTemperature(uint32_t D1, uint32_t D2, uint16_t *baroCali);
//计算真是的气压
int32_t MS5607B_GetPressure(uint32_t D1, uint32_t D2, uint16_t *baroCali);

/************/
//unsigned char MS5607B_StateUpdate_I2C(uint32_t *PressureD, uint32_t *TemperD);
//u8 MS5607B_UpdataData(MS5607B_DataType *data);

#endif
