#ifndef _MS5607B_H
#define _MS5607B_H

#include "HAL_MS5607B.h"

typedef struct
{
	uint16_t FS;
	uint16_t C1;
	uint16_t C2;
	uint16_t C3;
	uint16_t C4;
	uint16_t C5;
	uint16_t C6;
	uint16_t CR;
}MS5607B_CaliData;

typedef struct
{
	int32_t dT;
    int32_t TEMP;
}MS5607B_ProcData;

typedef struct
{
	int32_t pressure;
	int16_t temperature;		
	int16_t height;
}MS5607B_DataType;

/*I2C Group*/
#define MS5607B_I2C_ADDRESS     0xEE

#define CMD_RESET 				0x1E // ADC reset command
#define CMD_ADC_READ 			0x00 // ADC read command

#define CMD_ADC_CONV_TEMP 		0x50 // ADC conversion command
#define CMD_ADC_CONV_PRES 		0x40 // ADC D1 conversion
#define OSR_256 				0x00 // ADC D2 conversion
#define OSR_512					0x02
#define OSR_1024				0x04
#define OSR_2048				0x06
#define OSR_4096				0x08

#define CMD_PROM_RD_FS		0xA0
#define CMD_PROM_RD_C1 		0xA2 // Prom read command
#define CMD_PROM_RD_C2 		0xA4 // Prom read command
#define CMD_PROM_RD_C3 		0xA6 // Prom read command
#define CMD_PROM_RD_C4 		0xA8 // Prom read command
#define CMD_PROM_RD_C5 		0xAA // Prom read command
#define CMD_PROM_RD_C6 		0xAC // Prom read command
#define CMD_PROM_RD_CRC		0xAE


void MS5607B_I2C_Init(void);
u8 MS5607B_I2C_WriteCmd(u8 slAddr, u8 cmd);
u8 MS5607B_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

void MS5607B_SPI_Init(void);
u8 MS5607B_SPI_SendByte(u8 byte);
u8 MS5607B_SPI_ReadByte(void);

//reset cmd
u8 MS5607B_Reset(void);
//read calibration data
u8 MS5607B_GetCaliData(MS5607B_CaliData *CaliStructure);
//
u8 MS5607B_StartPressureADC(unsigned char OSR);
//
u8 MS5607B_StartTemperatureADC(unsigned char OSR);
//
u8 MS5607B_ReadADC(uint32_t *Databuff);
//
int32_t	MS5607B_GetTemperature(MS5607B_ProcData *midVal,uint32_t D2,MS5607B_CaliData *CaliStructure);
//
int32_t MS5607B_GetPressure(MS5607B_ProcData *midVal,uint32_t D1,MS5607B_CaliData *CaliStructure);

/************/
//unsigned char MS5607B_StateUpdate_I2C(uint32_t *PressureD, uint32_t *TemperD);
//u8 MS5607B_UpdataData(MS5607B_DataType *data);

#endif
