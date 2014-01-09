#ifndef _HAL_MS5607B_H
#define _HAL_MS5607B_H
					   
#include "stm32f10x.h"

#define MS5607B_I2C                  I2C2
#define MS5607B_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define MS5607B_I2C_Port             GPIOB
#define MS5607B_I2C_SCL_Pin          GPIO_Pin_10
#define MS5607B_I2C_SDA_Pin          GPIO_Pin_11
#define MS5607B_I2C_RCC_Port         RCC_APB2Periph_GPIOB								 
#define MS5607B_I2C_Speed            400000

#define MS5607B_SPI_CS_HIGH() 		GPIO_SetBits(MS5607B_SPI_NSS_Pin_Port,MS5607B_SPI_NSS_Pin)
#define MS5607B_SPI_CS_LOW() 		GPIO_ResetBits(MS5607B_SPI_NSS_Pin_Port,MS5607B_SPI_NSS_Pin)

#define MS5607B_SPI_NSS_Pin_Port      	GPIOB
#define MS5607B_SPI_NSS_Pin        		GPIO_Pin_12
#define MS5607B_SPI_NSS_Pin_Source 		GPIO_PinSource12
#define MS5607B_SPI_NSS_Pin_RCC_Port	RCC_APB2Periph_GPIOB

#endif
