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

#endif
