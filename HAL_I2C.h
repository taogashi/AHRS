#ifndef _HAL_I2C_H_
#define _HAL_I2C_H_

#include "stm32f10x.h"

#define USER_I2C                  I2C2
#define USER_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define USER_I2C_Port             GPIOB
#define USER_I2C_SCL_Pin          GPIO_Pin_10
#define USER_I2C_SDA_Pin          GPIO_Pin_11
#define USER_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define USER_I2C_Speed            300000

#define USER_I2C_EVIRQ_Channel 	I2C2_EV_IRQn
#define USER_I2C_ERIRQ_Channel	I2C2_ER_IRQn

#endif
