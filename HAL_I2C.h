#ifndef _HAL_I2C_H_
#define _HAL_I2C_H_

#include "stm32f10x.h"

#define USER_I2C                  I2C1
#define USER_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define USER_I2C_Port             GPIOB
#define USER_I2C_SCL_Pin          GPIO_Pin_6
#define USER_I2C_SDA_Pin          GPIO_Pin_7
#define USER_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define USER_I2C_Speed            300000

#define USER_I2C_EVIRQ_Channel 	I2C1_EV_IRQn
#define USER_I2C_ERIRQ_Channel	I2C1_ER_IRQn

#endif
