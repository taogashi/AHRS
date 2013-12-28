#ifndef _HAL_MAG3310_
#define _HAL_MAG3310_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

#define MAG3110_I2C                  I2C1
#define MAG3110_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define MAG3110_I2C_Port             GPIOB
#define MAG3110_I2C_SCL_Pin          GPIO_Pin_6
#define MAG3110_I2C_SDA_Pin          GPIO_Pin_7
#define MAG3110_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MAG3110_I2C_Speed            300000


#define MAG3110_INT_Pin                   GPIO_Pin_4
#define MAG3110_INT_Port                  GPIOB
#define MAG3110_INT_RCC_Port              RCC_APB2Periph_GPIOB
#define MAG3110_INT_Port_Source           GPIO_PortSourceGPIOB
#define MAG3110_INT_Pin_Source            GPIO_PinSource4

#ifdef  MAG3110_INT_ENABLE
#define MAG3110_INT_EXTI_Line             EXTI_Line4
#define MAG3110_INT_Edge                  EXTI_Trigger_Falling
#define MAG3110_INT_EXTI_IRQCHANNEL       EXTI4_IRQn
#define MAG3110_INT_Preemption_Priority   3
#define MAG3110_INT_Sub_Priority          3
#endif

#endif
