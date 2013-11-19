//与平台相关的定义
//只要是在stm32上进行移植，只需要修改这个文件的内容
/* Define to prevent recursive inclusion*/
#ifndef __HAL_ITG3200_H_
#define __HAL_ITG3200_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

/**
* @addtogroup ITG3200 
* @{
*/

/**
* @addtogroup  ITG3200_I2C_Define
* @{
*/

#define ITG_I2C                  I2C2
#define ITG_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define ITG_I2C_Port             GPIOB
#define ITG_I2C_SCL_Pin          GPIO_Pin_10
#define ITG_I2C_SDA_Pin          GPIO_Pin_11
#define ITG_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define ITG_I2C_Speed            300000

/**
*@}
*/ /* end of group ITG3200_I2C_Define */ 

/**
* @addtogroup Gyro_DRDY_Pin_Define
* @{
*/

//#ifdef  ITG_DRDY_ENABLE
#define ITG_DRDY_Pin                   GPIO_Pin_15
#define ITG_DRDY_Port                  GPIOA
#define ITG_DRDY_RCC_Port              RCC_APB2Periph_GPIOA
#define ITG_DRDY_Port_Source           GPIO_PortSourceGPIOA
#define ITG_DRDY_Pin_Source            GPIO_PinSource15
#define ITG_DRDY_EXTI_Line             EXTI_Line15
#define ITG_DRDY_Edge                  EXTI_Trigger_Falling
#define ITG_DRDY_EXTI_IRQCHANNEL       EXTI9_5_IRQChannel
#define ITG_DRDY_Preemption_Priority   3	  //待定
#define ITG_DRDY_Sub_Priority          3	  //待定
//#endif

/**
* @}
*/ /* end of group ITG_DRDY_Pin_Define */

/**
 * @}
 */ /* end of group ITG */


#endif /* __HAL_ITG3200_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
