/**
  * @file    HAL_ITG303DLH.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   Hardware Abstraction Layer for ITG303DLH.
  * @details
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
  * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
  * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Define to prevent recursive inclusion*/
#ifndef __HAL_HMC5843_H_
#define __HAL_HMC5843_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

/**
* @addtogroup HMC5843 
* @{
*/

/**
* @addtogroup  HMC5843_I2C_Define
* @{
*/

#define HMC_I2C                  I2C2
#define HMC_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define HMC_I2C_Port             GPIOB
#define HMC_I2C_SCL_Pin          GPIO_Pin_10
#define HMC_I2C_SDA_Pin          GPIO_Pin_11
#define HMC_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define HMC_I2C_Speed            200000

/**
*@}
*/ /* end of group HMC5843_I2C_Define */ 

/**
* @addtogroup Gyro_DRDY_Pin_Define
* @{
*/

//#ifdef  HMC_DRDY_ENABLE
#define HMC_DRDY_Pin                   GPIO_Pin_7
#define HMC_DRDY_Port                  GPIOC
#define HMC_DRDY_RCC_Port              RCC_APB2Periph_GPIOC
#define HMC_DRDY_Port_Source           GPIO_PortSourceGPIOC
#define HMC_DRDY_Pin_Source            GPIO_PinSource7
#define HMC_DRDY_EXTI_Line             EXTI_Line7
#define HMC_DRDY_Edge                  EXTI_Trigger_Falling
#define HMC_DRDY_EXTI_IRQCHANNEL       EXTI9_5_IRQChannel
#define HMC_DRDY_Preemption_Priority   3	  //´ý¶¨
#define HMC_DRDY_Sub_Priority          3	  //´ý¶¨
//#endif

/**
* @}
*/ /* end of group HMC_DRDY_Pin_Define */

/**
 * @}
 */ /* end of group ITG */


#endif /* __HAL_HMC5843_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
