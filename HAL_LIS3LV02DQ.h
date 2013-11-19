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
#ifndef __HAL_LIS3LV02DQ_H_
#define __HAL_LIS3LV02DQ_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

/**
* @addtogroup LIS3LV02DQ 
* @{
*/

/**
* @addtogroup  LIS3LV02DQ_I2C_Define
* @{
*/

#define LIS3_I2C                  I2C2
#define LIS3_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define LIS3_I2C_Port             GPIOB
#define LIS3_I2C_SCL_Pin          GPIO_Pin_10
#define LIS3_I2C_SDA_Pin          GPIO_Pin_11
#define LIS3_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define LIS3_I2C_Speed            200000

/**
*@}
*/ /* end of group LIS3LV02DQ_I2C_Define */ 

/**
* @addtogroup Gyro_DRDY_Pin_Define
* @{
*/

//#ifdef  LIS3_DRDY_ENABLE
#define LIS3_DRDY_Pin                   GPIO_Pin_9
#define LIS3_DRDY_Port                  GPIOA
#define LIS3_DRDY_RCC_Port              RCC_APB2Periph_GPIOA
#define LIS3_DRDY_Port_Source           GPIO_PortSourceGPIOA
#define LIS3_DRDY_Pin_Source            GPIO_PinSource9
#define LIS3_DRDY_EXTI_Line             EXTI_Line9
#define LIS3_DRDY_Edge                  EXTI_Trigger_Falling
#define LIS3_DRDY_EXTI_IRQCHANNEL       EXTI9_5_IRQChannel
#define LIS3_DRDY_Preemption_Priority   3	  //´ý¶¨
#define LIS3_DRDY_Sub_Priority          3	  //´ý¶¨
//#endif

#define LIS3_SPI_CS_HIGH() GPIO_SetBits(LIS3_SPI_NSS_Pin_Port,LIS3_SPI_NSS_Pin)
#define LIS3_SPI_CS_LOW() GPIO_ResetBits(LIS3_SPI_NSS_Pin_Port,LIS3_SPI_NSS_Pin)

#define LIS3_SPI                	SPI2
#define LIS3_SPI_RCC_Periph     	RCC_APB1Periph_SPI2
#define LIS3_SPI_RCC_Port			RCC_APB2Periph_GPIOB

#define LIS3_SPI_NSS_Pin_Port      GPIOA
#define LIS3_SPI_NSS_Pin        	GPIO_Pin_10
#define LIS3_SPI_NSS_Pin_Source 	GPIO_PinSource10

#define LIS3_SPI_CLK_Pin_Port      GPIOB
#define LIS3_SPI_CLK_Pin        	GPIO_Pin_13
#define LIS3_SPI_CLK_Pin_Source   	GPIO_PinSource13

#define LIS3_SPI_MISO_Pin_Port     GPIOB
#define LIS3_SPI_MISO_Pin        	GPIO_Pin_14
#define LIS3_SPI_MISO_Pin_Source   GPIO_PinSource14

#define LIS3_SPI_MOSI_Pin_Port     GPIOB
#define LIS3_SPI_MOSI_Pin        	GPIO_Pin_15
#define LIS3_SPI_MOSI_Pin_Source   GPIO_PinSource15

#define LIS3_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_128

#define LIS3_INT1_Pin_Port 			GPIOA
#define LIS3_INT1_Pin_RCC_Periph	RCC_APB2Periph_GPIOA
#define LIS3_INT1_Pin				GPIO_Pin_9
#define LIS3_INT1_Pin_Source		GPIO_PinSource9

#endif /* __HAL_LIS3LV02DQ_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
