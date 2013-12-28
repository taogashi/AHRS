
/* Define to prevent recursive inclusion*/
#ifndef __HAL_SD740_H_
#define __HAL_SD740_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

/**
* @addtogroup SD740 
* @{
*/

/**
* @addtogroup  SD740_I2C_Define
* @{
*/

#define SD740_SPI                	SPI2
#define SD740_SPI_RCC_Periph     	RCC_APB1Periph_SPI2
#define SD740_SPI_RCC_Port			RCC_APB2Periph_GPIOB

#define SD740_SPI_NSS_Pin_Port      GPIOB
#define SD740_SPI_NSS_Pin        	GPIO_Pin_12
#define SD740_SPI_NSS_Pin_Source 	GPIO_PinSource12

#define SD740_SPI_CLK_Pin_Port      GPIOB
#define SD740_SPI_CLK_Pin        	GPIO_Pin_13
#define SD740_SPI_CLK_Pin_Source   	GPIO_PinSource13

#define SD740_SPI_MISO_Pin_Port     GPIOB
#define SD740_SPI_MISO_Pin        	GPIO_Pin_14
#define SD740_SPI_MISO_Pin_Source   GPIO_PinSource14

#define SD740_SPI_MOSI_Pin_Port     GPIOB
#define SD740_SPI_MOSI_Pin        	GPIO_Pin_15
#define SD740_SPI_MOSI_Pin_Source   GPIO_PinSource15

#define SD740_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_64


#endif /* __HAL_SD740_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
