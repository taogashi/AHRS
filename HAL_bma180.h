/* Define to prevent recursive inclusion*/
#ifndef __HAL_BMA180_H_
#define __HAL_BMA180_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

#define BMA180_SPI_CS_HIGH() GPIO_SetBits(BMA180_SPI_NSS_Pin_Port,BMA180_SPI_NSS_Pin)
#define BMA180_SPI_CS_LOW() GPIO_ResetBits(BMA180_SPI_NSS_Pin_Port,BMA180_SPI_NSS_Pin)

#define BMA180_SPI                	SPI2
#define BMA180_SPI_RCC_Periph     	RCC_APB1Periph_SPI2
#define BMA180_SPI_RCC_Port			RCC_APB2Periph_GPIOB

#define BMA180_SPI_NSS_Pin_Port      GPIOA
#define BMA180_SPI_NSS_Pin        	GPIO_Pin_9
#define BMA180_SPI_NSS_Pin_Source 	GPIO_PinSource9

//#define BMA180_SPI_NSS_Pin_Port      GPIOB
//#define BMA180_SPI_NSS_Pin        	GPIO_Pin_12
//#define BMA180_SPI_NSS_Pin_Source 	GPIO_PinSource12

#define BMA180_SPI_CLK_Pin_Port      GPIOB
#define BMA180_SPI_CLK_Pin        	GPIO_Pin_13
#define BMA180_SPI_CLK_Pin_Source   	GPIO_PinSource13

#define BMA180_SPI_MISO_Pin_Port     GPIOB
#define BMA180_SPI_MISO_Pin        	GPIO_Pin_14
#define BMA180_SPI_MISO_Pin_Source   GPIO_PinSource14

#define BMA180_SPI_MOSI_Pin_Port     GPIOB
#define BMA180_SPI_MOSI_Pin        	GPIO_Pin_15
#define BMA180_SPI_MOSI_Pin_Source   GPIO_PinSource15

#define BMA180_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_128

#define BMA180_INT_Pin_Port 	GPIOA
#define BMA180_INT_Pin_RCC_Periph	RCC_APB2Periph_GPIOA
#define BMA180_INT_Pin		GPIO_Pin_10
#define BMA180_INT_Pin_Source	GPIO_PinSource10

#endif /* __HAL_BMA180_H */

/******************* (C) COPYRIGHT 2013 Skyworks copyright *****END OF FILE****/

