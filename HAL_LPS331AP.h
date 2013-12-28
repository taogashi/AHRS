/* Define to prevent recursive inclusion*/
#ifndef __HAL_LPS331AP_H_
#define __HAL_LPS331AP_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

#define LPS331AP_SPI_CS_HIGH() GPIO_SetBits(LPS331AP_SPI_NSS_Pin_Port,LPS331AP_SPI_NSS_Pin)
#define LPS331AP_SPI_CS_LOW() GPIO_ResetBits(LPS331AP_SPI_NSS_Pin_Port,LPS331AP_SPI_NSS_Pin)

#define LPS331AP_SPI                	SPI2
#define LPS331AP_SPI_RCC_Periph     	RCC_APB1Periph_SPI2
#define LPS331AP_SPI_RCC_Port			RCC_APB2Periph_GPIOB

#define LPS331AP_SPI_NSS_Pin_Port      GPIOB
#define LPS331AP_SPI_NSS_Pin        	GPIO_Pin_3
#define LPS331AP_SPI_NSS_Pin_Source 	GPIO_PinSource3

//#define LPS331AP_SPI_NSS_Pin_Port      GPIOB
//#define LPS331AP_SPI_NSS_Pin        	GPIO_Pin_12
//#define LPS331AP_SPI_NSS_Pin_Source 	GPIO_PinSource12

#define LPS331AP_SPI_CLK_Pin_Port      GPIOB
#define LPS331AP_SPI_CLK_Pin        	GPIO_Pin_13
#define LPS331AP_SPI_CLK_Pin_Source   	GPIO_PinSource13

#define LPS331AP_SPI_MISO_Pin_Port     GPIOB
#define LPS331AP_SPI_MISO_Pin        	GPIO_Pin_14
#define LPS331AP_SPI_MISO_Pin_Source   GPIO_PinSource14

#define LPS331AP_SPI_MOSI_Pin_Port     GPIOB
#define LPS331AP_SPI_MOSI_Pin        	GPIO_Pin_15
#define LPS331AP_SPI_MOSI_Pin_Source   GPIO_PinSource15

#define LPS331AP_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_128

#define LPS331AP_INT1_Pin_Port 			GPIOA
#define LPS331AP_INT1_Pin_RCC_Periph	RCC_APB2Periph_GPIOA
#define LPS331AP_INT1_Pin				GPIO_Pin_11
#define LPS331AP_INT1_Pin_Source		GPIO_PinSource11

#define LPS331AP_INT2_Pin_Port 			GPIOA
#define LPS331AP_INT2_Pin_RCC_Periph	RCC_APB2Periph_GPIOA
#define LPS331AP_INT2_Pin				GPIO_Pin_15
#define LPS331AP_INT2_Pin_Source		GPIO_PinSource15

#endif /* __HAL_LPS331AP_H */

/******************* (C) COPYRIGHT 2013 Skyworks copyright *****END OF FILE****/

