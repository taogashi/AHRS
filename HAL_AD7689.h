#ifndef _HAL_AD7689_H_
#define _HAL_AD7689_H_

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

#define AD7689_SPI_CS_HIGH() GPIO_SetBits(AD7689_SPI_NSS_Pin_Port,AD7689_SPI_NSS_Pin)
#define AD7689_SPI_CS_LOW() GPIO_ResetBits(AD7689_SPI_NSS_Pin_Port,AD7689_SPI_NSS_Pin)

#define AD7689_SPI                	SPI2
#define AD7689_SPI_RCC_Periph     	RCC_APB1Periph_SPI2
#define AD7689_SPI_RCC_Port			RCC_APB2Periph_GPIOB

#define AD7689_SPI_NSS_Pin_Port      GPIOA
#define AD7689_SPI_NSS_Pin        	GPIO_Pin_15
#define AD7689_SPI_NSS_Pin_Source 	GPIO_PinSource15
#define AD7689_SPI_NSS_Pin_RCC_Port	 RCC_APB2Periph_GPIOA

//#define AD7689_SPI_NSS_Pin_Port      GPIOB
//#define AD7689_SPI_NSS_Pin        	GPIO_Pin_12
//#define AD7689_SPI_NSS_Pin_Source 	GPIO_PinSource12

#define AD7689_SPI_CLK_Pin_Port      GPIOB
#define AD7689_SPI_CLK_Pin        	GPIO_Pin_13
#define AD7689_SPI_CLK_Pin_Source   	GPIO_PinSource13

#define AD7689_SPI_MISO_Pin_Port     GPIOB
#define AD7689_SPI_MISO_Pin        	GPIO_Pin_14
#define AD7689_SPI_MISO_Pin_Source   GPIO_PinSource14

#define AD7689_SPI_MOSI_Pin_Port     GPIOB
#define AD7689_SPI_MOSI_Pin        	GPIO_Pin_15
#define AD7689_SPI_MOSI_Pin_Source   GPIO_PinSource15

#define AD7689_SPI_BaudRatePrescaler	SPI_BaudRatePrescaler_32

#endif
