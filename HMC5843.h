/**
  * @file    ITG303DLH.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   Header for ITG303DLH.c file
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



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HMC5843_H_
#define _HMC5843_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "HAL_HMC5843.h"

/**
* @addtogroup HMC5843
* @{
*/

#define HMC_ODR_50HZ                   			((u8)0x18)
#define HMC_ODR_20Hz                   			((u8)0x14)
#define HMC_ODR_10Hz                   			((u8)0x10)
#define HMC_ODR_5Hz                   			((u8)0x0C)
#define HMC_ODR_2Hz                   			((u8)0x08)

#define HMC_Full_Scale_0_7Ga                   	((u8)0x00)
#define HMC_Full_Scale_1Ga                   	((u8)0x20)
#define HMC_Full_Scale_1_5Ga                   	((u8)0x40)
#define HMC_Full_Scale_2_0Ga                   	((u8)0x60)

#define HMC_Continuous_Mode                     ((u8)0x00)
#define HMC_Single_Mode                     	((u8)0x01)
#define HMC_Standby_Mode                     	((u8)0x02)
#define HMC_Sleep_Mode                     		((u8)0x03)


#define HMC_I2C_ADDRESS         	0x3C
#define HMC_CTRL_REGA_ADDR     		0x00
#define HMC_CTRL_REGB_ADDR     		0x01
#define HMC_MODE_REG_ADDR     		0x02
#define HMC_STATUS_ADDR     		0x0C
//---------------------------------------------------------

#define HMC_XOUT_H_ADDR     		0x03
#define HMC_XOUT_L_ADDR     		0x04
#define HMC_YOUT_H_ADDR     		0x05
#define HMC_YOUT_L_ADDR    			0x06
#define HMC_ZOUT_H_ADDR    			0x07
#define HMC_ZOUT_L_ADDR    			0x08

#define HMC_XOFFSET -42
#define HMC_YOFFSET -9
#define HMC_ZOFFSET 0

typedef struct
{
  u8 Odr; 
  u8 FS;     /*!< Full Scale */
  u8 SampleMode;   
}HMC_ConfigTypeDef;

void HMC_I2C_Init(void);
void HMC_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);
void HMC_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void HMC_Init(HMC_ConfigTypeDef *HMC_Config_Struct);

void HMC_Read_RawData(s16* out);
void HMC_Read_Mag(float* out);

#endif /* __HMC_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

