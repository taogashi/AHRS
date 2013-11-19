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
#ifndef __LIS3LV02DQ_H_
#define __LIS3LV02DQ_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "HAL_LIS3LV02DQ.h"
#include "stm32f10x.h"

/**
* @addtogroup LIS3LV02DQ
* @{
*/
#define LIS_USE_SPI

#define LIS3_Device_On                   		((u8)0xC0)
#define LIS3_Power_Down							((u8)0x00)
#define LIS3_SR_40Hz                      		((u8)0x00)
#define LIS3_SR_160Hz                      		((u8)0x10)
#define LIS3_SR_640Hz                      		((u8)0x20)
#define LIS3_SR_2560Hz                       	((u8)0x30)
#define LIS3_Self_Test_Enable					((u8)0x08)
#define LIS3_Self_Test_Disable					((u8)0x00)
#define LIS3_Xaxis_Enable						((u8)0x01)	
#define LIS3_Yaxis_Enable						((u8)0x02)
#define LIS3_Zaxis_Enable						((u8)0x04)

#define LIS3_Full_Scale_2g                   	((u8)0x00)
#define LIS3_Full_Scale_6g                   	((u8)0x80)
#define LIS3_BDU_Continuous                   	((u8)0x00)
#define LIS3_Little_Endian                   	((u8)0x00)
#define LIS3_Reboot_EN                   	    ((u8)0x10)
#define LIS3_Reboot_Dis                   	    ((u8)0x00)
#define LIS3_Int_Enable                   		((u8)0x08)
#define LIS3_DRDY_Enable                   		((u8)0x04)

#define CTRL_REG3_Content						((u8)0x00)	  //ÔÝÊ±²»¹Ü


#define LIS3_I2C_ADDRESS         		0x3A
#define LIS3_WHO_AM_I_ADDR				0x0f
#define LIS3_CTRL_REG1_ADDR     		0x20
#define LIS3_CTRL_REG2_ADDR     		0x21
#define LIS3_CTRL_REG3_ADDR     		0x22

//---------------------------------------------------------
#define LIS3_OFFSETX_ADDR     			0x16
#define LIS3_OFFSETY_ADDR     			0x17
#define LIS3_OFFSETZ_ADDR     			0x18
#define LIS3_GAINX_ADDR     			0x19
#define LIS3_GAINY_ADDR     			0x1A
#define LIS3_GAINZ_ADDR     			0x1B

#define LIS3_XOUT_L_ADDR     			0x28
#define LIS3_XOUT_H_ADDR     			0x29
#define LIS3_YOUT_L_ADDR     			0x2A
#define LIS3_YOUT_H_ADDR     			0x2B
#define LIS3_ZOUT_L_ADDR     			0x2C
#define LIS3_ZOUT_H_ADDR     			0x2D

#define LIS3_XOFFSET 20
#define LIS3_YOFFSET -37
#define LIS3_ZOFFSET 28

typedef struct
{
  u8 powerMode; 
  u8 SamRate; /*!< sample Rate */
  u8 SelfTestStatus;
  u8 EnabledAxis;
  u8 FS;     /*!< Full Scale */
  u8 RebootStatus;  
  u8 IntMode;   
}LIS3_ConfigTypeDef;

void LIS3_I2C_Init(void);
void LIS3_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);
void LIS3_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void LIS3_Init(LIS3_ConfigTypeDef *LIS3_Config_Struct);

void LIS3_SPI_Init(void);
u8 LIS3_SPI_SendByte(u8 byte);
u8 LIS3_SPI_ReadByte(void);
void LIS3_SPI_ByteWrite(u8 data, u8 WriteAddr);
void LIS3_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

u8 LIS3_Read_PID(void);
void LIS3_Reboot(char* offset,char* gain);
u8 LIS3_Read_RawData(s16* out);
void LIS3_Read_Acc(float* out);

void LIS3LV_Config(void);
/**
 * @} 
 */  /* end of group ITG303DLH */

#endif /* __LIS3_H */

/******************* (C) COPYRIGHT 2013 Skyworks *****END OF FILE****/

