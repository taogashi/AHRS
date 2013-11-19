//与平台无关
//宏定义传感器I2C地址、内部寄存器地址、各种配置参数以及函数的声明
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MXR6202_H_
#define _MXR6202_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "HAL_MXR6202.h"

/**
* @addtogroup MXR6202
* @{
*/

#define MXR_I2C_ADDRESS         		0x20

#define MXR_ONLY_REG_ADDR     			0x00

#define MXR_X_MSB_REG_ADDR				0x01

typedef struct
{
	u8 byte;
}MXR_ConfigTypeDef;

void MXR_I2C_Init(void);	 //初始化需要用到的I2C
void MXR_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);	//向指定器件的指定寄存器写字节
void MXR_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);		//读取
void MXR_Init(MXR_ConfigTypeDef *MXR_Config_Struct);		  //传感器的初始化

u8 MXR_Read_RawData(u16* out);	   //读取原始测量数据
void MXR_Read_Acc(float* out);	   //读取经过标准化的数据，即单位制转换、去除零漂、交换坐标轴

/**
 * @} 
 */  /* end of group MXR303DLH */

#endif /* __MXR_H */

/******************* (C) COPYRIGHT 2013 Skyworks Embedded System*****END OF FILE****/

