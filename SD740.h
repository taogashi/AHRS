#ifndef _SD740_H_
#define _SD740_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include "HAL_SD740.h"

#define SD740_SPI_CS_LOW()  GPIO_ResetBits(SD740_SPI_NSS_Pin_Port, SD740_SPI_NSS_Pin)
#define SD740_SPI_CS_HIGH() GPIO_SetBits(SD740_SPI_NSS_Pin_Port, SD740_SPI_NSS_Pin)

#define SD740_RATE_X_H ((u8)0x00)
#define SD740_RATE_X_L ((u8)0x01)

#define SD740_RATE_Y_H ((u8)0x02)
#define SD740_RATE_Y_L ((u8)0x03)

#define SD740_RATE_Z_H ((u8)0x04)
#define SD740_RATE_Z_L ((u8)0x05)

#define SD740_TEMP_REG ((u8)0x15)

#define SD740_STANDBY_MODE_REG 	((u8)0x46)
#define SD740_DEVICE_STATE_REG 	((u8)0x14)

#define SD740_ID0_REG			((u8)0x47)
#define SD740_ID1_REG			((u8)0x48)
#define SD740_ID2_REG			((u8)0x49)

#define SD740_CONF_REG			((u8)0x4A)

//-----------------------------------------------
#define SD740_Normal_Mode	((u8)0x00)
#define SD740_Standby_Mode	((u8)0x01)

#define SD740_CONF_REG_Content ((u8)0xff)

typedef struct
{
	u8 mode;
	u8 confContent;
}SD740_ConfigTypeDef;

void SD740_SPI_Init(void);
u8 SD740_SPI_SendByte(u8 byte);
u8 SD740_SPI_ReadByte(void);
void SD740_SPI_ByteWrite(u8 data, u8 WriteAddr);
void SD740_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void SD740_Init(SD740_ConfigTypeDef *SD740_Config_Struct);
void SD740_Reset(void);

u8 SD740_Read_PID(u8 ID_No);
u8 SD740_Read_State(void);
void SD740_Read_RawData(s16* out);
void SD740_Read_GyroRate(float* out);

void SD740_Delay(u32 n);

#endif
