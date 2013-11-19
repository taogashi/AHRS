#ifndef _I2C_H_
#define _I2C_H_

#include "stm32f10x.h"

#define I2C_BUFFER_LENGTH 20
#define  I2C_EVENT_UNEXPECTED                     ((uint32_t)0x00030041) 

void User_I2C_Config(void);
u8 User_I2C_ByteWrite(u8 slAddr, u8 data, u8 WriteAddr);
u8 User_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

void User_I2C_IT_Config(void);

#endif
