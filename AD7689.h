#ifndef _AD7689_H_
#define _AD7689_H_

#include "stm32f10x.h"

#define AD7689_CFG_MASK		(0x0001<<15)
#define AD7689_CFG_UPDATE	(0x0001<<15)
#define AD7689_CFG_REMAIN	(0x0000<<15)

#define AD7689_INCC_MASK		(0x0007<<12)
#define AD7689_IC_BiDif2VREF	(0x0000<<12)
#define AD7689_IC_Bi2COM		(0x0002<<12)
#define AD7689_IC_TEMP			(0x0003<<12)
#define AD7689_IC_UniDif2GND	(0x0004<<12)
#define AD7689_IC_Uni2COM		(0x0006<<12)
#define AD7689_IC_Uni2GND		(0x0007<<12)

#define AD7689_INx_MASK		(0x0007<<9)
#define AD7689_IN0			(0x0000<<9)
#define AD7689_IN1			(0x0001<<9)
#define AD7689_IN2			(0x0002<<9)
#define AD7689_IN3			(0x0003<<9)
#define AD7689_IN4			(0x0004<<9)
#define AD7689_IN5			(0x0005<<9)
#define AD7689_IN6			(0x0006<<9)
#define AD7689_IN7			(0x0007<<9)

#define AD7689_LPFBW_MASK	(0x0001<<8)
#define AD7689_LPFBW_1_4	(0x0000<<8)
#define AD7689_LPFBW_1		(0x0001<<8)

#define AD7689_REFSEL_MASK		(0x0007<<5)
#define AD7689_REFSEL_EX_TDIS	(0x0007<<5)

#define AD7689_SEQ_MASK			(0x0003<<4)
#define AD7689_SEQ_DIS			(0x0000<<4)
#define AD7689_SEQ_SCAN_NO_TEMP	(0x0003<<4)

#define AD7689_RB_MASK		(0x0001<<2)
#define AD7689_RB_EN		(0x0000<<2)
#define AD7689_RB_DIS		(0x0001<<2)

#define GRAVITY 9.8015

void AD7689_SPI_Init(void);
u8 AD7689_SPI_SendByte(u8 byte);
u8 AD7689_SPI_ReadByte(void);

void AD7689_Read_Gyro_Raw(u16 *out);
void AD7689_Read_Gyro(float *out);
void AD7689_Read_All_Raw(u16 *out);
void AD7689_Read_Gyro_Acc(float *gyr,float *acc);

#endif
