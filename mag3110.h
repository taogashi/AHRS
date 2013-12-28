#ifndef _MAG3110_H_
#define _MAG3110_H_

#include "stm32f10x.h"

#define MAG3110_ADDR	0x1C

//register address
#define MAG3110_DR_STATUS_REG	0x00
#define	MAG3110_OUT_X_MSB_REG	0x01
#define	MAG3110_OUT_X_LSB_REG	0x02
#define	MAG3110_OUT_Y_MSB_REG	0x03
#define	MAG3110_OUT_Y_LSB_REG	0x04
#define	MAG3110_OUT_Z_MSB_REG	0x05
#define	MAG3110_OUT_Z_LSB_REG	0x06
#define MAG3110_WHO_AM_I_REG	0x07
#define MAG3110_SYSMOD		0x08
#define MAG3110_OFF_X_MSB	0x09
#define MAG3110_OFF_X_LSB	0x0A
#define MAG3110_OFF_Y_MSB	0x0B
#define MAG3110_OFF_Y_LSB	0x0C
#define MAG3110_OFF_Z_MSB	0x0D
#define MAG3110_OFF_Z_LSB	0x0E
#define MAG3110_DIE_TEMP	0x0F

#define MAG3110_CTRL_REG1_ADDR  0x10
#define MAG3110_CTRL_REG2_ADDR  0x11

#define MAG3110_ODR_80Hz	0x00
#define MAG3110_ODR_40Hz	0x01
#define MAG3110_ODR_20Hz	0x02
#define MAG3110_ODR_10Hz	0x03
#define MAG3110_ODR_5Hz		0x04
#define MAG3110_ODR_2_5Hz	0x05
#define MAG3110_ODR_1_25Hz	0x06
#define MAG3110_ODR_0_63Hz	0x07

#define MAG3110_OSR_16		0x00
#define MAG3110_OSR_32		0x01
#define MAG3110_OSR_64		0x02
#define MAG3110_OSR_128		0x03

#define MAG3110_FAST_READ_EN	0x01
#define MAG3110_FAST_READ_DIS	0x00

#define MAG3110_TRIG_MEAS_EN	0x01
#define MAG3110_TRIG_MEAS_DIS	0x00

#define MAG3110_MODE_NORM	0x01
#define MAG3110_MODE_STANDBY	0x00

#define MAG3110_CTRL_REG2_BYTE	0x00

typedef struct
{
	u8 sampleRate;
	u8 overSampleRatio;
	u8 fastReadMode;
	u8 triggerMeasMode;
	u8 powerMode;
}MAG3110_ConfigTypeDef;

void MAG3110_I2C_Init(void);
void MAG3110_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);
void MAG3110_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void MAG3110_Init(MAG3110_ConfigTypeDef *MAG3110_Config_Struct);

u8 MAG3110_Read_PID(void);
void MAG3110_Read_RawData(s16* out);
void MAG3110_Read_Mag(s16* out);
void MAG3110_Raw2Mag(u8 *raw, s16 *mag);

void MAG_Config(void);

#endif

