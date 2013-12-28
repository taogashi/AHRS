#ifndef BMA180_H_
#define BMA180_H_

#include "stm32f10x.h"

/*
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
*/
typedef enum{
	ACCELERATION_X = 0,
	ACCELERATION_Y,
	ACCELERATION_Z,
	TEMPERATURE
}TAxe;

//====================//
// ADDRESS REGISTER
#define ID_ADDR 					0x00
#define VERSION_ADDR 				0x01
#define ACCXLSB_ADDR				0x02
#define ACCXMSB_ADDR 				0x03
#define ACCYLSB_ADDR 				0x04
#define ACCYMSB_ADDR 				0x05
#define ACCZLSB_ADDR 				0x06
#define ACCZMSB_ADDR 				0x07
#define TEMPERATURE 				0x08
#define STATREG1 					0x09
#define STATREG2 					0x0A
#define STATREG3 					0x0B
#define STATREG4 					0x0C
#define CTRL_REG0_ADDR 				0x0D
#define CTRL_REG1_ADDR 				0x0E
#define CTRL_REG2_ADDR 				0x0F
#define RESET_ADDR 					0x10
#define BW_TCS_ADDR	        		0x20
#define CTRL_REG3_ADDR				0x21
#define CTRL_REG4_ADDR				0x22
#define HY							0x23
#define SLOPE_TAPSENS_INFO_ADDR 	0x24
#define HIGH_LOW_INFO_ADDR   		0x25
#define LOW_DUR_ADDR         		0x26
#define HIGH_DUR_ADDR         		0x27
#define TAPSENSTH_ADDR         		0x28
#define LOWTH_ADDR					0x29
#define TCO_Y_ADDR 					0x2F
#define TCO_Z_ADDR					0x30


#define EE_OFFSET_LSB1_ADDR			0x35

//====================//
// MASK
#define EE_W					0x10
#define UPDATE_IMG				0x20

#define RANGE_1G				0x00
#define RANGE_1_5G				0x02
#define RANGE_2G				0x04
#define RANGE_3G				0x06
#define RANGE_4G				0x08
#define RANGE_8G				0x0A
#define RANGE_16G				0x0C

#define BW_10Hz			0x00
#define BW_20Hz			0x01
#define BW_40Hz			0x02
#define BW_75Hz			0x03
#define BW_150Hz			0x04
#define BW_300Hz			0x05
#define BW_600Hz			0x06
#define BW_1200Hz			0x07

#define MODE_LOW_NOISE		0x00//main mode
#define MODE_ULTRA_LOW_NOISE	0x01
#define MODE_REDUCE_POWER	0x02
#define MODE_LOWER_POWER	0x03

#define DATA_12BIT		0x01
#define DATA_14BIT		0x00

#define DIS_I2C_MASK         	0x01

#define RESET_VALUE				0xB6  // soft reset

//Range setting
#define RANGESHIFT 1
#define RANGE_MASK 0x0E
#define BWMASK 0xF0
#define BWSHIFT 4

typedef struct
{
	u8 range;
	u8 bw;
	u8 mode;
}BMA180_ConfigTypeDef;
/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void BMA180_SPI_Init(void);
u8 BMA180_SPI_SendByte(u8 byte);
u8 BMA180_SPI_ReadByte(void);
void BMA180_SPI_ByteWrite(u8 data, u8 WriteAddr);
void BMA180_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void BMA180_Init(BMA180_ConfigTypeDef *BMA180_Config_Struct);

u8 BMA180_Read_PID(void);
void BMA180_SoftReset(void);
void BMA180_Read_RawData(s16* out);
void BMA180_Read_Acc(float* out);

short Accel_get(TAxe axe);

short Accel_getX(void);
short Accel_getY(void);
short Accel_getZ(void);

#endif /* BMA180_H_ */

