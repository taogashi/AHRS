#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f10x.h"
#include "OSConfig.h"

#define GRAVITY 9.8015
#define AHRS_Read_Default_Acc AHRS_Read_SPI_Acc

extern xQueueHandle xAccCaliQueue;
extern xQueueHandle xEKFQueue;

typedef struct
{
	float gyr[3];
	float acc[3];
	s16	  mag[3];
}SensorDataType;

typedef struct{
	s16 data[9];//gyr[3],acc[3],mag[3]
	s32 Check;//Check=sum(*(u8 *)data)
}ComType;

void AHRS_Read_I2C_Acc(float *acc);
void AHRS_Read_I2C_Mag(s16 *mag);
void AHRS_Read_I2C_Gyr(float *gyr);
void AHRS_Read_SPI_Acc(float *gyr);

void vAHRSConfig(void* pvParameters);
void vAHRSCaliTask(void* pvParameters);

void vAHRSReadRaw(void* pvParameters);

#endif
