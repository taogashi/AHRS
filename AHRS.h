#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f10x.h"
#include "OSConfig.h"

#define GRAVITY 9.8015

extern xQueueHandle xAccCaliQueue;
extern xQueueHandle baroQueue;
extern xQueueHandle xEKFQueue;

typedef struct
{
	float gyr[3];
	float acc[3];
	s16	  mag[3];
}SensorDataType;

typedef struct{
	s16 data[9];//gyr[3],acc[3],mag[3]
	float height;
	s32 Check;//Check=sum(*(u8 *)data)
}ComType;

typedef struct{
	s16 baroheight;
	u8 check;
}BaroHeightType;

__inline void AHRS_Read_IMU(float *gyr,float *acc);
__inline void AHRS_Read_Mag(s16 *mag);

void vAHRSConfig(void* pvParameters);
void vAHRSCaliTask(void* pvParameters);

void vAHRSReadRaw(void* pvParameters);
void vAHRSReadBaroHeight(void* pvParameters);


#endif
