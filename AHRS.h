#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f10x.h"
#include "OSConfig.h"

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

typedef struct{
	s16 baroheight;
	u8 check;
}BaroHeightType;

typedef struct{
	float K[12];
}AccCaliType;

void vAHRSConfig(void* pvParameters);
void vAHRSCali(void* pvParameters);
void vAHRSReadRaw(void* pvParameters);
void vAHRSReadBaroHeight(void* pvParameters);

void LoadRawData(u8 *buffer);
void LoadBaroData(u8 *buffer);


#endif
