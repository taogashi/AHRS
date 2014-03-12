#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f10x.h"
#include "OSConfig.h"

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

typedef struct{
	float acc_coef[9];
	float acc_bias[3];
	float gyr_scale[3];
	float gyr_bias[3];
	uint8_t valid;
}IMUCaliType;

void vAHRSConfig(void* pvParameters);
void vAHRSCaliTask(void* pvParameters);

void AHRSAccCali(IMUCaliType *ict);
void AHRSGyrCali(IMUCaliType *ict);

void vAHRSReadRaw(void* pvParameters);
void vAHRSReadBaroHeight(void* pvParameters);

void LoadRawData(u8 *buffer);
void LoadBaroData(u8 *buffer);


#endif
