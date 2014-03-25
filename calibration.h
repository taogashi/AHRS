#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include "stm32f10x.h"

typedef struct{
	float acc_coef[9];
	float acc_bias[3];
	float mag_scale[3];
	float mag_bias[3];
	float gyr_scale[3];
	float gyr_bias[3];
	uint8_t valid;
}IMUCaliType;

void AHRSAccCali(IMUCaliType *ict);
void AHRSGyrCali(IMUCaliType *ict);
void AHRSMagCali(IMUCaliType *ict);

#endif
