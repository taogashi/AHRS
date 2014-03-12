#ifndef _AHRSEKF_H_
#define _AHRSEKF_H_

#include "stm32f10x.h"
#include "OSConfig.h"

typedef struct
{
	s16 data[10];
	s32 check;
}AttComType;

extern xQueueHandle EKFToComQueue;
/*
 * user define function, varies from model to model
 */
void AHRS_GetA(float *A,void *para1,void *para2,void *para3, void *para4);
void AHRS_GetH(float *H,void *para1,void *para2,void *para3, void *para4);
void AHRS_aFunc(float *q,void *para1,void *para2,void *para3, void *para4);
void AHRS_hFunc(float *hx,void *para1,void *para2,void *para3, void *para4);

/*------------------------------tasks----------------------------------------*/
void vAEKFProcessTask(void* pvParameters);

void LoadAttData(u8 *buffer);

#endif
