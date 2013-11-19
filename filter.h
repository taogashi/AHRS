#ifndef _FILTER_H_
#define _FILTER_H_

#include "stm32f10x.h"

typedef struct{
float pool[10];
u8 length;
u8 head;
} GFilterType;

extern const float GCoef[5][10];
float GaussianFilter(GFilterType *gft,float newData);

#endif
