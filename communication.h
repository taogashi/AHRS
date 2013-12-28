#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "stm32f10x.h"

typedef struct{
	s16 data[10];//gyr[3],acc[3],mag[3],baro height	
	s16 Check;//Check=sum(*(u8 *)data)
}ComType;

#endif
