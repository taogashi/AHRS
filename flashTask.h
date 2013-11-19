#ifndef _FLASHTASK_H_
#define _FLASHTASK_H_

#include "stm32f10x.h"
#include "OSConfig.h"

#define OperationFlashAddr ((uint32_t)0x0800F800)

#define PageSize ((uint16_t)0x400)
#define MaxSize ((uint16_t)0x800)

void vFlashTask(void* pvParameters);

#endif
