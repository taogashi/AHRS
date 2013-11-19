#ifndef _LED_H_
#define _LED_H_

#include "stm32f10x.h"

#define LED_ON() GPIO_SetBits(GPIOA, GPIO_Pin_3)
#define LED_OFF() GPIO_ResetBits(GPIOA, GPIO_Pin_3)

void LED_Config(void);

#endif
