#ifndef _LED_H_
#define _LED_H_

#include "stm32f10x.h"

#define LED1_ON() GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define LED1_OFF() GPIO_SetBits(GPIOA, GPIO_Pin_1)

#define LED2_ON() GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define LED2_OFF() GPIO_SetBits(GPIOA, GPIO_Pin_2)

void LED_Config(void);

#endif
