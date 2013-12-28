#ifndef _DEVICEINIT_H_
#define _DEVICEINIT_H_

#include "stm32f10x.h"

#define LED_ON() GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define LED_OFF() GPIO_ResetBits(GPIOB, GPIO_Pin_5)

void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	/* Configure PC.06, PC.07, PC.08 and PC.09 as output push-pull */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
}

#endif
