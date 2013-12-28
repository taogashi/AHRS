#include "flight.h"
#include "basictask.h"
#include "pwm.h"
#include <stdio.h>

void vFlightInit(void* pvParameters)
{
	PWMWidth pw;
	portBASE_TYPE xStatus;

	TIM2_Config();
	TIM2_IT_Config();
	TIM3_Config();
	TIM4_Config();
	TIM4_IT_Config();

	for(;;)
	{
		xStatus=xQueueReceive(xTIMQueue,&pw,portMAX_DELAY);	// 0   (portTickType)500/portTICK_RATE_MS
		vTaskDelay(100/portTICK_RATE_MS);
		if(xStatus==pdTRUE)
		{
			switch(pw.ch)
			{
				case TIM2Channel3:
					printf("tim2Channel3 %d\r\n",pw.width);
					break;
				case TIM2Channel4:
					printf("tim2Channel4 %d\r\n",pw.width);
					break;
				case TIM4Channel1:
					printf("tim4Channel1 %d\r\n",pw.width);
					break;
				case TIM4Channel2:
					printf("tim4Channel2 %d\r\n",pw.width);
					break;
				case TIM4Channel3:
					printf("tim4Channel3 %d\r\n",pw.width);
					break;
				case TIM4Channel4:
					printf("tim4Channel4 %d\r\n",pw.width);
					break;
				default:
					break;
			}
		} 
	}	
}
