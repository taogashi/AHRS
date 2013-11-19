#include "ledTask.h"
#include "led.h"

u8 flashHz[1]={1};

void vLED1Task( void *pvParameters )
{
	portTickType xlastFlashTime;
	u8 i;

	LED_Config();
	xlastFlashTime=xTaskGetTickCount();
	for(;;)
	{
		vTaskDelayUntil(&xlastFlashTime,(portTickType)(1000/portTICK_RATE_MS));
		LED_ON();
		for(i=0;i<flashHz[0];i++)
		{
			LED_ON();
			vTaskDelay((portTickType)(30/portTICK_RATE_MS));
			LED_OFF();
			vTaskDelay((portTickType)(120/portTICK_RATE_MS));
		}
	}
}

void Blinks(LED_ID id,u8 Hz)
{
	switch(id)
	{
		case LED1:
			flashHz[0] = Hz;
			break;
		default:
			break;
	}
}
