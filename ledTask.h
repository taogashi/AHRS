#ifndef _LEDTASK_H_
#define _LEDTASK_H_

#include "OSConfig.h"

typedef enum{
	LED1,LED2
}LED_ID;

void vLED1Task( void *pvParameters );
void vLED2Task( void *pvParameters );

void Blinks(LED_ID id,unsigned char Hz);

#endif

