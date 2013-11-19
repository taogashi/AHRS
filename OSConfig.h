#ifndef _OSCONFIG_H_
#define _OSCONFIG_H_

#define FREERTOS

#ifdef FREERTOS
#include "freeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )

#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* The number of nano seconds between each processor clock. */
//#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/*-----------------------------------------------------------*/
#endif

#ifdef FREERTOS
#define AHRS_ENTER_CRITICAL()  taskENTER_CRITICAL()
#define AHRS_EXIT_CRITICAL()   taskEXIT_CRITICAL()
#endif

#endif
