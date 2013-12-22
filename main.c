/* Standard includes. */
#include <stdio.h>

#include "AHRS.h"
#include "OSConfig.h"

/* Library includes. */
#include "spi.h"
#include "ledTask.h"
#include "flashTask.h"
#include "AHRSEKF.h"

//#include "flash.h"

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware( void );

int main( void )
{
#ifdef DEBUG
  debug();
#endif

	prvSetupHardware();

	xAccCaliQueue = xQueueCreate(1,sizeof(AccCaliType));
	xEKFQueue = xQueueCreate(1,sizeof(SensorDataType));
	EKFToComQueue = xQueueCreate(1,sizeof(AttComType));
//	xBaroQueue=xQueueCreate(1,sizeof(baroComType));     
																	   	
		xTaskCreate(vLED1Task
					,(signed char *)"Led flash"
					,configMINIMAL_STACK_SIZE
					,NULL
					,mainFLASH_TASK_PRIORITY
					,(xTaskHandle *)NULL);

		xTaskCreate(vFlashTask
		            ,(signed char *)"flash"
					,configMINIMAL_STACK_SIZE+128
					,NULL
					,mainFLASH_TASK_PRIORITY+2
					,(xTaskHandle *)NULL);
//		xTaskCreate(vI2CTest
//		            ,(signed char *)"i2c"
//					,configMINIMAL_STACK_SIZE+128
//					,NULL
//					,mainFLASH_TASK_PRIORITY+2
//					,(xTaskHandle *)NULL);
		xTaskCreate(vAHRSConfig
		            ,(signed char *)"ahrs_config"
					,configMINIMAL_STACK_SIZE+128
					,NULL
					,mainFLASH_TASK_PRIORITY+3
					,(xTaskHandle *)NULL);
//		xTaskCreate(vAEKFProcessTask
//		            ,(signed char *)"ahrs_ekf"
//					,configMINIMAL_STACK_SIZE+256
//					,NULL
//					,mainFLASH_TASK_PRIORITY+2
//					,(xTaskHandle *)NULL);
//		xTaskCreate(vAHRSReadBaroHeight
//		            ,(signed char *)"baro"
//					,configMINIMAL_STACK_SIZE+128
//					,NULL
//					,mainFLASH_TASK_PRIORITY+1
//					,(xTaskHandle *)NULL);
		
		/* Start the scheduler. */
		vTaskStartScheduler();

	for(;;);
	return 0;
}

void vApplicationIdleHook(void)
{
//	printf(".");
}

static void prvSetupHardware( void )
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );	
//	Sensor_Power_Config();
//	SENSOR_POWER_ON();
//	USART_Config();
}

#ifdef  DEBUG
/* Keep the linker happy. */
void assert_failed( unsigned portCHAR* pcFile, unsigned portLONG ulLine )
{
	for( ;; )
	{
	}
}
#endif
