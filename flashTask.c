#include "flashTask.h"
#include "AHRS.h"
#include "calibration.h"

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

void vFlashTask(void* pvParameters)
{
	portBASE_TYPE xstatus;
	IMUCaliType act;
	u32 buffer[20];
	u8 i;
	for(i=0;i<20;i++)
	{
		buffer[i]=*((u32 *)OperationFlashAddr+i);
	}

	//读出已存的信息
	act=*(IMUCaliType *)buffer;
	xQueueSend(xAccCaliQueue,&act,portMAX_DELAY);
	//make sure that data has been read
	xstatus = xQueuePeek(xAccCaliQueue,&act,0);
	while(xstatus == pdPASS)
	{
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
		xstatus = xQueuePeek(xAccCaliQueue,&act,0);
	}

	for(;;)
	{
		//接收消息
		xQueueReceive(xAccCaliQueue, &act, portMAX_DELAY);
		*(IMUCaliType *)buffer = act;
		//解锁
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
		//擦除
		FLASHStatus=FLASH_ErasePage(OperationFlashAddr);
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
		//写入
		for(i=0;i<20;i++)
			FLASHStatus=FLASH_ProgramWord(OperationFlashAddr+i*sizeof(u32),buffer[i]);
		//上锁
		FLASH_Lock();
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
}

