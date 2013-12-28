#include "I2Ctest.h"
#include "I2C.h"
#include "HAL_I2C.h"
#include "mag3110.h"

void vI2CTest(void* pvParameters)
{
	u8 mag_id=0;
	User_I2C_Config();
	User_I2C_IT_Config();
	
	for(;;)
	{
		User_I2C_BufferRead(MAG3110_ADDR, &mag_id, MAG3110_WHO_AM_I_REG, 1);
		vTaskDelay((portTickType)200/portTICK_RATE_MS);
//		mag_id=0;
	}
//	u8 mag_id=0;
//	MAG3110_I2C_Init();
//	for(;;)
//	{
//		MAG3110_I2C_BufferRead(MAG3110_ADDR, &mag_id, MAG3110_WHO_AM_I_REG, 1);
//		vTaskDelay((portTickType)20/portTICK_RATE_MS);
//	}
}
