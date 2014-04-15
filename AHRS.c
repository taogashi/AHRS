#include "AHRS.h"
#include "OSConfig.h"
#include "spi.h"
#include <stdio.h>
#include <arm_math.h>

#include "LSM303DLH.h"
#include "LIS3LV02DQ.h"
#include "ITG3200.h"

#include "HAL_I2C.h"
#include "I2C.h"
#include "ledTask.h"
#include "calibration.h"

u8 i2c_sensor_raw[18];

xQueueHandle xAccCaliQueue;
xQueueHandle xEKFQueue;

IMUCaliType caliStructure;

ComType comt;

void vAHRSConfig(void* pvParameters)
{		
	Blinks(LED1, 3);
	/*I2C peripherial initialize*/
	User_I2C_Config();
	vTaskDelay((portTickType)200/portTICK_RATE_MS);

	/*sensors initialize*/
	AHRS_ENTER_CRITICAL();
	ITG3205_Config();
	AHRS_EXIT_CRITICAL();
	vTaskDelay((portTickType)10/portTICK_RATE_MS);

	AHRS_ENTER_CRITICAL();
	LSM_Config();
	AHRS_EXIT_CRITICAL();
	vTaskDelay((portTickType)10/portTICK_RATE_MS);

	LIS3_SPI_Init();
	AHRS_ENTER_CRITICAL();
	LIS3LV_Config();
	AHRS_EXIT_CRITICAL();	
	/*I2C interrupt config*/
	User_I2C_IT_Config();
	
	xTaskCreate(vAHRSCaliTask,(signed char *)"ahrs_cali"
			,configMINIMAL_STACK_SIZE+512
			,(void *)(&caliStructure),mainFLASH_TASK_PRIORITY+3
			,(xTaskHandle *)NULL);

	vTaskDelete(NULL);	
}

void vAHRSCaliTask(void* pvParameters)
{
	float acc[3]={0.0};
	float gyr[3]={0.0};
	s16 mag[3] = {0};
	float acczMean=0.0;
	
	u8 i;
	//read flash 15 float = 60 byte
	xQueueReceive(xAccCaliQueue,&caliStructure,portMAX_DELAY);
	
	//sync I2C read mechanism
	for(i=0; i<10; i++)
	{
		AHRS_Read_I2C_Gyr(gyr);
		vTaskDelay((portTickType)40/portTICK_RATE_MS);
		AHRS_Read_I2C_Acc(acc);
		vTaskDelay((portTickType)40/portTICK_RATE_MS);
		AHRS_Read_I2C_Mag(mag);
		vTaskDelay((portTickType)40/portTICK_RATE_MS);
	}
	//check acc
	//if invert ..
	AHRS_Read_Default_Acc(acc);
	acczMean=acc[2];

	for(i=2;i<20;i++)
	{
		AHRS_Read_Default_Acc(acc);
		acczMean=acczMean+(acc[2]-acczMean)/i;
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
	}
	
	if(acczMean > 3.0)
	{
		AHRSAccCali(&caliStructure);
	}
	
	//check acc
	//if flip ..
	AHRS_Read_Default_Acc(acc);
	acczMean=acc[2]/acc[0];

	for(i=2;i<20;i++)
	{
		AHRS_Read_Default_Acc(acc);
		acczMean=acczMean+(acc[2]/acc[0]-acczMean)/i;
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
	}
	
	if(acczMean > -0.3 && acczMean < 0.3)
	{
		AHRSMagCali(&caliStructure);
	}
	
	vTaskDelay((portTickType)2000/portTICK_RATE_MS);
	AHRSGyrCali(&caliStructure);
	
	switch(caliStructure.valid)
	{
		case 3://all calibrated
			break;
		case 2://only acc cali
			//cali mag here
			AHRSMagCali(&caliStructure);
			break;
		default:
			//should run calibration algorithm
			caliStructure.valid = 0;
			AHRSAccCali(&caliStructure);
			AHRSMagCali(&caliStructure);
			break;
	}
//	vTaskDelay((portTickType)2000/portTICK_RATE_MS);
//	caliStructure.valid = 0x03;
	xQueueSend(xAccCaliQueue,&caliStructure,portMAX_DELAY);

	xTaskCreate(vAHRSReadRaw
		    ,(signed char *)"ahrs_read"
				,configMINIMAL_STACK_SIZE+256
				,NULL,mainFLASH_TASK_PRIORITY+3
				,(xTaskHandle *)NULL);
	vTaskDelete(NULL);
}

void vAHRSReadRaw(void* pvParameters)
{
	//閫氫俊鏁版嵁
	SensorDataType sdt;
	u8 mag_cnt=0;
	u8 acc_cnt=0;

	//raw data
	s16 mag_s16[3];
	float acc[3];
	float gyr[3];
	
	//婊ゆ尝
	float lastGyr[3]={0.0};
	u8 errCNT[3]={0};
	
	//鍏朵粬
	float gama;
	float radii[3];
	float mag_bias[3];
	float mag_scale[3];

//	u8 CNT=0;
	u8 i;
	portTickType xLastReadTime;
	
	mag_bias[0] = -caliStructure.mag_ellipsolid_coef[3]/caliStructure.mag_ellipsolid_coef[0];
	mag_bias[1] = -caliStructure.mag_ellipsolid_coef[4]/caliStructure.mag_ellipsolid_coef[1];
	mag_bias[2] = -caliStructure.mag_ellipsolid_coef[5]/caliStructure.mag_ellipsolid_coef[2];
		
	gama = 1+(caliStructure.mag_ellipsolid_coef[3]*caliStructure.mag_ellipsolid_coef[3]/caliStructure.mag_ellipsolid_coef[0]
				+caliStructure.mag_ellipsolid_coef[4]*caliStructure.mag_ellipsolid_coef[4]/caliStructure.mag_ellipsolid_coef[1]
				+caliStructure.mag_ellipsolid_coef[5]*caliStructure.mag_ellipsolid_coef[5]/caliStructure.mag_ellipsolid_coef[2]);
	arm_sqrt_f32(gama/caliStructure.mag_ellipsolid_coef[0], &radii[0]);
	arm_sqrt_f32(gama/caliStructure.mag_ellipsolid_coef[1], &radii[1]);
	arm_sqrt_f32(gama/caliStructure.mag_ellipsolid_coef[2], &radii[2]);
		
	mag_scale[0] = 500.0/radii[0];
	mag_scale[1] = 500.0/radii[1];
	mag_scale[2] = 500.0/radii[2];

	Blinks(LED1,2);

	//纭欢閰嶇疆
	SPI_DMA_Config();
	SPI_DMA_IT_Config();
	
	Blinks(LED1,1);
	xLastReadTime=xTaskGetTickCount();
	for(;;)
	{
		AHRS_Read_I2C_Gyr(gyr);
		sdt.gyr[0] = (gyr[0]-caliStructure.gyr_bias[0])*caliStructure.gyr_scale[0];
		sdt.gyr[1] = (gyr[1]-caliStructure.gyr_bias[1])*caliStructure.gyr_scale[1];
		sdt.gyr[2] = (gyr[2]-caliStructure.gyr_bias[2])*caliStructure.gyr_scale[2];
		
		for(i=0;i<3;i++)
		{
			if(lastGyr[i]-sdt.gyr[i] > 0.35 || lastGyr[i]-sdt.gyr[i] < -0.35)
			{
				errCNT[i]++;
				if(errCNT[i]>=2)
				{ 
					lastGyr[i]=sdt.gyr[i];
					errCNT[i]=0;
				}
				else
				{
					sdt.gyr[i]=lastGyr[i];
				}
			} 
			else
			{
					lastGyr[i]=sdt.gyr[i];
					errCNT[i]=0;
			}
		}
		
		if(acc_cnt++>=5)
		{
			acc_cnt = 0;
			AHRS_Read_Default_Acc(acc);
			sdt.acc[0] = caliStructure.acc_coef[0]*(acc[0]-caliStructure.acc_bias[0])
						+ caliStructure.acc_coef[1]*(acc[1]-caliStructure.acc_bias[1])
						+ caliStructure.acc_coef[2]*(acc[2]-caliStructure.acc_bias[2]);
			sdt.acc[1] = caliStructure.acc_coef[3]*(acc[0]-caliStructure.acc_bias[0])
						+ caliStructure.acc_coef[4]*(acc[1]-caliStructure.acc_bias[1])
						+ caliStructure.acc_coef[5]*(acc[2]-caliStructure.acc_bias[2]);
			sdt.acc[2] = caliStructure.acc_coef[6]*(acc[0]-caliStructure.acc_bias[0])
						+ caliStructure.acc_coef[7]*(acc[1]-caliStructure.acc_bias[1])
						+ caliStructure.acc_coef[8]*(acc[2]-caliStructure.acc_bias[2]);
		}
		
		if(mag_cnt++>=20)
		{
			mag_cnt=0;
			AHRS_Read_I2C_Mag(mag_s16);
			sdt.mag[0] = (mag_s16[0] - mag_bias[0])*mag_scale[0];
			sdt.mag[1] = (mag_s16[1] - mag_bias[1])*mag_scale[1];
			sdt.mag[2] = (mag_s16[2] - mag_bias[2])*mag_scale[2];
		}

		xQueueSend(xEKFQueue, &sdt, 0);

		vTaskDelayUntil(&xLastReadTime,(portTickType)2/portTICK_RATE_MS);
	}
}


void AHRS_Read_Default_Acc(float *acc)
{
	AHRS_ENTER_CRITICAL();
	LIS3_Read_Acc(acc);
	AHRS_EXIT_CRITICAL();
}

void AHRS_Read_I2C_Acc(float *acc)
{	
	User_I2C_BufferRead(LSM_A_I2C_ADDRESS, i2c_sensor_raw, LSM_A_OUT_X_L_ADDR | 0x80, 6);
	LSM303DLH_Raw2Acc(i2c_sensor_raw+6, acc);
	vTaskDelay((portTickType)2/portTICK_RATE_MS);	
}

void AHRS_Read_I2C_Gyr(float *gyr)
{
	User_I2C_BufferRead(ITG_I2C_ADDRESS, i2c_sensor_raw, ITG_XOUT_H_ADDR, 6);
	ITG_Raw2Gyro(i2c_sensor_raw, gyr);
	vTaskDelay((portTickType)1/portTICK_RATE_MS);
}

void AHRS_Read_I2C_Mag(s16 *mag)
{
	User_I2C_BufferRead(LSM_M_I2C_ADDRESS, i2c_sensor_raw, LSM_M_OUT_X_H_ADDR | 0x80, 6);
	LSM303DLH_Raw2Mag(i2c_sensor_raw+12, mag);
	vTaskDelay((portTickType)1/portTICK_RATE_MS);
}


