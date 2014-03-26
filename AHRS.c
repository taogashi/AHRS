#include "AHRS.h"

#include <stdio.h>
#include <arm_math.h>

#include "mag3110.h"
#include "AD7689.h"
#include "MS5607B.h"

#include "I2C.h"
#include "spi.h"
#include "ledTask.h"
#include "calibration.h"
//#include "filter.h"
//#include "axisTrans.h"

xQueueHandle xAccCaliQueue;
xQueueHandle baroQueue;
xQueueHandle xEKFQueue;

IMUCaliType caliStructure;

ComType comt;
BaroHeightType bht;

void vAHRSConfig(void* pvParameters)
{	
	MS5607B_SPI_Init();
	AD7689_SPI_Init();
	vTaskDelay((portTickType)200/portTICK_RATE_MS);

	User_I2C_Config();
	vTaskDelay((portTickType)200/portTICK_RATE_MS);

	MAG_Config();	
	User_I2C_IT_Config();

	xTaskCreate(vAHRSCaliTask,(signed char *)"ahrs_cali"
				,configMINIMAL_STACK_SIZE+1024
				,(void *)(&caliStructure),mainFLASH_TASK_PRIORITY+3
				,(xTaskHandle *)NULL);
	vTaskDelete(NULL);	
}

void vAHRSCaliTask(void* pvParameters)
{
	float acc[3]={0.0};
	float gyr[3]={0.0};
	float acczMean=0.0;
	
	u8 i;
	//read flash 15 float = 60 byte
	xQueueReceive(xAccCaliQueue,&caliStructure,portMAX_DELAY);
	
	//check acc
	//if invert ..
	AHRS_Read_IMU(gyr,acc);
	acczMean=acc[2];

	for(i=2;i<20;i++)
	{
		AHRS_Read_IMU(gyr,acc);
		acczMean=acczMean+(acc[2]-acczMean)/i;
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
	}
	
	if(acczMean > 3.0)
	{
		Blinks(LED2, 1);
		AHRSAccCali(&caliStructure);
	}
	
	//check acc
	//if flip ..
	AHRS_Read_IMU(gyr,acc);
	acczMean=acc[2]/acc[0];

	for(i=2;i<20;i++)
	{
		AHRS_Read_IMU(gyr,acc);
		acczMean=acczMean+(acc[2]/acc[0]-acczMean)/i;
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
	}
	
	if(acczMean > -0.3 && acczMean < 0.3)
	{
		Blinks(LED2, 2);
		AHRSMagCali(&caliStructure);
	}
	
	vTaskDelay((portTickType)2000/portTICK_RATE_MS);
	AHRSGyrCali(&caliStructure);
	
	switch(caliStructure.valid)
	{
		case 3://all calibrated
			Blinks(LED2,0);
			break;
		case 2://only acc cali
			Blinks(LED2,2);
			//cali mag here
			AHRSMagCali(&caliStructure);
			break;
		default:
			//should run calibration algorithm
			Blinks(LED2,1);
			caliStructure.valid = 0;
			AHRSAccCali(&caliStructure);
			Blinks(LED2,2);
			AHRSMagCali(&caliStructure);
			break;
	}
	
	xQueueSend(xAccCaliQueue,&caliStructure,portMAX_DELAY);

	Blinks(LED2, 0);
	xTaskCreate(vAHRSReadRaw
		    ,(signed char *)"ahrs_read"
				,configMINIMAL_STACK_SIZE+256
				,NULL,mainFLASH_TASK_PRIORITY+3
				,(xTaskHandle *)NULL);
	xTaskCreate(vAHRSReadBaroHeight
		            ,(signed char *)"baro"
					,configMINIMAL_STACK_SIZE+128
					,NULL
					,mainFLASH_TASK_PRIORITY+3
					,(xTaskHandle *)NULL);
	vTaskDelete(NULL);
}

void vAHRSReadRaw(void* pvParameters)
{
	//通信数据
	SensorDataType sdt;
	u8 CNT=0;

	//raw data
	u8 mag_raw[6];
	float acc[3];
	float gyr[3];
	
	//滤波
	float lastGyr[3]={0.0};
	u8 errCNT[3]={0};
	
	//其他

//	u8 CNT=0;
	u8 i;
	portTickType xLastReadTime;

	Blinks(LED1,2);

	//硬件配置
	SPI_DMA_Config();
	SPI_DMA_IT_Config();
	
	Blinks(LED1,1);
	xLastReadTime=xTaskGetTickCount();
	for(;;)
	{
		AHRS_Read_IMU(gyr,acc);
		
		sdt.acc[0] = caliStructure.acc_coef[0]*(acc[0]-caliStructure.acc_bias[0])
					+ caliStructure.acc_coef[1]*(acc[1]-caliStructure.acc_bias[1])
					+ caliStructure.acc_coef[2]*(acc[2]-caliStructure.acc_bias[2]);
		sdt.acc[1] = caliStructure.acc_coef[3]*(acc[0]-caliStructure.acc_bias[0])
					+ caliStructure.acc_coef[4]*(acc[1]-caliStructure.acc_bias[1])
					+ caliStructure.acc_coef[5]*(acc[2]-caliStructure.acc_bias[2]);
		sdt.acc[2] = caliStructure.acc_coef[6]*(acc[0]-caliStructure.acc_bias[0])
					+ caliStructure.acc_coef[7]*(acc[1]-caliStructure.acc_bias[1])
					+ caliStructure.acc_coef[8]*(acc[2]-caliStructure.acc_bias[2]);

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

		if(CNT++>20)
		{
			CNT=0;
			User_I2C_BufferRead(MAG3110_ADDR, mag_raw, MAG3110_OUT_X_MSB_REG, 6);
			MAG3110_Raw2Mag(mag_raw, sdt.mag);
		}

//		xQueueReceive(baroQueue, &comt.height, 0);
//		for(i=0;i<3;i++) comt.data[i]=(s16)(sdt.gyr[i]*4000.0);
//		for(i=0;i<3;i++) comt.data[i+3]=(s16)(sdt.acc[i]*1000.0);
//		for(i=0;i<3;i++) comt.data[i+6]=sdt.mag[i];
//		comt.Check=0;
//		for(i=0;i<9;i++) 
//			comt.Check+=comt.data[i];
//		
//		buffer_lock_global = 1;
//		LoadRawData(spi_mid_buffer);
//		buffer_lock_global = 0;
			
		xQueueSend(xEKFQueue, &sdt, 0);

		vTaskDelayUntil(&xLastReadTime,(portTickType)2/portTICK_RATE_MS);
	}
}

void vAHRSReadBaroHeight(void* pvParameters)
{
	u8 i=0;
	MS5607B_CaliData caliStructre;
	MS5607B_ProcData midVal;
	uint32_t D1;
	uint32_t D2;
	
	double temperature;
	double pressure_orig;
	double pressure_cur;
	float height;
	
	MS5607B_Reset();
	vTaskDelay((portTickType)5/portTICK_RATE_MS);
	MS5607B_GetCaliData(&caliStructre);
	
	/* initialize */
	while(i++ < 250)
	{
		MS5607B_StartTemperatureADC(OSR_4096);
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
		MS5607B_ReadADC(&D2);
		temperature = (double)MS5607B_GetTemperature(&midVal, D2, &caliStructre);
		MS5607B_StartPressureADC(OSR_4096);
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
		MS5607B_ReadADC(&D1);
		pressure_orig = (double)MS5607B_GetPressure(&midVal, D1, &caliStructre);
	}
	
	for(;;)
	{
		MS5607B_StartTemperatureADC(OSR_4096);
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
		MS5607B_ReadADC(&D2);
		temperature = 0.99*temperature + 0.01*MS5607B_GetTemperature(&midVal, D2, &caliStructre);
		MS5607B_StartPressureADC(OSR_4096);
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
		MS5607B_ReadADC(&D1);
		pressure_cur = MS5607B_GetPressure(&midVal, D1, &caliStructre);
		
		height = 18400*(1+0.00003663*temperature)*0.4343*(pressure_orig/pressure_cur - 1);
		xQueueSend(baroQueue, &height, 0);
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
}

//void BMA_Config(void)
//{
//	BMA180_ConfigTypeDef  BMA180_InitStructure;
//	BMA180_InitStructure.range=RANGE_2G;
//	BMA180_InitStructure.bw=BW_600Hz;
//	BMA180_InitStructure.mode=MODE_LOW_NOISE;
//	BMA180_Init(&BMA180_InitStructure);
//}

__inline void AHRS_Read_IMU(float *gyr,float *acc)
{
	AHRS_ENTER_CRITICAL();
	AD7689_Read_Gyro_Acc(gyr,acc);
	AHRS_EXIT_CRITICAL();
}

__inline void AHRS_Read_Mag(s16 *mag)
{
	AHRS_ENTER_CRITICAL();
	MAG3110_Read_Mag(mag);
	AHRS_EXIT_CRITICAL();
}


