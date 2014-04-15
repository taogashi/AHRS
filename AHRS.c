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



u8 sensor_raw[18];

xQueueHandle xAccCaliQueue;
xQueueHandle xEKFQueue;

AccCaliType accCaliStructure;

ComType comt;

__inline void AHRS_Read_SPI_Acc(float *acc)
{
	AHRS_ENTER_CRITICAL();
	LIS3_Read_Acc(acc);
	AHRS_EXIT_CRITICAL();
}

__inline void AHRS_Read_I2C_Acc(float *acc)
{	
	User_I2C_BufferRead(LSM_A_I2C_ADDRESS, sensor_raw, LSM_A_OUT_X_L_ADDR | 0x80, 6);
	LSM303DLH_Raw2Acc(sensor_raw+6, acc);
	vTaskDelay((portTickType)2/portTICK_RATE_MS);	
}

__inline void AHRS_Read_I2C_Gyr(float *gyr)
{
	User_I2C_BufferRead(ITG_I2C_ADDRESS, sensor_raw, ITG_XOUT_H_ADDR, 6);
	ITG_Raw2Gyro(sensor_raw, gyr);
	vTaskDelay((portTickType)1/portTICK_RATE_MS);
}

__inline void AHRS_Read_I2C_Mag(s16 *mag)
{
	User_I2C_BufferRead(LSM_M_I2C_ADDRESS, sensor_raw, LSM_M_OUT_X_H_ADDR | 0x80, 6);
	LSM303DLH_Raw2Mag(sensor_raw+12, mag);
	vTaskDelay((portTickType)1/portTICK_RATE_MS);
}

void vAHRSConfig(void* pvParameters)
{	
	float acc[3]={0.0};
	float acczMean=0.0;
	u8 i;
	
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

	/*read acc cali data from flash*/
	//read flash 15 float = 60 byte
	xQueueReceive(xAccCaliQueue,&accCaliStructure,portMAX_DELAY);
	vTaskDelay((portTickType)3000/portTICK_RATE_MS);
	
	/*I2C interrupt config*/
	User_I2C_IT_Config();
	
	//check acc
	//if invert ..
	//else read flash
	AHRS_Read_SPI_Acc(acc);
	
	acczMean=acc[2];
	for(i=2;i<20;i++)
	{	
		AHRS_Read_SPI_Acc(acc);
		acczMean=acczMean+(acc[2]-acczMean)/i;
		vTaskDelay((portTickType)10/portTICK_RATE_MS);
	}
	if(acczMean>7.5)	//invert
	{
		xTaskCreate(vAHRSCali,(signed char *)"ahrs_cali"
				,configMINIMAL_STACK_SIZE+1024
				,(void *)(&accCaliStructure),mainFLASH_TASK_PRIORITY+2
				,(xTaskHandle *)NULL);
	}
	else
	{
		xTaskCreate(vAHRSReadRaw
				    ,(signed char *)"ahrs_read"
						,configMINIMAL_STACK_SIZE+128
						,NULL,mainFLASH_TASK_PRIORITY+3
						,(xTaskHandle *)NULL);
	}
	vTaskDelete(NULL);	
}

void vAHRSCali(void* pvParameters)
{
	AccCaliType act={{0}};
	float A[18]={0.0};
	float U[24]={
		 GRAVITY, -GRAVITY, 0.0, 	0.0, 	0.0, 	0.0
		,0.0,   0.0,  	GRAVITY, -GRAVITY, 	0.0, 	0.0
		,0.0,   0.0,  	0.0, 	0.0 ,	GRAVITY, -GRAVITY
		,1.0,   1.0,  	1.0, 	1.0, 	1.0, 	1.0
		};
	float acc[3]={0.0};
	float gyro[3]={0.0};
	float gyroVar=0.0;
	float gyroVarStable=0.0;
	float buffer[100]={0.0};

	arm_matrix_instance_f32 KMat,AMat,UMat;
	arm_matrix_instance_f32 trUMat,UtrUMat,invUtrUMat,trUinvUtrUMat;

	u8 i;

	KMat.numRows = 3;
	KMat.numCols = 4;
	KMat.pData = act.K;

	AMat.numRows = 3;
	AMat.numCols = 6;
	AMat.pData = A;

	UMat.numRows = 4;
	UMat.numCols = 6;
	UMat.pData = U;

	Blinks(LED1,3);//indicate calibration process begin
	vTaskDelay((portTickType)5000/portTICK_RATE_MS);
	
	/*refresh sensor data*/
	AHRS_Read_I2C_Gyr(gyro);
	
	Blinks(LED1,2);//indicate collecting data
	for(i=0;i<100;i++)
	{
		AHRS_Read_I2C_Gyr(gyro);
		buffer[i]=gyro[0];
		vTaskDelay((portTickType)20/portTICK_RATE_MS);
	}
	Blinks(LED1,3);
	gyroVarStable = Var(buffer,100);

	AHRS_Read_SPI_Acc(acc);
	while(acc[0]<7.8 || gyroVar>gyroVarStable)  //x axis upward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_I2C_Gyr(gyro);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_SPI_Acc(acc);
	}
	Blinks(LED1,2);
	A[0]=acc[0];	A[6]=acc[1];	A[12]=acc[2];

	for(i=2;i<100;i++)
	{
		AHRS_Read_SPI_Acc(acc);
		A[0]=A[0]+(acc[0]-A[0])/i;
		A[6]=A[6]+(acc[1]-A[6])/i;
		A[12]=A[12]+(acc[2]-A[12])/i;
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_SPI_Acc(acc);
	while(acc[0]>-7.8 || gyroVar>gyroVarStable)  //x axis downward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_I2C_Gyr(gyro);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_SPI_Acc(acc);
	}
	Blinks(LED1,2);
	A[1]=acc[0];	A[7]=acc[1];	A[13]=acc[2];

	for(i=2;i<100;i++)
	{
		AHRS_Read_SPI_Acc(acc);
		A[1]=A[1]+(acc[0]-A[1])/i;
		A[7]=A[7]+(acc[1]-A[7])/i;
		A[13]=A[13]+(acc[2]-A[13])/i;
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_SPI_Acc(acc);
	while(acc[1]<7.8 || gyroVar>gyroVarStable)  //y axis upward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_I2C_Gyr(gyro);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_SPI_Acc(acc);
	}
	Blinks(LED1,2);
	A[2]=acc[0];	A[8]=acc[1];A[14]=acc[2];

	for(i=2;i<100;i++)
	{
		AHRS_Read_SPI_Acc(acc);
		A[2]=A[2]+(acc[0]-A[2])/i;
		A[8]=A[8]+(acc[1]-A[8])/i;
		A[14]=A[14]+(acc[2]-A[14])/i;
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
	Blinks(LED1,3);
	
	AHRS_Read_SPI_Acc(acc);
	while(acc[1]>-7.8 || gyroVar>gyroVarStable)  //y axis downward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_I2C_Gyr(gyro);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_SPI_Acc(acc);
	}
	Blinks(LED1,2);
	A[3]=acc[0];	A[9]=acc[1];	A[15]=acc[2];

	for(i=2;i<100;i++)
	{
		AHRS_Read_SPI_Acc(acc);
		A[3]=A[3]+(acc[0]-A[3])/i;
		A[9]=A[9]+(acc[1]-A[9])/i;
		A[15]=A[15]+(acc[2]-A[15])/i;
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_SPI_Acc(acc);
	while(acc[2]<7.8 || gyroVar>gyroVarStable)  //z axis upward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_I2C_Gyr(gyro);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_SPI_Acc(acc);
	}
	Blinks(LED1,2);
	A[4]=acc[0];	A[10]=acc[1];	A[16]=acc[2];

	for(i=2;i<100;i++)
	{
		AHRS_Read_SPI_Acc(acc);
		A[4]=A[4]+(acc[0]-A[4])/i;
		A[10]=A[10]+(acc[1]-A[10])/i;
		A[16]=A[16]+(acc[2]-A[16])/i;
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_SPI_Acc(acc);
	while(acc[2]>-7.8 || gyroVar>gyroVarStable)  //z axis downward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_I2C_Gyr(gyro);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_SPI_Acc(acc);
	}
	Blinks(LED1,2);
	A[5]=acc[0];	A[11]=acc[1];A[17]=acc[2];

	for(i=2;i<100;i++)
	{
		AHRS_Read_SPI_Acc(acc);
		A[5]=A[5]+(acc[0]-A[5])/i;
		A[11]=A[11]+(acc[1]-A[11])/i;
		A[17]=A[17]+(acc[2]-A[17])/i;
		vTaskDelay((portTickType)50/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	trUMat.numRows = 6;
	trUMat.numCols = 4;
	trUMat.pData = pvPortMalloc(96);

	arm_mat_trans_f32(&UMat, &trUMat);

	UtrUMat.numRows = 4;
	UtrUMat.numCols = 4;
	UtrUMat.pData = pvPortMalloc(64);

	arm_mat_mult_f32(&UMat, &trUMat, &UtrUMat);

	invUtrUMat.numRows = 4;
	invUtrUMat.numCols = 4;
	invUtrUMat.pData = pvPortMalloc(64);

	arm_mat_inverse_f32(&UtrUMat, &invUtrUMat);

	vPortFree(UtrUMat.pData);

	trUinvUtrUMat.numRows = 6;
	trUinvUtrUMat.numCols = 4;
	trUinvUtrUMat.pData = pvPortMalloc(96);

	arm_mat_mult_f32(&trUMat, &invUtrUMat, &trUinvUtrUMat);

	vPortFree(trUMat.pData);
	vPortFree(invUtrUMat.pData);

	arm_mat_mult_f32(&AMat, &trUinvUtrUMat, &KMat);
	vPortFree(trUinvUtrUMat.pData);

	*(AccCaliType *)pvParameters = act;
	xQueueSend(xAccCaliQueue,&act,portMAX_DELAY);

	xTaskCreate(vAHRSReadRaw
		    ,(signed char *)"ahrs_read"
				,configMINIMAL_STACK_SIZE+128
				,NULL,mainFLASH_TASK_PRIORITY+2
				,(xTaskHandle *)NULL);
	vTaskDelete(NULL);
}

void vAHRSReadRaw(void* pvParameters)
{
	//通信数据
	SensorDataType sdt;
	SensorDataType sdtTrashCan;
	u8 CNT=0;

	//补偿量
	float K[9];
	float bias_a[3];
	arm_matrix_instance_f32 KMat,invKMat;
	float gyro_offset[3]={0};
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
	//初始化补偿系数
	Gyro_Cali(gyro_offset);

	K[0]=accCaliStructure.K[0];	K[1]=accCaliStructure.K[1];	K[2]=accCaliStructure.K[2];
	K[3]=accCaliStructure.K[4]; K[4]=accCaliStructure.K[5]; K[5]=accCaliStructure.K[6];
	K[6]=accCaliStructure.K[8]; K[7]=accCaliStructure.K[9]; K[8]=accCaliStructure.K[10];

	bias_a[0]=accCaliStructure.K[3];	bias_a[1]=accCaliStructure.K[7];	bias_a[2]=accCaliStructure.K[11];

	KMat.numRows = 3;
	KMat.numCols = 3;
	KMat.pData = K;

	invKMat.numRows = 3;
	invKMat.numCols = 3;
	invKMat.pData = pvPortMalloc(36);

	arm_mat_inverse_f32(&KMat, &invKMat);
	memcpy(K, invKMat.pData, 36);

	vPortFree(invKMat.pData);

	//硬件配置
	SPI_DMA_Config();
	SPI_DMA_IT_Config();
	
	Blinks(LED1,1);
	xLastReadTime=xTaskGetTickCount();
	for(;;)
	{
		AHRS_Read_I2C_Gyr(gyr);
		AHRS_Read_SPI_Acc(acc);
		
		sdt.acc[0] = K[0]*(acc[0]-bias_a[0])+K[1]*(acc[1]-bias_a[1])+K[2]*(acc[2]-bias_a[2]);
		sdt.acc[1] = K[3]*(acc[0]-bias_a[0])+K[4]*(acc[1]-bias_a[1])+K[5]*(acc[2]-bias_a[2]);
		sdt.acc[2] = K[6]*(acc[0]-bias_a[0])+K[7]*(acc[1]-bias_a[1])+K[8]*(acc[2]-bias_a[2]);

		sdt.gyr[0] = gyr[0]-gyro_offset[0];
		sdt.gyr[1] = gyr[1]-gyro_offset[1];
		sdt.gyr[2] = gyr[2]-gyro_offset[2];

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

		if(CNT++>10)
		{
			CNT=0;
			AHRS_Read_I2C_Mag(sdt.mag);
		}

		for(i=0;i<3;i++) comt.data[i]=(s16)(sdt.gyr[i]*4000.0);
		for(i=0;i<3;i++) comt.data[i+3]=(s16)(sdt.acc[i]*1000.0);
		for(i=0;i<3;i++) comt.data[i+6]=sdt.mag[i];
		comt.Check=0;
		for(i=0;i<9;i++) 
			comt.Check+=comt.data[i];
		
		buffer_lock_global = 1;
		LoadRawData(spi_mid_buffer);
		buffer_lock_global = 0;
		
		xQueueReceive(xEKFQueue, &sdtTrashCan, 0);
		xQueueSend(xEKFQueue, &sdt, 0);

		vTaskDelayUntil(&xLastReadTime,(portTickType)2/portTICK_RATE_MS);
	}
}

float Mean(float* sample,u16 N)
{
	float sum=0.0;
	u16 i;
	for(i=0;i<N;i++)
	{
		sum += sample[i];
	}
	return sum/N;
}

float Var(float* sample,u16 N)
{
	float mean;
	u16 i;
	float var=0.0;
	mean = Mean(sample,N);
	for(i=0;i<N;i++)
	{
		var += (sample[i]-mean)*(sample[i]-mean);
	}
	var /= N;
	
	return var;
}

void Gyro_Cali(float* gyro_offset)
{
	u16 i;
	u8 j;

	float rawData[3];
	float sum[3]={0.0};

	for(i=0;i<1000;i++)
	{
		AHRS_Read_I2C_Gyr(rawData);
		
		for(j=0;j<3;j++)
			sum[j]+=rawData[j];
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	for(j=0;j<3;j++)
		gyro_offset[j]=(sum[j]/1000.0);
}

void LoadRawData(u8 *buffer)
{
	*(ComType *)buffer = comt;
}

void LoadBaroData(u8 *buffer)
{
	*(BaroHeightType *)buffer = bht;
}

