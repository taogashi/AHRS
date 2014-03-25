#include "calibration.h"
#include "ledTask.h"
#include "AHRS.h"
#include "statistic.h"
#include "kalman.h"
#include "mag3110.h"
#include "I2C.h"
#include <arm_math.h>

/********************************/
const float mP[36]={1.0,	0,	0, 0, 0, 0,
					0,	1.0,	0, 0, 0, 0,
					0,	0,	1.0, 0,0, 0,
					0,  0,  0, 1.0, 0, 0,
					0,  0,  0,  0,  1.0, 0,
					0,  0,  0,  0,  0,  1.0};

const float mR=100.0;   //????????

const float caliQ[36]={0.0};

void Cali_GetH(float *H,void *para1,void *para2,void *para3, void *para4);
void Cali_hFunc(float *hx,void *para1,void *para2,void *para3, void *para4);

void AHRSAccCali(IMUCaliType *ict)
{
	//----------------------------------------------------
	float acc_cali_mat[12]={0.0};
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
	arm_matrix_instance_f32 CoefMat;
	
	arm_matrix_instance_f32 invCoefMat;
	arm_matrix_instance_f32 trUMat,UtrUMat,invUtrUMat,trUinvUtrUMat;

	u16 i;

	Blinks(LED1,3);//indicate calibration process begin
	vTaskDelay((portTickType)5000/portTICK_RATE_MS);
	Blinks(LED1,2);//indicate collecting data
	for(i=0;i<100;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		buffer[i]=gyro[0];
		vTaskDelay((portTickType)20/portTICK_RATE_MS);
	}
	Blinks(LED1,3);
	gyroVarStable = Var(buffer,100);

	AHRS_Read_IMU(gyro,acc);
	while(acc[0]<7.8 || gyroVar>gyroVarStable)  //x axis upward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_IMU(gyro,acc);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_IMU(gyro,acc);
	}
	Blinks(LED1,2);
	A[0]=acc[0];	A[6]=acc[1];	A[12]=acc[2];

	for(i=2;i<1000;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		A[0]=A[0]+(acc[0]-A[0])/i;
		A[6]=A[6]+(acc[1]-A[6])/i;
		A[12]=A[12]+(acc[2]-A[12])/i;
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_IMU(gyro,acc);
	while(acc[0]>-7.8 || gyroVar>gyroVarStable)  //x axis downward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_IMU(gyro,acc);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_IMU(gyro,acc);
	}
	Blinks(LED1,2);
	A[1]=acc[0];	A[7]=acc[1];	A[13]=acc[2];

	for(i=2;i<1000;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		A[1]=A[1]+(acc[0]-A[1])/i;
		A[7]=A[7]+(acc[1]-A[7])/i;
		A[13]=A[13]+(acc[2]-A[13])/i;
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_IMU(gyro,acc);
	while(acc[1]<7.8 || gyroVar>gyroVarStable)  //y axis upward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_IMU(gyro,acc);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_IMU(gyro,acc);
	}
	Blinks(LED1,2);
	A[2]=acc[0];	A[8]=acc[1];A[14]=acc[2];

	for(i=2;i<1000;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		A[2]=A[2]+(acc[0]-A[2])/i;
		A[8]=A[8]+(acc[1]-A[8])/i;
		A[14]=A[14]+(acc[2]-A[14])/i;
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1,3);
	
	AHRS_Read_IMU(gyro,acc);
	while(acc[1]>-7.8 || gyroVar>gyroVarStable)  //y axis downward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_IMU(gyro,acc);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_IMU(gyro,acc);
	}
	Blinks(LED1,2);
	A[3]=acc[0];	A[9]=acc[1];	A[15]=acc[2];

	for(i=2;i<1000;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		A[3]=A[3]+(acc[0]-A[3])/i;
		A[9]=A[9]+(acc[1]-A[9])/i;
		A[15]=A[15]+(acc[2]-A[15])/i;
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_IMU(gyro,acc);
	while(acc[2]<7.8 || gyroVar>gyroVarStable)  //z axis upward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_IMU(gyro,acc);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_IMU(gyro,acc);
	}
	Blinks(LED1,2);
	A[4]=acc[0];	A[10]=acc[1];	A[16]=acc[2];

	for(i=2;i<1000;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		A[4]=A[4]+(acc[0]-A[4])/i;
		A[10]=A[10]+(acc[1]-A[10])/i;
		A[16]=A[16]+(acc[2]-A[16])/i;
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1,3);

	AHRS_Read_IMU(gyro,acc);
	while(acc[2]>-7.8 || gyroVar>gyroVarStable)  //z axis downward
	{
		for(i=0;i<100;i++)
		{
			AHRS_Read_IMU(gyro,acc);
			buffer[i]=gyro[0];
			vTaskDelay((portTickType)20/portTICK_RATE_MS);
		}
		gyroVar=Var(buffer,100);
		AHRS_Read_IMU(gyro,acc);
	}
	Blinks(LED1,2);
	A[5]=acc[0];	A[11]=acc[1];A[17]=acc[2];

	for(i=2;i<1000;i++)
	{
		AHRS_Read_IMU(gyro,acc);
		A[5]=A[5]+(acc[0]-A[5])/i;
		A[11]=A[11]+(acc[1]-A[11])/i;
		A[17]=A[17]+(acc[2]-A[17])/i;
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1,1);
	
	KMat.numRows = 3;
	KMat.numCols = 4;
	KMat.pData = acc_cali_mat;

	AMat.numRows = 3;
	AMat.numCols = 6;
	AMat.pData = A;

	UMat.numRows = 4;
	UMat.numCols = 6;
	UMat.pData = U;

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
	
	invCoefMat.numRows = 3;
	invCoefMat.numCols = 3;
	invCoefMat.pData = pvPortMalloc(36);
	
	invCoefMat.pData[0] = KMat.pData[0];	invCoefMat.pData[1] = KMat.pData[1];	invCoefMat.pData[2] = KMat.pData[2];
	invCoefMat.pData[3] = KMat.pData[4];	invCoefMat.pData[4] = KMat.pData[5];	invCoefMat.pData[5] = KMat.pData[6];
	invCoefMat.pData[6] = KMat.pData[8];	invCoefMat.pData[7] = KMat.pData[9];	invCoefMat.pData[8] = KMat.pData[10];
	
	CoefMat.numRows = 3;
	CoefMat.numCols = 3;
	CoefMat.pData = ict->acc_coef;
	
	arm_mat_inverse_f32(&invCoefMat, &CoefMat);
	vPortFree(invCoefMat.pData);

	ict->acc_bias[0] = KMat.pData[3];
	ict->acc_bias[1] = KMat.pData[7];
	ict->acc_bias[2] = KMat.pData[11];
	
	ict->valid |= 0x02;
}

void AHRSGyrCali(IMUCaliType *ict)
{
	u16 i;
	u8 j;
	float rawData[3];
	float acc[3];
	float sum[3]={0.0};

	Blinks(LED1, 2);
	for(i=0;i<500;i++)
	{
		AHRS_Read_IMU(rawData,acc);
		for(j=0;j<3;j++)
			sum[j]+=rawData[j];
		vTaskDelay((portTickType)5/portTICK_RATE_MS);
	}
	Blinks(LED1, 1);
	for(j=0;j<3;j++)
		ict->gyr_bias[j]=(sum[j]/500.0);
		
	ict->gyr_scale[0] = 1.0;
	ict->gyr_scale[1] = 1.0;
	ict->gyr_scale[2] = 1.0;
	/*
	x axis
	
	x_cali = 0;
	while(! x_cali)
	{
		g_roll = 0;
		read IMU and compute roll angle by acc, named a_roll_start;
		for(i=0; i<125; i++)
		{
			delayUntil(2ms);
			g_roll += x_rate*dt;
			read IMU;
			if y_rate > 0.1rad/s or z_rate > 0.1rad/s
				break;
		}
		compute roll angle again, named a_roll_end;
		if(i==124)
		{
			scale = (a_roll_end - a_roll_start)/g_roll;
			x_cali = 1;
		}	
	}
	*/
}

void AHRSMagCali(IMUCaliType *ict)
{	
	u8 mag_raw[6];
	s16 mag[3];
	
	float measure = 1.0;
	
	ekf_filter mag_estimator;
	
	float gama;
	float radii[3];
	
	float geo_amp = 0.0;
	float last_amp = 0.0;
	float square_amp_err = 100.0;
	float calid_mag[3];
	
	mag_estimator = ekf_filter_new(6, 1
							, caliQ, &mR
							, NULL, Cali_GetH
							, NULL, Cali_hFunc);
	memcpy(mag_estimator->P, mP, mag_estimator->state_dim*mag_estimator->state_dim*sizeof(float));
	mag_estimator->x[0] = 0.0;
	mag_estimator->x[1] = 0.0;
	mag_estimator->x[2] = 0.0;
	mag_estimator->x[3] = 0.0;
	mag_estimator->x[4] = 0.0;
	mag_estimator->x[5] = 0.0;				
	
	Blinks(LED1, 2);
	for(;;)
	{	
		User_I2C_BufferRead(MAG3110_ADDR, mag_raw, MAG3110_OUT_X_MSB_REG, 6);
		MAG3110_Raw2Mag(mag_raw, mag);
		
		EKF_update(mag_estimator
					, &measure
					, (void *)mag
					, (void *)mag_estimator->H
					, (void *)mag_estimator->x
					, NULL);
		ict->mag_bias[0] = -mag_estimator->x[3]/mag_estimator->x[0];
		ict->mag_bias[1] = -mag_estimator->x[4]/mag_estimator->x[1];
		ict->mag_bias[2] = -mag_estimator->x[5]/mag_estimator->x[2];
		
		gama = 1+(mag_estimator->x[3]*mag_estimator->x[3]/mag_estimator->x[0]
					+mag_estimator->x[4]*mag_estimator->x[4]/mag_estimator->x[1]
					+mag_estimator->x[5]*mag_estimator->x[5]/mag_estimator->x[2]);
		arm_sqrt_f32(gama/mag_estimator->x[0], &radii[0]);
		arm_sqrt_f32(gama/mag_estimator->x[1], &radii[1]);
		arm_sqrt_f32(gama/mag_estimator->x[2], &radii[2]);
		
		calid_mag[0] = (mag[0] - ict->mag_bias[0])/radii[0];
		calid_mag[1] = (mag[1] - ict->mag_bias[1])/radii[1];
		calid_mag[2] = (mag[2] - ict->mag_bias[2])/radii[2];
		
		geo_amp = calid_mag[0]*calid_mag[0] + calid_mag[1]*calid_mag[1] + calid_mag[2]*calid_mag[2];
		square_amp_err = 0.98*square_amp_err + 0.02*(geo_amp - last_amp)*(geo_amp - last_amp);
		last_amp = geo_amp;
		
		if(square_amp_err < 0.000000000000001)
		{
			ict->mag_scale[0] = 500.0/radii[0];
			ict->mag_scale[1] = 500.0/radii[1];
			ict->mag_scale[2] = 500.0/radii[2];
			
			ict->valid |= 0x01;
			
			Blinks(LED1, 1);
			return;
		}
		
		vTaskDelay((portTickType)40/portTICK_RATE_MS);
	}
}

void Cali_GetH(float *H,void *para1,void *para2,void *para3, void *para4)
{
	float *data = (float *)para1;
	H[0] = data[0]*data[0];
	H[1] = data[1]*data[1];
	H[2] = data[2]*data[2];
	H[3] = 2*data[0];
	H[4] = 2*data[1];
	H[5] = 2*data[2];
}

void Cali_hFunc(float *hx,void *para1,void *para2,void *para3, void *para4)
{
	u8 i;
	
	float *H = (float *)para2;
	float *x = (float *)para3;
	
	*hx = 0.0;
	
	for(i=0; i<6; i++)
	{
		*hx += x[i]*H[i];
	}
}
