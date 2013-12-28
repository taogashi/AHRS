#include "MPU6050.h"
#include "HAL_MPU6050.h"
#include "OSConfig.h"

u8 gyroFS=0xff;
const float gyroFac[3][3]=
{
	{0.007694,0.00016434,0.000065527},
  	{-0.00014163,0.0075211,-0.000049992},
  	{0.00007821,-0.0000364,0.0076165}
};

void MPU6050_I2C_Init(void)	//初始化I2C总线
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(MPU6050_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(MPU6050_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  MPU6050_I2C_SCL_Pin | MPU6050_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(MPU6050_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = MPU6050_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(MPU6050_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(MPU6050_I2C, ENABLE);
}

u8 MPU6050_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
	u32 timeout;
	AHRS_ENTER_CRITICAL();
	/* Send START condition */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);

	/* Test on EV5 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) && timeout-->0);
	if(timeout==0) return 0;

	/* Send MPU6050303DLH_Magn address for write */
	I2C_Send7bitAddress(MPU6050_I2C, slAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout-->0);
	if(timeout==0) return 0;

	/* Send the MPU6050303DLH_Magn's internal address to write to */
	I2C_SendData(MPU6050_I2C, WriteAddr);

	/* Test on EV8 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout-->0);
	if(timeout==0) return 0;

	/* Send the byte to be written */
	I2C_SendData(MPU6050_I2C, *pBuffer);

	/* Test on EV8 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout-->0);
	if(timeout==0) return 0;

	/* Send STOP condition */
	I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
	AHRS_EXIT_CRITICAL();  
	return 1;
}

u8 MPU6050_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u32 timeout;
	AHRS_ENTER_CRITICAL();
	/* While the bus is busy */
	timeout = 50000;
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY) && timeout-->0);
	if(timeout==0) return 0;

	/* Send START condition */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);

	/* Test on EV5 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) && timeout-->0);
	if(timeout==0) return 0;

	/* Send MPU6050303DLH_Magn address for write */
	I2C_Send7bitAddress(MPU6050_I2C, slAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout-->0);
	if(timeout==0) return 0;

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(MPU6050_I2C, ENABLE);

	/* Send the MPU6050303DLH_Magn's internal address to write to */
	I2C_SendData(MPU6050_I2C, ReadAddr);

	/* Test on EV8 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout-->0);
	if(timeout==0) return 0;

	/* Send STRAT condition a second time */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);

	/* Test on EV5 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) && timeout-->0);
	if(timeout==0) return 0;

	/* Send MPU6050303DLH address for read */
	I2C_Send7bitAddress(MPU6050_I2C, slAddr, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout-->0);
	if(timeout==0) return 0;

	/* While there is data to be read */
	while(NumByteToRead)
	{
	if(NumByteToRead == 1)
	{
	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

	/* Send STOP Condition */
	I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
	}

	/* Test on EV7 and clear it */
	if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
	/* Read a byte from the MPU6050303DLH */
	*pBuffer = I2C_ReceiveData(MPU6050_I2C);

	/* Point to the next location where the byte read will be saved */
	pBuffer++;

	/* Decrement the read bytes counter */
	NumByteToRead--;
	}
	}

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
	AHRS_EXIT_CRITICAL();
	return 1;
}

u8 MPU6050_Init(MPU6050_ConfigTypeDef *MPU6050_Config_Struct)
{
	u8 errStatus=1;
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 smplrt_div;	   //0x13
	u8 ext_sync_dlpf=MPU6050_Config_Struct->LPFBandwidth;		 //0x00
	u8 gyro_selftest_FS=MPU6050_Config_Struct->GyroSelfTestStatus 
						| MPU6050_Config_Struct->GyroFS;	 //0x00
	u8 acc_selftest_FS_hpf=MPU6050_Config_Struct->AccSelfTestStatus 
						| MPU6050_Config_Struct->AccFS 
						| MPU6050_Config_Struct->AccHPF;	  //0x00
	u8 fifo=MPU6050_Config_Struct->FIFOConfig;				 //0x00
	u8 int_pin=MPU6050_Config_Struct->IntPinLevel 
			| MPU6050_Config_Struct->IntPinType 
			| MPU6050_Config_Struct->IntLatchMode 
			| MPU6050_Config_Struct->IntClearMode
			| MPU6050_Config_Struct->FSYNCIntLevel
			| MPU6050_Config_Struct->FSYNCIntMode
			| MPU6050_Config_Struct->CLKOUTMode;		   //0xb0

	u8 int_en=MPU6050_Config_Struct->DRDYIntMode;		//0x01
	u8 user_ctrl=MPU6050_Config_Struct->FIFOMode | MPU6050_Config_Struct->I2CMasterStatus;	 //0x00
	u8 pwr_mgmt1=MPU6050_Config_Struct->CLKSel;		 //0x01
	u8 pwr_mgmt2=0x00;

	if(MPU6050_Config_Struct->LPFBandwidth == MPU6050_DLPF_Bandwidth_A260_G256)
		smplrt_div=(u8) (8000.0/MPU6050_Config_Struct->SamRate)-1;
	else
		smplrt_div=(u8) (1000.0/MPU6050_Config_Struct->SamRate)-1;		 

	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&smplrt_div,MPU6050_RA_SMPLRT_DIV);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&ext_sync_dlpf,MPU6050_RA_CONFIG);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&gyro_selftest_FS,MPU6050_RA_GYRO_CONFIG);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&acc_selftest_FS_hpf,MPU6050_RA_ACCEL_CONFIG);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&fifo,MPU6050_RA_FIFO_EN);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&int_pin,MPU6050_RA_INT_PIN_CFG);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&int_en,MPU6050_RA_INT_ENABLE);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&user_ctrl,MPU6050_RA_USER_CTRL);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&pwr_mgmt1,MPU6050_RA_PWR_MGMT_1);
	errStatus &= MPU6050_I2C_ByteWrite(MPU6050_ADDRESS_AD0_LOW,&pwr_mgmt2,MPU6050_RA_PWR_MGMT_2);

	RCC_APB2PeriphClockCmd(MPU6050_DRDY_RCC_Port, ENABLE);	  //GPIOC时钟
	GPIO_InitStructure.GPIO_Pin = MPU6050_DRDY_Pin ;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MPU6050_DRDY_Port, &GPIO_InitStructure);

	gyroFS=MPU6050_Config_Struct->GyroFS;
	return errStatus;
}

u8 MPU6050_Read_PID(u8* pid)	//读设备ID
{
	u8 who_am_i=0x00;
	u8 errStatus=1;;
	errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW,&who_am_i,MPU6050_RA_WHO_AM_I,1);
	*pid=who_am_i;
	return errStatus;
}

u8 MPU6050_Read_GyroRaw(s16* out)   //读陀螺仪原始数据
{
	u8 errStatus=1;
	u8 buffer[6];
	u8 testReady;
//	u8 intStatus;
	u8 i;
	
	testReady = GPIO_ReadInputDataBit(MPU6050_DRDY_Port , MPU6050_DRDY_Pin);
	if(testReady == 0)
	{
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &intStatus, ITG_INT_STATUS_REG_ADDR, 1);
//		if(intStatus==0x01)
//		{	  
			errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, buffer, MPU6050_RA_GYRO_XOUT_H, 6);
			for(i=0;i<3;i++)
			{
				out[i] = buffer[2*i] & 0x00ff;
				out[i] <<= 8;
				out[i] |= buffer[2*i+1];
			}
//		}
	}	
	return errStatus;
}

u8 MPU6050_Read_GyroRate(float* out)	 //读陀螺仪角速率
{
	u8 errStatus=1;
	s16 rawData[3];
	float temp;
	static u8 FS[2]={0xff,0xff};
	u8 i;
	errStatus &= MPU6050_Read_GyroRaw(rawData);

	FS[0]=gyroFS;
	if(FS[0]==0xff)
		errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, FS, MPU6050_RA_GYRO_CONFIG, 2);
	if((FS[0]&(0x18))==MPU6050_Gyro_FS_250)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]/7505.75;//(rawData[0]*gyroFac[i][0]+rawData[1]*gyroFac[i][1]+rawData[2]*gyroFac[i][2])/57.296;
		}
	else if((FS[0]&(0x18))==MPU6050_Gyro_FS_500)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]/3752.9;
		}
	else if((FS[0]&(0x18))==MPU6050_Gyro_FS_1000)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]/1879.3;
		}
	else if((FS[0]&(0x18))==MPU6050_Gyro_FS_2000)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]/938.2;
		}
	temp=out[0];
	out[0]=-out[1];
	out[1]=-temp;
	out[2]=-out[2];
	return errStatus;
}

u8 MPU6050_Read_AccRaw(s16* out)	   //读加速度计原始数据
{
	u8 errStatus=1;
	u8 buffer[6];
	u8 testReady;
//	u8 intStatus;
	u8 i;
	
	testReady = GPIO_ReadInputDataBit(MPU6050_DRDY_Port , MPU6050_DRDY_Pin);
	if(testReady == 0)
	{
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &intStatus, ITG_INT_STATUS_REG_ADDR, 1);
//		if(intStatus==0x01)
//		{	  
			errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, buffer, MPU6050_RA_ACCEL_XOUT_H, 6);
			for(i=0;i<3;i++)
			{
				out[i] = buffer[2*i] & 0x00ff;
				out[i] <<= 8;
				out[i] |= buffer[2*i+1];
			}
//		}
	}
	return errStatus;
}

u8 MPU6050_Read_Acc(float* out)	 //读加速度
{
	u8 errStatus=1;
	s16 rawData[3];
	static u8 FS=0xff;
	u8 i;
	errStatus &= MPU6050_Read_AccRaw(rawData);
	if(FS==0xff)
		errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, &FS, MPU6050_RA_ACCEL_CONFIG, 1);
	if((FS&(0x18))==MPU6050_Acc_FS_2g)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]*0.06;
		}
	else if((FS&(0x18))==MPU6050_Acc_FS_4g)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]*0.12;
		}
	else if((FS&(0x18))==MPU6050_Acc_FS_8g)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]*0.24;
		}
	else if((FS&(0x18))==MPU6050_Acc_FS_16g)
		for(i=0;i<3;i++)
		{
			out[i]=(float)rawData[i]*0.49;
		}
	return errStatus;
}

u8 MPU6050_Read_TempRaw(s16* temp) //读温度原始数据
{
	u8 errStatus=1;
	u8 buffer[2]; 
	errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, buffer, MPU6050_RA_TEMP_OUT_H, 2);
	temp[0] = buffer[0] & 0x00ff;
	temp[0] <<= 8;
	temp[0] |= buffer[1];
	return errStatus;
}

u8 MPU6050_read_Temp(s16* temp)//读温度值，精确到0.1℃
{
	u8 errStatus=1;
	errStatus &= MPU6050_Read_TempRaw(temp);
	 *temp = (s16)((*temp+3079)/32.5);
	return errStatus;
}

u8 MPU6050_Read_Raw(s16* gyr,s16* acc)		//读取所有数据
{
	u8 errStatus=1;
	u8 buffer[14];
	u8 testReady;
	u8 i;	
	testReady = GPIO_ReadInputDataBit(MPU6050_DRDY_Port , MPU6050_DRDY_Pin);
	if(testReady == 0)
	{  
		errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, buffer, MPU6050_RA_ACCEL_XOUT_H, 14);
		for(i=0;i<3;i++)
		{
			acc[i] = buffer[2*i] & 0x00ff;
			acc[i] <<= 8;
			acc[i] |= buffer[2*i+1];
		}
		for(i=0;i<3;i++)
		{
			gyr[i] = buffer[2*i+8] & 0x00ff;
			gyr[i] <<= 8;
			gyr[i] |= buffer[2*i+1+8];
		}
	}
	return errStatus;
}

u8 MPU6050_Read_GyroAcc(float* gyr,float* acc) 	
{
	u8 errStatus=1;
	s16 gyrRaw[3];
	s16 accRaw[3];	
	static u8 gFS=0xff;
	static u8 aFS=0xff;
	u8 i;
	errStatus &= MPU6050_Read_Raw(gyrRaw,accRaw);
	if(gFS==0xff)
		errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, &gFS, MPU6050_RA_GYRO_CONFIG, 1);
	if(aFS==0xff)
		errStatus &= MPU6050_I2C_BufferRead(MPU6050_ADDRESS_AD0_LOW, &aFS, MPU6050_RA_ACCEL_CONFIG, 1);

	if((aFS&(0x18))==MPU6050_Acc_FS_2g)
		for(i=0;i<3;i++)
		{
			acc[i]=(float)accRaw[i]*0.06;
		}
	else if((aFS&(0x18))==MPU6050_Acc_FS_4g)
		for(i=0;i<3;i++)
		{
			acc[i]=(float)accRaw[i]*0.12;
		}
	else if((aFS&(0x18))==MPU6050_Acc_FS_8g)
		for(i=0;i<3;i++)
		{
			acc[i]=(float)accRaw[i]*0.24;
		}
	else if((aFS&(0x18))==MPU6050_Acc_FS_16g)
		for(i=0;i<3;i++)
		{
			acc[i]=(float)accRaw[i]*0.49;
		}
//--------------------------------------------
	if((gFS&(0x18))==MPU6050_Gyro_FS_250)
		for(i=0;i<3;i++)
		{
			gyr[i]=(float)gyrRaw[i]/7505.7;
		}
	else if((gFS&(0x18))==MPU6050_Gyro_FS_500)
		for(i=0;i<3;i++)
		{
			gyr[i]=(float)gyrRaw[i]/3752.9;
		}
	else if((gFS&(0x18))==MPU6050_Gyro_FS_1000)
		for(i=0;i<3;i++)
		{
			gyr[i]=(float)gyrRaw[i]/1876.4;
		}
	else if((gFS&(0x18))==MPU6050_Gyro_FS_2000)
		for(i=0;i<3;i++)
		{
			gyr[i]=(float)gyrRaw[i]/938.2;
		}
	return errStatus;
}

