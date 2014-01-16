#include "MS5607B.h"
#include "HAL_MS5607B.h"
#include "OSConfig.h"

/*I2C Group*/

void MS5607B_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(MS5607B_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(MS5607B_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  MS5607B_I2C_SCL_Pin | MS5607B_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(MS5607B_I2C_Port, &GPIO_InitStructure);

  I2C_DeInit(MS5607B_I2C);	  	//把I2C外设的寄存器重置成默认值	  

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = MS5607B_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(MS5607B_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(MS5607B_I2C, ENABLE);
}

u8 MS5607B_I2C_WriteCmd(u8 slAddr, u8 cmd)
{
	u32 timeout;
	AHRS_ENTER_CRITICAL();
	/* Send START condition */
	I2C_GenerateSTART(MS5607B_I2C, ENABLE);
	
	/* Test on EV5 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_MODE_SELECT) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send MS5607B303DLH_Magn address for write */
	I2C_Send7bitAddress(MS5607B_I2C, slAddr, I2C_Direction_Transmitter);
	
	/* Test on EV6 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send the MS5607B303DLH_Magn's internal address to write to */
	I2C_SendData(MS5607B_I2C, cmd);
	
	/* Test on EV8 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send STOP condition */
	I2C_GenerateSTOP(MS5607B_I2C, ENABLE);
	
	AHRS_EXIT_CRITICAL();
	return 1;  
}

u8 MS5607B_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u32 timeout;
	AHRS_ENTER_CRITICAL();
	/* While the bus is busy */
	timeout = 50000;
	while(I2C_GetFlagStatus(MS5607B_I2C, I2C_FLAG_BUSY) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send START condition */
	I2C_GenerateSTART(MS5607B_I2C, ENABLE);
	
	/* Test on EV5 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_MODE_SELECT) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send MS5607B303DLH_Magn address for write */
	I2C_Send7bitAddress(MS5607B_I2C, slAddr, I2C_Direction_Transmitter);
	
	/* Test on EV6 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(MS5607B_I2C, ENABLE);
	
	/* Send the MS5607B303DLH_Magn's internal address to write to */
	I2C_SendData(MS5607B_I2C, ReadAddr);
	
	/* Test on EV8 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send STRAT condition a second time */
	I2C_GenerateSTART(MS5607B_I2C, ENABLE);
	
	/* Test on EV5 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_MODE_SELECT) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* Send MS5607B303DLH address for read */
	I2C_Send7bitAddress(MS5607B_I2C, slAddr, I2C_Direction_Receiver);
	
	/* Test on EV6 and clear it */
	timeout = 50000;
	while(!I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout-->0);
	if(timeout == 0) return 0;
	
	/* While there is data to be read */
	while(NumByteToRead)
	{
	if(NumByteToRead == 1)
	{
	  /* Disable Acknowledgement */
	  I2C_AcknowledgeConfig(MS5607B_I2C, DISABLE);
	
	  /* Send STOP Condition */
	  I2C_GenerateSTOP(MS5607B_I2C, ENABLE);
	}
	
	/* Test on EV7 and clear it */
	if(I2C_CheckEvent(MS5607B_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
	  /* Read a byte from the MS5607B303DLH */
	  *pBuffer = I2C_ReceiveData(MS5607B_I2C);
	
	  /* Point to the next location where the byte read will be saved */
	  pBuffer++;
	
	  /* Decrement the read bytes counter */
	  NumByteToRead--;
	}
	}
	
	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(MS5607B_I2C, ENABLE);
	AHRS_EXIT_CRITICAL();
	return 1;	
}

void MS5607B_SPI_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MS5607B_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(MS5607B_SPI_NSS_Pin_Port,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =MS5607B_SPI_CLK_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(MS5607B_SPI_CLK_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =MS5607B_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(MS5607B_SPI_MOSI_Pin_Port,&GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MS5607B_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(MS5607B_SPI_NSS_Pin_Port,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin	= MS5607B_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MS5607B_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(MS5607B_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = MS5607B_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(MS5607B_SPI, &SPI_InitStructure);

	SPI_Cmd(MS5607B_SPI, ENABLE);	
	MS5607B_SPI_CS_HIGH();
}

u8 MS5607B_GetData_I2C(uint32_t *Databuff)
{
	u8 errStatus=1;
	u8 tmpdata[3];	
	errStatus &= MS5607B_I2C_BufferRead(MS5607B_I2C_ADDRESS, tmpdata, 0x00, 3);
	*Databuff = 0;
	*Databuff = ( (tmpdata[0]<<16) | (tmpdata[1]<<8) | tmpdata[2] );
	return errStatus;
}

u8 MS5607B_StartPressureADC(void)
{
	u8 errStatus=1;
	u8 cmd;
	cmd=CMD_ADC_CONV | CMD_ADC_D1 | CMD_ADC_4096;
   	errStatus &= MS5607B_I2C_WriteCmd(MS5607B_I2C_ADDRESS, cmd);
	return errStatus;
}

u8 MS5607B_StartTemperatureADC(void)
{
	u8 errStatus=1;
	u8 cmd=0x58;
	cmd = CMD_ADC_CONV | CMD_ADC_D2 | CMD_ADC_4096;
   	errStatus &= MS5607B_I2C_WriteCmd(MS5607B_I2C_ADDRESS, cmd);
	return errStatus;
}

u8 MS5607B_Reset(void)
{
	u8 errStatus=1;
	u8 cmd = CMD_RESET;
	errStatus &= MS5607B_I2C_WriteCmd(MS5607B_I2C_ADDRESS, cmd);
	return errStatus;
}

//unsigned char MS5607B_StateUpdate_I2C(uint32_t *PressureD, uint32_t *TemperD)
//{
//	static unsigned char stateofMS5607B=0x00;	
//	
//	static unsigned char convertPressureCNT=0;
//
//	uint32_t tmptime;
//	unsigned char tstate,retval = 0;
//
//	// 是否转换完成
//	if(stateofMS5607B & 0x06)
//	{	
//		tmptime = GetSystikMS5607B();	
//		if( tmptime > 1000)	 	//1000 对应systick计 10ms
//		{
//			stateofMS5607B |= 0x01;
//		}
//	}
//	// 状态机
//	switch (stateofMS5607B)
//	{
//		case 0x00:
//		case 0x01:
//			if(convertPressureCNT>30) 	//气压测量10次启动一次温度测量
//			{
//				convertPressureCNT=0;
//				//触发温度测量
//				tstate=MS5607B_StartTemperatureADC_I2C();
////				if(tstate == 0)
////					return tstate;
//				ResetSystikMS5607B();
//				stateofMS5607B |= 0x04;
//			}
//			else	  
//			{
//				//触发气压测量
//				tstate=MS5607B_StartPressureADC_I2C();
////				if(tstate == 0)
////					return tstate;
//				ResetSystikMS5607B();
//				stateofMS5607B |= 0x02;				
//			}
//			retval=0;
//			break;
//		
//		case 0x05:		//温度测量完成
//			stateofMS5607B = 0x00;
//			tstate=MS5607B_GetData_I2C(TemperD);	  	//得到温度测量原始值
////			if(tstate == 0)
////				return tstate;			
//			tstate = MS5607B_StartPressureADC_I2C();		//并触发气压测量							
////			if(tstate == 0)
////				return tstate;		
//			ResetSystikMS5607B();
//			stateofMS5607B |= 0x02;
//
//			convertPressureCNT=0;
//			retval=0;
//			break;
//
//		case 0x03:	   	//气压测量完成
//			stateofMS5607B = 0x00;
//			tstate=MS5607B_GetData_I2C(PressureD);		//得到气压测量原始值							 
////			if(tstate == 0)
////				return tstate;
//			convertPressureCNT++;
//
//			retval=1;
//			break;
//	
//		default:  		//other impossible condition
//			retval = 0;
//			break;
//	}
//
//	return retval;
//				
//}

u8 MS5607B_GetCaliData(uint16_t *baroCali)
{
	u8 errStatus=1;
	unsigned char calicnt,tmpData[2];//tmpMSB,tmpLSB;

	for (calicnt=0;calicnt<6;calicnt++)	
	{	
		errStatus &= MS5607B_I2C_BufferRead(MS5607B_I2C_ADDRESS, tmpData, 0xA2+(calicnt<<1), 2);
		baroCali[calicnt] = (tmpData[0]<<8)|tmpData[1];
	}	
	return errStatus;
}


/**
  * @brief  get real pressure data.
  * @param  
  			D1 : original pressure data
			D2 : original temperature data					  
  * @retval Real pressure data, 110002=110.002 kPa.
  */
int32_t MS5607B_GetPressure(uint32_t D1, uint32_t D2, uint16_t *baroCali)
{
	float tmpP;
	float dT,TEMP;
	float OFF,SENS;
	float OFF2,SENS2,T2;
		
	//tmpP = (int32_t)( ( ((uint64_t)D1)*C[0])>>36  - (int64_t)(((uint64_t)C[1])<<2) );

						 	    
	dT = D2 - baroCali[4]*(float)(1<<8);
	TEMP = (float)(2000 + ((dT*baroCali[5]))/((float)(1<<23)));

	OFF = baroCali[1]*(float)(1<<17) + (baroCali[3]*(float)dT)/((float)(1<<6));
	SENS = baroCali[0]*(float)(1<<16) + (baroCali[2]*(float)dT)/((float)(1<<7));

	if(TEMP < 2000)
	{		
		T2 = (((float)dT)*dT)/((float)((uint32_t)0x00000001<<31));
		OFF2 = (61*((float)(TEMP-2000))*(TEMP-2000))/((float)(1<<4));
		SENS2 = 2*((float)(TEMP-2000))*(TEMP-2000);
		if(TEMP < -1500)
		{
		 	OFF2 = OFF2 + 15*(TEMP+1500)*(TEMP+1500);
			SENS2 = SENS2 + 8*(TEMP+1500)*(TEMP+1500);
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	TEMP = TEMP-T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	

	tmpP = ( ((float)(D1*SENS))/((float)(1<<21)) - OFF) / ((float)(1<<15));

	return (int32_t)tmpP;	
}

/**
  * @brief  get real temperature data.
  * @param  
  			D1 : original pressure data
			D2 : original temperature data					  
  * @retval Real temperature data, 2000=20.00 degree.
  */										 
int32_t	MS5607B_GetTemperature(uint32_t D1, uint32_t D2, uint16_t *baroCali)
{	
	float dT;
	float tmpTEMP;

//	dT = D2 - (C[4]*0x0001<<8);
//	tmpTEMP = 2000 + (dT*C[5])/(uint32_t)(0x00000001<<23);
	dT = D2 - baroCali[4]*(float)(1<<8);
	tmpTEMP = (float)2000 + dT*(baroCali[5]/((float)(1<<23)));
	
	return (int32_t)tmpTEMP;
}

//u8 MS5607B_UpdataData(MS5607B_DataType *data)
//{
//	static uint32_t PD,TD;
//	u8 state;
//
//#ifdef MS5607B_USE_SPI
//	state = MS5607B_StateUpdate_SPI(&PD, &TD);	
//#endif
//
//#ifdef MS5607B_USE_I2C
//	state = MS5607B_StateUpdate_I2C(&PD, &TD);	
//#endif						  
//	if(state != 0)
//	{
//		data->temperature = MS5607B_GetTemperature(PD,TD);
//		data->pressure = MS5607B_GetPressure(PD,TD);	
//	}
//	return state;
//}
