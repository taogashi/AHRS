#include "MS5607B.h"
#include "HAL_MS5607B.h"
#include "OSConfig.h"

void MS5607B_Delay(u32 n)
{
	for(;n>0;n--);
}

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
	
	RCC_APB2PeriphClockCmd(MS5607B_SPI_RCC_Port, ENABLE);
	RCC_APB1PeriphClockCmd(MS5607B_SPI_RCC_Periph, ENABLE);
	
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
	
	GPIO_InitStructure.GPIO_Pin	= MS5607B_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MS5607B_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

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

u8 MS5607B_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(MS5607B_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(MS5607B_SPI,byte);
	while(SPI_I2S_GetFlagStatus(MS5607B_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(MS5607B_SPI);
}

u8 MS5607B_SPI_ReadByte(void)
{
	return (MS5607B_SPI_SendByte(0xff));
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

u8 MS5607B_StartPressureADC(unsigned char OSR)
{
#ifndef MS5607B_USE_SPI
	u8 errStatus=1;
	u8 cmd;
	cmd=CMD_ADC_CONV | CMD_ADC_D1 | CMD_ADC_4096;
   	errStatus &= MS5607B_I2C_WriteCmd(MS5607B_I2C_ADDRESS, cmd);
	return errStatus;
#else
	u8 cmd = CMD_ADC_CONV_PRES | OSR;
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(cmd);
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	return 1;
#endif
}

u8 MS5607B_StartTemperatureADC(unsigned char OSR)
{
#ifndef MS5607B_USE_SPI
	u8 errStatus=1;
	u8 cmd=0x58;
	cmd = CMD_ADC_CONV | CMD_ADC_D2 | CMD_ADC_4096;
   	errStatus &= MS5607B_I2C_WriteCmd(MS5607B_I2C_ADDRESS, cmd);
	return errStatus;
#else
	u8 cmd = CMD_ADC_CONV_TEMP | OSR;
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(cmd);
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	return 1;
#endif
}

u8 MS5607B_Reset(void)
{
#ifndef MS5607B_USE_SPI
	u8 errStatus=1;
	u8 cmd = CMD_RESET;
	errStatus &= MS5607B_I2C_WriteCmd(MS5607B_I2C_ADDRESS, cmd);
	return errStatus;
#else
	u8 cmd = CMD_RESET;
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(cmd);
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	return 1;
#endif
}

u8 MS5607B_GetCaliData(MS5607B_CaliData *CaliStructure)
{
#ifndef MS5607B_USE_SPI
	u8 errStatus=1;
	unsigned char calicnt,tmpData[2];//tmpMSB,tmpLSB;

	for (calicnt=0;calicnt<6;calicnt++)	
	{	
		errStatus &= MS5607B_I2C_BufferRead(MS5607B_I2C_ADDRESS, tmpData, 0xA2+(calicnt<<1), 2);
		baroCali[calicnt] = (tmpData[0]<<8)|tmpData[1];
	}	
	return errStatus;
#else
	u8 byteBuffer[2]={0};

	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(CMD_PROM_RD_C1);
	byteBuffer[0] = MS5607B_SPI_ReadByte();
	byteBuffer[1] = MS5607B_SPI_ReadByte();
	CaliStructure->C1 = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(CMD_PROM_RD_C2);
	byteBuffer[0] = MS5607B_SPI_ReadByte();
	byteBuffer[1] = MS5607B_SPI_ReadByte();
	CaliStructure->C2 = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(CMD_PROM_RD_C3);
	byteBuffer[0] = MS5607B_SPI_ReadByte();
	byteBuffer[1] = MS5607B_SPI_ReadByte();
	CaliStructure->C3 = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(CMD_PROM_RD_C4);
	byteBuffer[0] = MS5607B_SPI_ReadByte();
	byteBuffer[1] = MS5607B_SPI_ReadByte();
	CaliStructure->C4 = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(CMD_PROM_RD_C5);
	byteBuffer[0] = MS5607B_SPI_ReadByte();
	byteBuffer[1] = MS5607B_SPI_ReadByte();
	CaliStructure->C5 = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);
	MS5607B_SPI_SendByte(CMD_PROM_RD_C6);
	byteBuffer[0] = MS5607B_SPI_ReadByte();
	byteBuffer[1] = MS5607B_SPI_ReadByte();
	CaliStructure->C6 = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	
//	MS5607B_SPI_SendByte(CMD_PROM_RD_CRC);
//	byteBuffer[0] = MS5607B_SPI_ReadByte();
//	byteBuffer[1] = MS5607B_SPI_ReadByte();
//	CaliStructure->CR = ((0xffff & byteBuffer[0])<<8)| byteBuffer[1];
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
	return 1;
#endif
}

u8 MS5607B_ReadADC(uint32_t *Databuff)
{
	u8 buffer[3];
	u8 i;
	*Databuff = 0;
#ifndef MS5607B_USE_SPI

#else
	MS5607B_SPI_CS_LOW();
	MS5607B_Delay(5);	
	MS5607B_SPI_SendByte(CMD_ADC_READ);
	
	for(i=0; i<3; i++)
	{
		*Databuff <<= 8;
		buffer[i] = MS5607B_SPI_ReadByte();
		*Databuff |= buffer[i];
	}
	
	MS5607B_Delay(5);
	MS5607B_SPI_CS_HIGH();
		
	return 1;
#endif
}

/**
  * @brief  get real pressure data.
  * @param  
  			D1 : original pressure data
			D2 : original temperature data					  
  * @retval Real pressure data, 110002=110.002 kPa.
  */
int32_t MS5607B_GetPressure(MS5607B_ProcData *midVal,uint32_t D1,MS5607B_CaliData *CaliStructure)
{
	double OFF;
	double SENS;
	double T2;
	double OFF2;
	double SENS2;
	
	double P;

	OFF = (double)CaliStructure->C2*((uint32_t)1<<17)+ (double)CaliStructure->C4 * midVal->dT * 0.015625;
	SENS = (double)CaliStructure->C1*((uint32_t)1<<16) + (double)CaliStructure->C3 * midVal->dT * 0.0078125;

	if(midVal->TEMP < 2000)
	{		
		T2 = (double)midVal->dT*midVal->dT*0.0000000004656612873077392578125;
		OFF2 = 61*((double)midVal->TEMP-2000)*(midVal->TEMP-2000)*0.0625;
		SENS2 = 2*((double)midVal->TEMP-2000)*(midVal->TEMP-2000);
		if(midVal->TEMP < -1500)
		{
		 	OFF2 = OFF2 + 15*((double)midVal->TEMP+1500)*(midVal->TEMP+1500);
			SENS2 = SENS2 + 8*((double)midVal->TEMP+1500)*(midVal->TEMP+1500);
		}
	}
	else
	{
		T2 = 0.0;
		OFF2 = 0.0;
		SENS2 = 0.0;
	}

	midVal->TEMP = midVal->TEMP-(int32_t)T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	

	P = (D1*SENS*0.000000476837158203125-OFF)*0.000030517578125;

	return (int32_t)P;	
}

/**
  * @brief  get real temperature data.
  * @param  
  			D1 : original pressure data
			D2 : original temperature data					  
  * @retval Real temperature data, 2000=20.00 degree.
  */										 
int32_t	MS5607B_GetTemperature(MS5607B_ProcData *midVal,uint32_t D2,MS5607B_CaliData *CaliStructure)
{	
	midVal->dT = D2 - ((int32_t)CaliStructure->C5<<8);
	midVal->TEMP = (int32_t)2000 + ((midVal->dT*(int64_t)CaliStructure->C6)>>23);
	
	return midVal->TEMP;
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
