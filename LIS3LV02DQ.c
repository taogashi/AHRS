/* Includes */
#include "LIS3LV02DQ.h"
#include "HAL_LIS3LV02DQ.h"
#include "stm32f10x.h"

/**
* @defgroup ITG
* @{
*/

u8 LIS3_FS=LIS3_Full_Scale_2g;
/** @defgroup LIS3_I2C_Function
* @{
*/
void LIS3_Delay(u32 n)
{
	for(;n>0;n--);
}

void LIS3_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(LIS3_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(LIS3_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  LIS3_I2C_SCL_Pin | LIS3_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(LIS3_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = LIS3_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(LIS3_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(LIS3_I2C, ENABLE);
}

/**
* @brief  Writes one byte to the  ITG.
* @param  slAddr : slave address LIS3_A_I2C_ADDRESS or LIS3_M_I2C_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the ITG.
* @param  WriteAddr : address of the register in which the data will be written
* @retval None
*/
void LIS3_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
//  iNEMO_ENTER_CRITICAL();
  /* Send START condition */
  I2C_GenerateSTART(LIS3_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send LIS3_Magn address for write */
  I2C_Send7bitAddress(LIS3_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the LIS3_Magn's internal address to write to */
  I2C_SendData(LIS3_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(LIS3_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(LIS3_I2C, ENABLE);
//  iNEMO_EXIT_CRITICAL();  
}


void LIS3_I2C_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
//  iNEMO_ENTER_CRITICAL();
  /* While the bus is busy */
  while(I2C_GetFlagStatus(LIS3_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(LIS3_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send LIS3_Magn address for write */
  I2C_Send7bitAddress(LIS3_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(LIS3_I2C, ENABLE);

  /* Send the LIS3_Magn's internal address to write to */
  I2C_SendData(LIS3_I2C, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(LIS3_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send ITG address for read */
  I2C_Send7bitAddress(LIS3_I2C, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(LIS3_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(LIS3_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(LIS3_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the ITG */
      *pBuffer = I2C_ReceiveData(LIS3_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(LIS3_I2C, ENABLE);
//  iNEMO_EXIT_CRITICAL();
}

void LIS3_SPI_Init(void)
{
   	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(LIS3_SPI_RCC_Periph,ENABLE);
	RCC_APB2PeriphClockCmd(LIS3_SPI_RCC_Port,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	GPIO_InitStructure.GPIO_Pin =LIS3_SPI_CLK_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LIS3_SPI_CLK_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =LIS3_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LIS3_SPI_MOSI_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LIS3_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LIS3_SPI_NSS_Pin_Port,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin	= LIS3_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIS3_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(LIS3_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = LIS3_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(LIS3_SPI, &SPI_InitStructure);

	SPI_Cmd(LIS3_SPI, ENABLE);
	LIS3_SPI_CS_HIGH(); 
}

u8 LIS3_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(LIS3_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(LIS3_SPI,byte);
	while(SPI_I2S_GetFlagStatus(LIS3_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(LIS3_SPI);
}

u8 LIS3_SPI_ReadByte(void)
{
	return (LIS3_SPI_SendByte(0xff));
}

void LIS3_SPI_ByteWrite(u8 data, u8 WriteAddr)
{
	LIS3_SPI_CS_LOW();
	LIS3_Delay(10);
	LIS3_SPI_SendByte(WriteAddr);
	LIS3_SPI_SendByte(data);
	LIS3_Delay(50);
	LIS3_SPI_CS_HIGH();
	LIS3_Delay(20);
}

void LIS3_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u8 i;
	LIS3_SPI_CS_LOW();
	LIS3_Delay(10);
	LIS3_SPI_SendByte(ReadAddr | 0x80);
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i]=LIS3_SPI_ReadByte();
	}
	LIS3_Delay(50);
	LIS3_SPI_CS_HIGH();
	LIS3_Delay(20);
}

u8 LIS3_Read_PID(void)
{
	u8 pid;
#ifdef LIS_USE_SPI
	LIS3_SPI->CR1 &= 0xffbf;
	LIS3_SPI->CR1 |= ~0xfffc;
	LIS3_SPI->CR1 |= ~0xffbf;

	LIS3_SPI_BufferRead(&pid,(LIS3_WHO_AM_I_ADDR & 0xbf),1); //0xBB

	LIS3_SPI->CR1 &= 0xffbf;
	LIS3_SPI->CR1 &= 0xfffc;
	LIS3_SPI->CR1 |= ~0xffbf;
#else
	LIS3_I2C_BufferRead(LIS3_I2C_ADDRESS,&pid, LIS3_WHO_AM_I_ADDR,1);
#endif
	return pid;
}

void LIS3_Reboot(char* offset,char* gain)
{
	u8 RebootStatus=LIS3_Reboot_EN;
	u32 i=10000;
//	u8 buffer[6]={0};
	LIS3_I2C_ByteWrite(LIS3_I2C_ADDRESS,&RebootStatus,LIS3_CTRL_REG2_ADDR);
	while(i--);
//	LIS3_I2C_BufferRead(LIS3_I2C_ADDRESS,buffer,LIS3_OFFSETX_ADDR|0x80,6);
//	for(i=0;i<3;i++)
//		acc_adj[i]=(char)buffer[i];
//	for(i=0;i<3;i++)
//		acc_gain[i]=(char)buffer[i+3];
}

void LIS3_Init(LIS3_ConfigTypeDef *LIS3_Config_Struct)
{
	u32 i=100000;
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 CtrlReg1Content=(LIS3_Config_Struct->powerMode) 
					| (LIS3_Config_Struct->SamRate)
					| (LIS3_Config_Struct->SelfTestStatus)
					| (LIS3_Config_Struct->EnabledAxis);
	u8 CtrlReg2Content=LIS3_Config_Struct->FS | LIS3_Config_Struct->RebootStatus | LIS3_Config_Struct->IntMode;
	u8 CtrlReg3Content=CTRL_REG3_Content;
#ifdef LIS_USE_SPI
	LIS3_SPI->CR1 &= 0xffbf;
	LIS3_SPI->CR1 |= ~0xfffc;
	LIS3_SPI->CR1 |= ~0xffbf;

	LIS3_SPI_ByteWrite(CtrlReg1Content,LIS3_CTRL_REG1_ADDR); 
	LIS3_SPI_ByteWrite(CtrlReg2Content,LIS3_CTRL_REG2_ADDR); 
	LIS3_SPI_ByteWrite(CtrlReg3Content,LIS3_CTRL_REG3_ADDR); 

	LIS3_SPI->CR1 &= 0xffbf;
	LIS3_SPI->CR1 &= 0xfffc;
	LIS3_SPI->CR1 |= ~0xffbf;
#else
  	LIS3_I2C_ByteWrite(LIS3_I2C_ADDRESS,&CtrlReg1Content,LIS3_CTRL_REG1_ADDR);
	LIS3_I2C_ByteWrite(LIS3_I2C_ADDRESS,&CtrlReg2Content,LIS3_CTRL_REG2_ADDR);
	LIS3_I2C_ByteWrite(LIS3_I2C_ADDRESS,&CtrlReg3Content,LIS3_CTRL_REG3_ADDR);
#endif

	LIS3_FS=LIS3_Config_Struct->FS;
	while(i--);

	//配置加速度计数据准备好端口的GPIO，PC8
	RCC_APB2PeriphClockCmd(LIS3_DRDY_RCC_Port, ENABLE);	  //GPIOC时钟
	GPIO_InitStructure.GPIO_Pin = LIS3_DRDY_Pin ;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIS3_DRDY_Port, &GPIO_InitStructure);
}

u8 LIS3_Read_RawData(s16* out)
{
	u8 buffer[6];
	u8 testReady;
	u8 i;
	
	testReady = GPIO_ReadInputDataBit(LIS3_DRDY_Port , LIS3_DRDY_Pin);
	if(testReady != 0)
	{
#ifdef LIS_USE_SPI
		LIS3_SPI->CR1 &= 0xffbf;
		LIS3_SPI->CR1 |= ~0xfffc;
		LIS3_SPI->CR1 |= ~0xffbf;
	
		LIS3_SPI_BufferRead(buffer, LIS3_XOUT_L_ADDR|0x40, 6);
	
		LIS3_SPI->CR1 &= 0xffbf;
		LIS3_SPI->CR1 &= 0xfffc;
		LIS3_SPI->CR1 |= ~0xffbf;
#else
		LIS3_I2C_BufferRead(LIS3_I2C_ADDRESS, buffer, LIS3_XOUT_L_ADDR|0x80, 6);
#endif	  		
		for(i=0;i<3;i++)
		{
			out[i] = buffer[2*i+1]&0x00ff;
			out[i] <<= 8;
			out[i] |= buffer[2*i];
		}
		return 1;
	}
	return 0;
}
/**
函数尽量写成可重入的
*/

void LIS3_Read_Acc(float* out)
{
	s16 rawData[3];
	if(1==LIS3_Read_RawData(rawData))
	{
		if(LIS3_FS==LIS3_Full_Scale_2g)
		{
			out[0]=rawData[1]*0.0095718;///1024*GRAVITY;
			out[1]=rawData[0]*0.0095718;
			out[2]=-rawData[2]*0.0095718;	 
		}
		else if(LIS3_FS==LIS3_Full_Scale_6g)
		{
			out[0]=rawData[1]*0.02882794;///340*GRAVITY;
			out[1]=rawData[0]*0.02882794;
			out[2]=-rawData[2]*0.02882794;	 
		}
	}
}

/*
 * @brief	a demo configuration
 */
void LIS3LV_Config(void)
{
	LIS3_ConfigTypeDef  LIS_InitStructure;
	LIS_InitStructure.powerMode=LIS3_Device_On;
	LIS_InitStructure.SamRate=LIS3_SR_640Hz;
	LIS_InitStructure.SelfTestStatus=LIS3_Self_Test_Disable;
	LIS_InitStructure.EnabledAxis=LIS3_Xaxis_Enable | LIS3_Yaxis_Enable | LIS3_Zaxis_Enable;
	LIS_InitStructure.FS=LIS3_Full_Scale_2g;
	LIS_InitStructure.RebootStatus=LIS3_Reboot_Dis;
	LIS_InitStructure.IntMode=LIS3_DRDY_Enable;
	LIS3_Init(&LIS_InitStructure);
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
