/* Includes */
#include "ITG3200.h"
#include "HAL_ITG3200.h"
#include "OSConfig.h"
#include "stm32f10x.h"


/**
* @defgroup ITG
* @{
*/

/** @defgroup ITG_I2C_Function
* @{
*/

/**
* @brief  Initializes the I2C peripheral used to drive the ITG
* @param  None
* @retval None
*/
void ITG_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(ITG_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(ITG_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  ITG_I2C_SCL_Pin | ITG_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(ITG_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ITG_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(ITG_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(ITG_I2C, ENABLE);
}

/**
* @brief  Writes one byte to the  ITG.
* @param  slAddr : slave address ITG_A_I2C_ADDRESS or ITG_M_I2C_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the ITG.
* @param  WriteAddr : address of the register in which the data will be written
* @retval None
*/
void ITG_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
  /* Send START condition */
  I2C_GenerateSTART(ITG_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send ITG_Magn address for write */
  I2C_Send7bitAddress(ITG_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the ITG_Magn's internal address to write to */
  I2C_SendData(ITG_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(ITG_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(ITG_I2C, ENABLE);
}

/**
* @brief  Reads a block of data from the ITG.
* @param  slAddr  : slave address ITG_A_I2C_ADDRESS or ITG_M_I2C_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the ITG.
* @param  ReadAddr : ITG's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the ITG ( NumByteToRead >1  only for the Mgnetometer readinf).
* @retval None
*/

void ITG_I2C_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
  /* While the bus is busy */
  while(I2C_GetFlagStatus(ITG_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(ITG_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send ITG_Magn address for write */
  I2C_Send7bitAddress(ITG_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(ITG_I2C, ENABLE);

  /* Send the ITG_Magn's internal address to write to */
  I2C_SendData(ITG_I2C, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(ITG_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send ITG address for read */
  I2C_Send7bitAddress(ITG_I2C, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(ITG_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(ITG_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(ITG_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the ITG */
      *pBuffer = I2C_ReceiveData(ITG_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(ITG_I2C, ENABLE);
}

void ITG_Init(ITG_ConfigTypeDef *ITG_Config_Struct)
{
	u32 i=100000;
	GPIO_InitTypeDef GPIO_InitStructure;

	u8 FS_LPF=ITG_Config_Struct->FS | ITG_Config_Struct->LPFBandwidth;
  	ITG_I2C_ByteWrite(ITG_I2C_ADDRESS,&(ITG_Config_Struct->ClkSel),ITG_POWERMANAGE_REG_ADDR);
	ITG_I2C_ByteWrite(ITG_I2C_ADDRESS,&(ITG_Config_Struct->SamRate),ITG_SR_DIVIDER_REG_ADDR);
	ITG_I2C_ByteWrite(ITG_I2C_ADDRESS,&FS_LPF,ITG_FULL_SCALE_REG_ADDR);
	ITG_I2C_ByteWrite(ITG_I2C_ADDRESS,&(ITG_Config_Struct->IntMode),ITG_INT_CONFIG_ADDR);
	while(i--);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); // 改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能
	RCC_APB2PeriphClockCmd(ITG_DRDY_RCC_Port, ENABLE);	  //GPIOC时钟
	GPIO_InitStructure.GPIO_Pin = ITG_DRDY_Pin ;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ITG_DRDY_Port, &GPIO_InitStructure);
}

u8 ITG_Read_RawData(s16* out)
{
	u8 buffer[6];
	u8 intStatus;
	u8 i;
	
	ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &intStatus, ITG_INT_STATUS_REG_ADDR, 1);
	if(intStatus==0x01)
	{	  
		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, buffer, ITG_XOUT_H_ADDR, 6);
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &buffer[0], ITG_XOUT_H_ADDR, 1);
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &buffer[1], ITG_XOUT_L_ADDR, 1);
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &buffer[2], ITG_YOUT_H_ADDR, 1);
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &buffer[3], ITG_YOUT_L_ADDR, 1);
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &buffer[4], ITG_ZOUT_H_ADDR, 1);
//		ITG_I2C_BufferRead(ITG_I2C_ADDRESS, &buffer[5], ITG_ZOUT_L_ADDR, 1);
		for(i=0;i<3;i++)
		{
			out[i] = buffer[2*i] & 0x00ff;
			out[i] <<= 8;
			out[i] |= buffer[2*i+1];
		}
		return 1;
	}
	return 0;
}
/**
* @brief   Read ITG output register, and calculate the acceleration ACC=SENSITIVITY* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param  out : buffer to store data
* @retval None
*/

void ITG_Read_GyroRate(float* out)
{
	s16 rawData[3];
	u8 i;
	float temp;
	if(1==ITG_Read_RawData(rawData))
	{
		for(i=0;i<3;i++)
		{
			out[i]=-(float)(rawData[i])*0.00121414;
		}
		temp=out[0];
		out[0]=out[1];
		out[1]=temp;	
	}
}

/*
 * @brief  	convert raw data (in byte) to real gyrorate in rad/s
 * @param 	raw : pointer to raw data
 * 			gyr : pointer to output gyrorate
 * @retval	none
 */
void ITG_Raw2Gyro(u8 *raw, float *gyr)
{
	u8 i;
	s16 raw_s16[3];
	float temp;
	
	for(i=0;i<3;i++)
	{
		raw_s16[i] = raw[2*i] & 0x00ff;
		raw_s16[i] <<= 8;
		raw_s16[i] |= raw[2*i+1];
	}	
	
	for(i=0;i<3;i++)
	{
		gyr[i]=-(float)(raw_s16[i])*0.00121414;
	}
	temp=gyr[0];
	gyr[0]=gyr[1];
	gyr[1]=temp;	
}

/*
 * a demo configuration
 */
void ITG3205_Config(void)
{
	ITG_ConfigTypeDef  ITG_InitStructure;
	ITG_InitStructure.ClkSel=ITG_Internal_Clk_Sel;
	ITG_InitStructure.SamRate=ITG_SMPLRT_DIV;
	ITG_InitStructure.FS=ITG_Full_Scale_2000;
	ITG_InitStructure.LPFBandwidth=ITG_LPFilter_Bandwidth_42Hz;
	ITG_InitStructure.IntMode=ITG_Int_Mode;
	ITG_Init(&ITG_InitStructure);
}

/******************* (C) COPYRIGHT 2010 Skyworks  *****END OF FILE****/
