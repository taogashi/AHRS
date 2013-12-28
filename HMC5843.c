/* Includes */
#include "HMC5843.h"
#include "HAL_HMC5843.h"
#include "stm32f10x.h"

/**
* @defgroup ITG			
* @{
*/

/** @defgroup HMC_I2C_Function
* @{
*/

void HMC_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(HMC_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(HMC_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  HMC_I2C_SCL_Pin | HMC_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(HMC_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = HMC_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(HMC_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(HMC_I2C, ENABLE);
}

/**
* @brief  Writes one byte to the  ITG.
* @param  slAddr : slave address HMC_A_I2C_ADDRESS or HMC_M_I2C_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the ITG.
* @param  WriteAddr : address of the register in which the data will be written
* @retval None
*/
void HMC_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
//  iNEMO_ENTER_CRITICAL();
  /* Send START condition */
  I2C_GenerateSTART(HMC_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send HMC_Magn address for write */
  I2C_Send7bitAddress(HMC_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the HMC_Magn's internal address to write to */
  I2C_SendData(HMC_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(HMC_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(HMC_I2C, ENABLE);
//  iNEMO_EXIT_CRITICAL();  
}


void HMC_I2C_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
//  iNEMO_ENTER_CRITICAL();
  /* While the bus is busy */
  while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(HMC_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send HMC_Magn address for write */
  I2C_Send7bitAddress(HMC_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(HMC_I2C, ENABLE);

  /* Send the HMC_Magn's internal address to write to */
  I2C_SendData(HMC_I2C, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(HMC_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send ITG address for read */
  I2C_Send7bitAddress(HMC_I2C, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(HMC_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(HMC_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(HMC_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the ITG */
      *pBuffer = I2C_ReceiveData(HMC_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(HMC_I2C, ENABLE);
//  iNEMO_EXIT_CRITICAL();
}

void HMC_Init(HMC_ConfigTypeDef *HMC_Config_Struct)
{
	u32 i=100000;
	GPIO_InitTypeDef GPIO_InitStructure;
  	HMC_I2C_ByteWrite(HMC_I2C_ADDRESS,&(HMC_Config_Struct->Odr),HMC_CTRL_REGA_ADDR);
	HMC_I2C_ByteWrite(HMC_I2C_ADDRESS,&(HMC_Config_Struct->FS),HMC_CTRL_REGB_ADDR);
	HMC_I2C_ByteWrite(HMC_I2C_ADDRESS,&(HMC_Config_Struct->SampleMode),HMC_MODE_REG_ADDR);
	while(i--);

	//配置加速度计数据准备好端口的GPIO，PC8
	RCC_APB2PeriphClockCmd(HMC_DRDY_RCC_Port, ENABLE);	  //GPIOC时钟
	GPIO_InitStructure.GPIO_Pin = HMC_DRDY_Pin ;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(HMC_DRDY_Port, &GPIO_InitStructure);
}

void HMC_Read_RawData(s16* out)
{
	u8 buffer[6];
	u8 testReady;
	u8 i;
	
	testReady = GPIO_ReadInputDataBit(HMC_DRDY_Port , HMC_DRDY_Pin);
	if(testReady != 0)
	{	  
		HMC_I2C_BufferRead(HMC_I2C_ADDRESS, buffer, HMC_XOUT_H_ADDR, 6);
		for(i=0;i<3;i++)
		{
			out[i] = buffer[2*i]&0x00ff;
			out[i] <<= 8;
			out[i] |= buffer[2*i+1];
		}
	}
}
/**
函数尽量写成可重入的
*/

void HMC_Read_Acc(float* out)
{	 
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
