/* Includes */
#include "MXR6202.h"
#include "HAL_MXR6202.h"
#include "OSConfig.h"
#include "stm32f10x.h"


/**
* @defgroup MXR
* @{
*/

/** @defgroup MXR_I2C_Function
* @{
*/

/**
* @brief  Initializes the I2C peripheral used to drive the MXR
* @param  None
* @retval None
*/
void MXR_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(MXR_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(MXR_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  MXR_I2C_SCL_Pin | MXR_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(MXR_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = MXR_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(MXR_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(MXR_I2C, ENABLE);
}

/**
* @brief  Writes one byte to the  MXR.
* @param  slAddr : slave address MXR_A_I2C_ADDRESS or MXR_M_I2C_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MXR.
* @param  WriteAddr : address of the register in which the data will be written
* @retval None
*/
void MXR_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
  /* Send START condition */
  I2C_GenerateSTART(MXR_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MXR_Magn address for write */
  I2C_Send7bitAddress(MXR_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the MXR_Magn's internal address to write to */
  I2C_SendData(MXR_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(MXR_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(MXR_I2C, ENABLE);
}

/**
* @brief  Reads a block of data from the MXR.
* @param  slAddr  : slave address MXR_A_I2C_ADDRESS or MXR_M_I2C_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MXR.
* @param  ReadAddr : MXR's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MXR ( NumByteToRead >1  only for the Mgnetometer readinf).
* @retval None
*/

void MXR_I2C_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
  /* While the bus is busy */
  while(I2C_GetFlagStatus(MXR_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(MXR_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MXR_Magn address for write */
  I2C_Send7bitAddress(MXR_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(MXR_I2C, ENABLE);

  /* Send the MXR_Magn's internal address to write to */
  I2C_SendData(MXR_I2C, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(MXR_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MXR address for read */
  I2C_Send7bitAddress(MXR_I2C, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(MXR_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(MXR_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(MXR_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MXR */
      *pBuffer = I2C_ReceiveData(MXR_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(MXR_I2C, ENABLE);
}

void MXR_Init(MXR_ConfigTypeDef *MXR_Config_Struct)
{
	u8 wake_up_byte=0;
	MXR_I2C_ByteWrite(MXR_I2C_ADDRESS, &wake_up_byte, MXR_ONLY_REG_ADDR);
	
	//75ms should be inserted here
}

u8 MXR_Read_RawData(u16* out)
{
	u8 buffer[5]={0};
	  
	MXR_I2C_BufferRead(MXR_I2C_ADDRESS, buffer, MXR_ONLY_REG_ADDR, 5);

	if(buffer[2]+buffer[4] == 0)
		return 0;

	out[0] = buffer[1] & 0x00ff;
	out[0] <<= 8;
	out[0] |= buffer[2];

	out[1] = buffer[3] & 0x00ff;
	out[1] <<= 8;
	out[1] |= buffer[4];
	
	return 1;
}
/**
* @brief   Read MXR output register, and calculate the acceleration ACC=SENSITIVITY* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param  out : buffer to store data
* @retval None
*/

void MXR_Read_Acc(float* out)
{

}

/******************* (C) COPYRIGHT 2013 Skyworks Embedded System *****END OF FILE****/
