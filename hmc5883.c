#include "hmc5883.h"
#include "hal_hmc5883.h"
#include "OSConfig.h"

void HMC5883_I2C_Init(void)	//初始化I2C总线
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(HMC5883_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(HMC5883_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  HMC5883_I2C_SCL_Pin | HMC5883_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(HMC5883_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = HMC5883_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(HMC5883_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(HMC5883_I2C, ENABLE);
}

void HMC5883_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
  AHRS_ENTER_CRITICAL();
  /* Send START condition */
  I2C_GenerateSTART(HMC5883_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send HMC5883303DLH_Magn address for write */
  I2C_Send7bitAddress(HMC5883_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the HMC5883303DLH_Magn's internal address to write to */
  I2C_SendData(HMC5883_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(HMC5883_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(HMC5883_I2C, ENABLE);
  AHRS_EXIT_CRITICAL();  
}

void HMC5883_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
  AHRS_ENTER_CRITICAL();
  /* While the bus is busy */
  while(I2C_GetFlagStatus(HMC5883_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(HMC5883_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send HMC5883303DLH_Magn address for write */
  I2C_Send7bitAddress(HMC5883_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(HMC5883_I2C, ENABLE);

  /* Send the HMC5883303DLH_Magn's internal address to write to */
  I2C_SendData(HMC5883_I2C, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(HMC5883_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send HMC5883303DLH address for read */
  I2C_Send7bitAddress(HMC5883_I2C, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(HMC5883_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(HMC5883_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(HMC5883_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the HMC5883303DLH */
      *pBuffer = I2C_ReceiveData(HMC5883_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(HMC5883_I2C, ENABLE);
  AHRS_EXIT_CRITICAL();
}

void HMC5883_Init(HMC5883_InitTypeDef *HMC5883_InitStructure)
{
	u32 i=10000;
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 Con_A_content=HMC5883_InitStructure->Sample_Num |HMC5883_InitStructure->Output_Rate | HMC5883_InitStructure->Measure_Mode;
	u8 Con_B_content=HMC5883_InitStructure->Gain_Configure;
	u8 Mode_content=HMC5883_InitStructure->Operating_Mode;

	HMC5883_I2C_ByteWrite(HMC5883_I2C_ADD, &Con_A_content,HMC5883_CON_A);
	HMC5883_I2C_ByteWrite(HMC5883_I2C_ADD, &Con_B_content,HMC5883_CON_B);
  	HMC5883_I2C_ByteWrite(HMC5883_I2C_ADD, &Mode_content,HMC5883_MODE);
	while(i--);

	RCC_APB2PeriphClockCmd(HMC5883_DRDY_RCC_Port, ENABLE);	  //GPIOC时钟
	GPIO_InitStructure.GPIO_Pin = HMC5883_DRDY_Pin ;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(HMC5883_DRDY_Port, &GPIO_InitStructure);
}

void HMC5883_Read_ID(u8* pid)//确认磁罗盘ID
{
	u8 buffer;
	HMC5883_I2C_BufferRead(HMC5883_I2C_ADD,&buffer,HMC5883_ID_A,1);
	*pid=buffer;	
}

void HMC5883_Read_Raw(s16* mag)//读取磁罗盘数据 
{
	u8 i;
	u8 buffer[6];
	u8 status;
//	test_ready = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6);
	HMC5883_I2C_BufferRead(HMC5883_I2C_ADD ,&status,HMC5883_STATUS ,1);
	if(	(status&0x01) != 0)
	{		
		HMC5883_I2C_BufferRead(HMC5883_I2C_ADD, buffer,HMC5883_OUT_X_MSB, 6);	
		for(i=0; i<3; i++)
		{
			mag[i] = buffer[2*i];//&0x00ff;
			mag[i] <<= 8;
			mag[i] |= buffer[2*i+1];
		}
	}
}
