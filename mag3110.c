#include "mag3110.h"
#include "HAL_mag3110.h"

void MAG3110_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(MAG3110_I2C_RCC_Periph, ENABLE);
  RCC_APB2PeriphClockCmd(MAG3110_I2C_RCC_Port, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  MAG3110_I2C_SCL_Pin | MAG3110_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(MAG3110_I2C_Port, &GPIO_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = MAG3110_I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(MAG3110_I2C, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(MAG3110_I2C, ENABLE);
}

void MAG3110_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr)
{
  /* Send START condition */
  I2C_GenerateSTART(MAG3110_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MAG3110_Magn address for write */
  I2C_Send7bitAddress(MAG3110_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the MAG3110_Magn's internal address to write to */
  I2C_SendData(MAG3110_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(MAG3110_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(MAG3110_I2C, ENABLE);
}

void MAG3110_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
  /* While the bus is busy */
  while(I2C_GetFlagStatus(MAG3110_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(MAG3110_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MAG3110_Magn address for write */
  I2C_Send7bitAddress(MAG3110_I2C, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(MAG3110_I2C, ENABLE);

  /* Send the MAG3110_Magn's internal address to write to */
  I2C_SendData(MAG3110_I2C, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(MAG3110_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send ITG address for read */
  I2C_Send7bitAddress(MAG3110_I2C, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(MAG3110_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(MAG3110_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(MAG3110_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the ITG */
      *pBuffer = I2C_ReceiveData(MAG3110_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(MAG3110_I2C, ENABLE);
}

void MAG3110_Init(MAG3110_ConfigTypeDef *MAG3110_Config_Struct)
{
	u8 ctrlReg1=((MAG3110_Config_Struct->sampleRate)<<5) | ((MAG3110_Config_Struct->overSampleRatio)<<3)
				| ((MAG3110_Config_Struct->fastReadMode)<<2) | ((MAG3110_Config_Struct->triggerMeasMode)<<1)
				| ((MAG3110_Config_Struct->powerMode));
	u8 ctrlReg2=MAG3110_CTRL_REG2_BYTE;
	MAG3110_I2C_ByteWrite(MAG3110_ADDR,&ctrlReg1,MAG3110_CTRL_REG1_ADDR);
	MAG3110_I2C_ByteWrite(MAG3110_ADDR,&ctrlReg2,MAG3110_CTRL_REG2_ADDR);
}

u8 MAG3110_Read_PID(void)
{
	u8 pid=0;
	MAG3110_I2C_BufferRead(MAG3110_ADDR,&pid,MAG3110_WHO_AM_I_REG,1);
	return pid;
}

void MAG3110_Read_RawData(s16* out)
{
	u8 buffer[6];
	u8 i;
	MAG3110_I2C_BufferRead(MAG3110_ADDR,buffer,MAG3110_OUT_X_MSB_REG,6);
	for(i=0;i<3;i++)
	{
		out[i]=0;
		out[i] |= buffer[2*i]<<8;
		out[i] |= buffer[2*i+1];
	}
}

void MAG3110_Read_Mag(s16* out)
{
	s16 rawData[3];
	MAG3110_Read_RawData(rawData);

	out[0]=(s16)((-rawData[0]+54)*0.903);
	out[1]=(s16)((rawData[1]+105.5)*0.96142);
	out[2]=(s16)((rawData[2]+2696.8)*0.94086);
}

void MAG3110_Raw2Mag(u8 *raw, s16 *mag)
{
	u8 i;
	s16 raw_s16[3];
	
	for(i=0;i<3;i++)
	{
		raw_s16[i]=0;
		raw_s16[i] |= raw[2*i]<<8;
		raw_s16[i] |= raw[2*i+1];
	}

//	mag[0] = -0.77068*raw_s16[0] + 5494;
//	mag[1] = 0.8859*raw_s16[1] - 23964;
//	mag[2] = 0.70888*raw_s16[2] + 575;
	mag[0] = -raw_s16[0];
	mag[1] = raw_s16[1];
	mag[2] = raw_s16[2];
}

/*
 * a demo configuration
 */
void MAG_Config(void)
{
	MAG3110_ConfigTypeDef MAG3110_InitStructure;
	MAG3110_InitStructure.sampleRate = MAG3110_ODR_80Hz;
	MAG3110_InitStructure.overSampleRatio = MAG3110_OSR_16;
	MAG3110_InitStructure.triggerMeasMode = MAG3110_TRIG_MEAS_DIS;
	MAG3110_InitStructure.fastReadMode = MAG3110_FAST_READ_DIS;
	MAG3110_InitStructure.powerMode = MAG3110_MODE_NORM;

	MAG3110_Init(&MAG3110_InitStructure);
}
