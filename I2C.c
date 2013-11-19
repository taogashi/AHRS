#include "HAL_I2C.h"
#include "I2C.h"
#include "stm32f10x.h"
#include "ITG3200.h"
#include "LSM303DLH.h"

struct{
	u8 target_addr;
	u8 target_sub_addr;
	u8 last_target_addr;
	u8 num_to_read;
	u8 num_to_write;
	u8 i2c_tx_buffer[I2C_BUFFER_LENGTH];
	u8 i2c_rx_buffer[I2C_BUFFER_LENGTH];
	u8 *p_tx;
	u8 *p_rx;
	/*
		 * i2c_com_stage=0: in write stage
		 * i2c_com_stage=1: in read stage
		 */
	u8 i2c_com_stage;
	u8 i2c_operation_done;
}i2c_com_struct;



void User_I2C_Config(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable I2C and GPIO clocks */
	RCC_APB1PeriphClockCmd(USER_I2C_RCC_Periph, ENABLE);
	RCC_APB2PeriphClockCmd(USER_I2C_RCC_Port, ENABLE);

	/* Configure I2C pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  USER_I2C_SCL_Pin | USER_I2C_SDA_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(USER_I2C_Port, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = USER_I2C_Speed;

	/* Apply I2C configuration after enabling it */
	I2C_Init(USER_I2C, &I2C_InitStructure);

	/* I2C Peripheral Enable */
	I2C_Cmd(USER_I2C, ENABLE);	
	
	i2c_com_struct.num_to_read = 0;
	i2c_com_struct.num_to_write = 0;
}

void User_I2C_IT_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	I2C_ITConfig(USER_I2C, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USER_I2C_EVIRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

u8 User_I2C_ByteWrite(u8 slAddr, u8 data, u8 WriteAddr)
{
//	if(i2c_operation_done == 0)
//		return 0;
//	target_addr = slAddr;
//	num_to_write = 2;
//	num_to_read = 0;
//	i2c_tx_buffer[0] = WriteAddr;
//	i2c_tx_buffer[1] = data;
//	
//	p_tx = i2c_tx_buffer;
//	
////	I2C_ITConfig(USER_I2C, I2C_IT_EVT, ENABLE);
////	I2C_ITConfig(USER_I2C, I2C_IT_ERR, ENABLE);
//	i2c_operation_done = 0;
//	I2C_GenerateSTART(USER_I2C, ENABLE);
	return 1;
}

u8 User_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u8 i;
	static u16 user_trigger=0;
	
	if(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BUSY) == RESET)
	{		
		I2C_AcknowledgeConfig(USER_I2C, ENABLE);
		
		I2C_ITConfig(USER_I2C, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE);
		
		i2c_com_struct.target_addr = slAddr;
		i2c_com_struct.target_sub_addr = ReadAddr;
		
		switch(i2c_com_struct.last_target_addr)
		{
			case ITG_I2C_ADDRESS:
				for(i=0; i<6; i++)
					pBuffer[i] = i2c_com_struct.i2c_rx_buffer[i];
				break;
			case LSM_A_I2C_ADDRESS:
				for(i=0; i<6; i++)
					pBuffer[i+6] = i2c_com_struct.i2c_rx_buffer[i];
				break;
			case LSM_M_I2C_ADDRESS:
				for(i=0; i<6; i++)
					pBuffer[i+12] = i2c_com_struct.i2c_rx_buffer[i];
				break;
			default:
				break;
		}
				
		i2c_com_struct.last_target_addr = slAddr;
		i2c_com_struct.num_to_read = NumByteToRead;
		
		i2c_com_struct.p_rx = i2c_com_struct.i2c_rx_buffer;
		
		i2c_com_struct.i2c_operation_done = 0;
		i2c_com_struct.i2c_com_stage = 0;
		
		I2C_GenerateSTART(USER_I2C, ENABLE);
		user_trigger++;
		return 1;
	}
	
	return 0;	
}

void I2C2_EV_IRQHandler(void)
{
	uint32_t i2c_event;
//	u8 dummy_byte=0;
	static u16 m_mode_select=0;
	static u16 m_trans_mode_selected=0;
	static u16 m_byte_transmitted=0;
	static u16 m_rece_mode_selected=0;
	static u16 m_byte_received=0;
	
	i2c_event = I2C_GetLastEvent(USER_I2C);
	
	switch(i2c_event)
	{
		/*succeed to generate start signal  ****************/
		case I2C_EVENT_MASTER_MODE_SELECT:
			m_mode_select++;
			if(i2c_com_struct.i2c_com_stage == 0)
				I2C_Send7bitAddress(USER_I2C, i2c_com_struct.target_addr, I2C_Direction_Transmitter);
			else
			{
				I2C_Send7bitAddress(USER_I2C, i2c_com_struct.target_addr, I2C_Direction_Receiver);
			}
			break;
		/*address transmitted and is to write **************/
		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
			m_trans_mode_selected++;
			I2C_SendData(USER_I2C, i2c_com_struct.target_sub_addr);
			if(i2c_com_struct.num_to_write == 0)
				I2C_ITConfig(USER_I2C, I2C_IT_BUF, DISABLE);
			break;
		
		/**/
		case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
			if(i2c_com_struct.num_to_write > 0)
			{
				I2C_SendData(USER_I2C, *(i2c_com_struct.p_tx++));
				i2c_com_struct.num_to_write--;
			}
			else
			{
				I2C_ITConfig(USER_I2C, I2C_IT_BUF, DISABLE);
			}
		break;
			
		/*this event only happen on write stage *************/
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
			m_byte_transmitted++;
			if(i2c_com_struct.num_to_read == 0)
			{
				/*stop*/
				I2C_GenerateSTOP(USER_I2C, ENABLE);
				I2C_ITConfig(USER_I2C, I2C_IT_EVT, DISABLE);
				/*end of an i2c communication*/
				i2c_com_struct.i2c_operation_done = 1;
			}
			/*if byte to be send exits*/
			else
			{
				/*generate a restart signal*/
				i2c_com_struct.i2c_com_stage = 1;
				I2C_ITConfig(USER_I2C, I2C_IT_BUF, ENABLE);
				I2C_GenerateSTART(USER_I2C, ENABLE);
			}
			break;
		/*this event only happens on read stage **************/
		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
			m_rece_mode_selected++;
			
			if(i2c_com_struct.num_to_read == 1)
			{
				I2C_AcknowledgeConfig(USER_I2C, DISABLE);
				I2C_GenerateSTOP(USER_I2C, ENABLE);
			}
			break;
		
		/*receive a byte, only happens on read stage **********/
		case I2C_EVENT_MASTER_BYTE_RECEIVED:
			m_byte_received++;
			*(i2c_com_struct.p_rx++) = I2C_ReceiveData(USER_I2C);
			i2c_com_struct.num_to_read--;
			if(i2c_com_struct.num_to_read == 1)
			{
				I2C_AcknowledgeConfig(USER_I2C, DISABLE);
				I2C_GenerateSTOP(USER_I2C, ENABLE);
			}
			break;

		default:
			break;
	}
}

void I2C2_ER_IRQHandler(void)
{
	if(I2C_GetITStatus(USER_I2C, I2C_IT_AF) != RESET)
	{
		I2C_ClearITPendingBit(USER_I2C, I2C_IT_AF);
	}
}
