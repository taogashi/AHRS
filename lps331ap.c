#include "lps331ap.h"
#include "HAL_lps331ap.h"

#define MIX_SPI

void LPS331AP_Delay(u32 n)
{
	for(;n>0;n--);
}

void LPS331AP_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(LPS331AP_SPI_RCC_Periph,ENABLE);
	RCC_APB2PeriphClockCmd(LPS331AP_SPI_RCC_Port,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	GPIO_InitStructure.GPIO_Pin =LPS331AP_SPI_CLK_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LPS331AP_SPI_CLK_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =LPS331AP_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LPS331AP_SPI_MOSI_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LPS331AP_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LPS331AP_SPI_NSS_Pin_Port,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin	= LPS331AP_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LPS331AP_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(LPS331AP_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = LPS331AP_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(LPS331AP_SPI, &SPI_InitStructure);

	SPI_Cmd(LPS331AP_SPI, ENABLE);
	LPS331AP_SPI_CS_HIGH(); 
}

u8 LPS331AP_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(LPS331AP_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(LPS331AP_SPI,byte);
	while(SPI_I2S_GetFlagStatus(LPS331AP_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(LPS331AP_SPI);
}

u8 LPS331AP_SPI_ReadByte(void)
{
	return (LPS331AP_SPI_SendByte(0xff));
}

void LPS331AP_SPI_ByteWrite(u8 data, u8 WriteAddr)
{
	LPS331AP_SPI_CS_LOW();
	LPS331AP_Delay(10);
	LPS331AP_SPI_SendByte(WriteAddr);
	LPS331AP_SPI_SendByte(data);
	LPS331AP_Delay(50);
	LPS331AP_SPI_CS_HIGH();
	LPS331AP_Delay(20);
}

void LPS331AP_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u8 i;
	LPS331AP_SPI_CS_LOW();
	LPS331AP_Delay(10);
	LPS331AP_SPI_SendByte(ReadAddr | 0x80);
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i]=LPS331AP_SPI_ReadByte();
	}
	LPS331AP_Delay(50);
	LPS331AP_SPI_CS_HIGH();
	LPS331AP_Delay(20);
}

//void LPS331AP_Init(LPS331AP_ConfigTypeDef *LPS331AP_Config_Struct)
//{
//#ifdef MIX_SPI
//	LPS331AP_SPI->CR1 &= 0xffbf;
//	LPS331AP_SPI->CR1 |= ~0xfffc;
//	LPS331AP_SPI->CR1 |= ~0xffbf;
//#endif
//
//	//add init code here
//
//#ifdef MIX_SPI
//	LPS331AP_SPI->CR1 &= 0xffbf;
//	LPS331AP_SPI->CR1 &= 0xfffc;
//	LPS331AP_SPI->CR1 |= ~0xffbf;
//#endif
//}

u8 LPS331AP_Read_PID(void)
{
	u8 pid;

#ifdef MIX_SPI
	LPS331AP_SPI->CR1 &= 0xffbf;
	LPS331AP_SPI->CR1 |= ~0xfffc;
	LPS331AP_SPI->CR1 |= ~0xffbf;
#endif

	LPS331AP_SPI_BufferRead(&pid,WHO_AM_I,1); //0xBB

#ifdef MIX_SPI
	LPS331AP_SPI->CR1 &= 0xffbf;
	LPS331AP_SPI->CR1 &= 0xfffc;
	LPS331AP_SPI->CR1 |= ~0xffbf;
#endif

	return pid;
}

void LPS331AP_Simple_Init(void)
{
	u8 resConfigByte=0x6A;
	u8 ctrlReg1Byte=0xF0;

#ifdef MIX_SPI
	LPS331AP_SPI->CR1 &= 0xffbf;
	LPS331AP_SPI->CR1 |= ~0xfffc;
	LPS331AP_SPI->CR1 |= ~0xffbf;
#endif
	//add init code here
	LPS331AP_SPI_ByteWrite(resConfigByte,TP_RESOL);
	LPS331AP_SPI_ByteWrite(ctrlReg1Byte,CTRL_REG1);


#ifdef MIX_SPI
	LPS331AP_SPI->CR1 &= 0xffbf;
	LPS331AP_SPI->CR1 &= 0xfffc;
	LPS331AP_SPI->CR1 |= ~0xffbf;
#endif
}

void LPS331AP_Read_RawData(BaroDataType* bdt)
{
	u8 status=0;
	u8 buffer[3]={0};

#ifdef MIX_SPI
	LPS331AP_SPI->CR1 &= 0xffbf;
	LPS331AP_SPI->CR1 |= ~0xfffc;
	LPS331AP_SPI->CR1 |= ~0xffbf;
#endif
	
	LPS331AP_SPI_BufferRead(&status,STATUS_REG,1);
	if((status & 0x20)!=0) //pressure data ready
	{
		LPS331AP_SPI_BufferRead(buffer,PRESS_OUT_XL|0x40,3);
		bdt->press = 0;
		bdt->press = ((u32)buffer[2]<<16) | ((u32)buffer[1]<<8) | ((u32)buffer[0]);
	}
	if((status & 0x01)!=0)//temperature data ready
	{
		LPS331AP_SPI_BufferRead(buffer,TEMP_OUT_L|0x40,2);
//		LPS331AP_SPI_BufferRead(buffer+1,TEMP_OUT_L+1,1);
		bdt->temp =0;
		bdt->temp = ((u16)buffer[1]<<8) | buffer[0];
	}

#ifdef MIX_SPI
	LPS331AP_SPI->CR1 &= 0xffbf;
	LPS331AP_SPI->CR1 &= 0xfffc;
	LPS331AP_SPI->CR1 |= ~0xffbf;
#endif
}
