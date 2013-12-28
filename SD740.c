#include "SD740.h"

void SD740_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(SD740_SPI_RCC_Periph,ENABLE);
	RCC_APB2PeriphClockCmd(SD740_SPI_RCC_Port,ENABLE);

	GPIO_InitStructure.GPIO_Pin =SD740_SPI_CLK_Pin | SD740_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SD740_SPI_CLK_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SD740_SPI_NSS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SD740_SPI_NSS_Pin_Port,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin	= SD740_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SD740_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(SD740_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SD740_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SD740_SPI, &SPI_InitStructure);

	SPI_Cmd(SD740_SPI, ENABLE);
	SD740_SPI_CS_HIGH(); 
}

u8 SD740_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(SD740_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SD740_SPI,byte);
	while(SPI_I2S_GetFlagStatus(SD740_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SD740_SPI);
}

u8 SD740_SPI_ReadByte(void)
{
	return (SD740_SPI_SendByte(0xff));
}

void SD740_SPI_ByteWrite(u8 data, u8 WriteAddr)
{
	SD740_SPI_CS_LOW();
	SD740_Delay(10);
	SD740_SPI_SendByte(WriteAddr);
	SD740_SPI_SendByte(data);
	SD740_Delay(50);
	SD740_SPI_CS_HIGH();
	SD740_Delay(20);						
}

void SD740_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u8 i;
	SD740_SPI_CS_LOW();
	SD740_Delay(10);
	SD740_SPI_SendByte(ReadAddr | 0x80);
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i]=SD740_SPI_ReadByte();
	}
	SD740_Delay(50);
	SD740_SPI_CS_HIGH();
	SD740_Delay(20);
}

void SD740_Init(SD740_ConfigTypeDef *SD740_Config_Struct)
{
	SD740_SPI_ByteWrite(SD740_Config_Struct->mode,SD740_STANDBY_MODE_REG);
	SD740_SPI_ByteWrite(SD740_Config_Struct->confContent,SD740_CONF_REG);
}

void SD740_Reset(void)
{
	u8 state=0;
	u8 standbyMode=0;
	//enter standby mode
	SD740_SPI_ByteWrite(SD740_Standby_Mode,SD740_STANDBY_MODE_REG);
	SD740_Delay(10000);
	//check status
	SD740_SPI_BufferRead(&standbyMode,SD740_STANDBY_MODE_REG,1);
	while((standbyMode & 0x01) == 0)
	{
		SD740_SPI_ByteWrite(SD740_Standby_Mode,SD740_STANDBY_MODE_REG);
		SD740_Delay(10000);
		//check status
		SD740_SPI_BufferRead(&standbyMode,SD740_STANDBY_MODE_REG,1);
	}

	SD740_Delay(100000);
	//resume from standby mode
	SD740_SPI_ByteWrite(SD740_Normal_Mode,SD740_STANDBY_MODE_REG);
	SD740_Delay(10000);

	//check status
	SD740_SPI_BufferRead(&standbyMode,SD740_STANDBY_MODE_REG,1);
	while((standbyMode & 0x01) != 0)
	{
		SD740_SPI_ByteWrite(SD740_Normal_Mode,SD740_STANDBY_MODE_REG);
		SD740_Delay(10000);	
		//check status
		SD740_SPI_BufferRead(&standbyMode,SD740_STANDBY_MODE_REG,1);
	}

	SD740_Delay(10000);

	//check AGC and PLL loop
	state=SD740_Read_State();
	while((state & 0x07)!=0x06)
	{
		state=SD740_Read_State();
		SD740_Delay(10000);
	}
}

u8 SD740_Read_PID(u8 ID_No)
{
	u8 pid=0;
	switch(ID_No)
	{
		case 0:
			SD740_SPI_BufferRead(&pid,SD740_ID0_REG,1);
			break;
		case 1:
			SD740_SPI_BufferRead(&pid,SD740_ID1_REG,1);			
			break;
		case 2:
			SD740_SPI_BufferRead(&pid,SD740_ID2_REG,1);			
			break;
		default:
			pid=0xff;
			break;
	}
	return pid;
}

u8 SD740_Read_State(void)
{
	u8 state=0;
	SD740_SPI_BufferRead(&state,SD740_DEVICE_STATE_REG,1);
	return state;
}

void SD740_Read_RawData(s16* out)
{
	u8 buffer[6];
	s16 temp;
	u8 i;
	SD740_SPI_BufferRead(buffer,SD740_RATE_X_H,6);
	for(i=0;i<3;i++)
	{
		out[i]=buffer[i*2] & 0x00ff;
		out[i]<<=8;
		out[i] |= buffer[i*2+1];
	}
	temp = out[0];
	out[0] = out[1];
	out[1] = temp;
	out[2]=-out[2];
}

void SD740_Read_GyroRate(float* out)
{
	u8 i;
	s16 rawData[3];
	SD740_Read_RawData(rawData);
	for(i=0;i<3;i++)
	{	
		out[i]=rawData[i]*0.00054542; //»¡¶ÈÖÆ
	}
}

void SD740_Delay(u32 n)
{
	for(;n>0;n--);
}
