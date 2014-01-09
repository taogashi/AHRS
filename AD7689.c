#include "AD7689.h"
#include "HAL_AD7689.h"

void AD7689_Delay(u32 n)
{
	for(;n>0;n--);
}

void AD7689_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(AD7689_SPI_RCC_Periph,ENABLE);
	RCC_APB2PeriphClockCmd(AD7689_SPI_RCC_Port,ENABLE);
	RCC_APB2PeriphClockCmd(AD7689_SPI_NSS_Pin_RCC_Port,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin =AD7689_SPI_CLK_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(AD7689_SPI_CLK_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =AD7689_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(AD7689_SPI_MOSI_Pin_Port,&GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = AD7689_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD7689_SPI_NSS_Pin_Port,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin	= AD7689_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(AD7689_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(AD7689_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = AD7689_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(AD7689_SPI, &SPI_InitStructure);

	SPI_Cmd(AD7689_SPI, ENABLE);
	AD7689_SPI_CS_HIGH(); 
}

u8 AD7689_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(AD7689_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(AD7689_SPI,byte);
	while(SPI_I2S_GetFlagStatus(AD7689_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(AD7689_SPI);
}

u8 AD7689_SPI_ReadByte(void)
{
	return (AD7689_SPI_SendByte(0xff));
}

void AD7689_Read_Gyro_Raw(u16 *out)
{
	u8 buffer[12];

	u16 CFGByte=AD7689_CFG_UPDATE | AD7689_IC_Uni2GND
				| AD7689_IN0 | AD7689_LPFBW_1 |	AD7689_REFSEL_EX_TDIS
				| AD7689_SEQ_DIS | AD7689_RB_DIS;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[0] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[1] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
//	buffer[2] = AD7689_SPI_ReadByte();
//	buffer[3] = AD7689_SPI_ReadByte();
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(150);

	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN2;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[4] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[5] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
//	buffer[6] = AD7689_SPI_ReadByte();
//	buffer[7] = AD7689_SPI_ReadByte();
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(150);

	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN4;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[8] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[9] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
//	buffer[10] = AD7689_SPI_ReadByte();
//	buffer[11] = AD7689_SPI_ReadByte();
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(150);

	out[0]=0;
	out[0] = ((u16)buffer[0])<<8;
	out[0] |= (u16)buffer[1];

	out[1]=0;
	out[1] = ((u16)buffer[4])<<8;
	out[1] |= (u16)buffer[5];

	out[2]=0;
	out[2] = ((u16)buffer[8])<<8;
	out[2] |= (u16)buffer[9];
}

void AD7689_Read_All_Raw(u16 *out)
{
	u8 buffer[12];
	u8 i;

	//channel 0
	u16 CFGByte=AD7689_CFG_UPDATE | AD7689_IC_Uni2GND
				| AD7689_IN0 | AD7689_LPFBW_1 |	AD7689_REFSEL_EX_TDIS
				| AD7689_SEQ_DIS | AD7689_RB_DIS;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[0] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[1] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(60);

	//channel 1
	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN1;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[2] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[3] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(60);

	//channel 2
	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN2;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[4] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[5] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(60);

	//channel 4
	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN4;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[6] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[7] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(60);
	
	//channel 5
	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN5;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[8] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[9] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(60);
	
	//channel 6
	CFGByte &= ~AD7689_INx_MASK;
	CFGByte |= AD7689_IN6;
	AD7689_SPI_CS_LOW();
	AD7689_Delay(5);
	buffer[10] = AD7689_SPI_SendByte((u8)((CFGByte & 0xff00)>>8));
	buffer[11] = AD7689_SPI_SendByte((u8)(CFGByte & 0x00ff));
	AD7689_Delay(5);
	AD7689_SPI_CS_HIGH();
	AD7689_Delay(60);

	for(i=0;i<6;i++)
	{
		out[i] = 0;
		out[i] = ((u16)buffer[2*i])<<8;
		out[i] |= (u16)buffer[2*i+1];
	}
}

void AD7689_Read_Gyro(float *out)
{
	u16 rawData[3];
	AD7689_Read_Gyro_Raw(rawData);
	out[0]=-(float)rawData[0]*0.0002141080;
	out[1]=-(float)rawData[2]*0.0002129769;
	out[2]=(float)rawData[1]*0.0002219712;
}

void AD7689_Read_Gyro_Acc(float *gyr,float *acc)
{
	u16 rawData[6];
	AD7689_Read_All_Raw(rawData);
	
	acc[0] = 0.00041667*GRAVITY*(rawData[0]-26830);
	acc[1] = -0.00041667*GRAVITY*(rawData[1]-26830);
	acc[2] = -0.00041667*GRAVITY*(rawData[5]-26830);
	
	gyr[0] = 0.00022193345*(rawData[4] - 33090);
	gyr[1] = 0.00022193345*(rawData[3] - 32480);
	gyr[2] = 0.00022193345*(rawData[2] - 33600);
}
