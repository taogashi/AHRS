#include "HAL_bma180.h"
#include "bma180.h"

#define MIX_SPI
/*
*********************************************************************************************************
*                                                VARIABLES
*********************************************************************************************************
*/
u8 SetRange;

/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            FUNCTION
*********************************************************************************************************
*/
void BMA180_Delay(u32 n)
{
	for(;n>0;n--);
}

void BMA180_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(BMA180_SPI_RCC_Periph,ENABLE);
	RCC_APB2PeriphClockCmd(BMA180_SPI_RCC_Port,ENABLE);

	GPIO_InitStructure.GPIO_Pin =BMA180_SPI_CLK_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(BMA180_SPI_CLK_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =BMA180_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(BMA180_SPI_MOSI_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BMA180_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(BMA180_SPI_NSS_Pin_Port,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin	= BMA180_SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BMA180_SPI_MISO_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(BMA180_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = BMA180_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(BMA180_SPI, &SPI_InitStructure);

	SPI_Cmd(BMA180_SPI, ENABLE);
	BMA180_SPI_CS_HIGH(); 
}

u8 BMA180_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(BMA180_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(BMA180_SPI,byte);
	while(SPI_I2S_GetFlagStatus(BMA180_SPI,SPI_I2S_FLAG_RXNE) == RESET);

	return SPI_I2S_ReceiveData(BMA180_SPI);
}

u8 BMA180_SPI_ReadByte(void)
{
	return (BMA180_SPI_SendByte(0xff));
}

void BMA180_SPI_ByteWrite(u8 data, u8 WriteAddr)
{
	BMA180_SPI_CS_LOW();
	BMA180_Delay(10);
	BMA180_SPI_SendByte(WriteAddr);
	BMA180_SPI_SendByte(data);
	BMA180_Delay(50);
	BMA180_SPI_CS_HIGH();
	BMA180_Delay(20);				
}

void BMA180_SPI_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	u8 i;
	BMA180_SPI_CS_LOW();
	BMA180_Delay(10);
	BMA180_SPI_SendByte(ReadAddr | 0x80);
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i]=BMA180_SPI_ReadByte();
	}
	BMA180_Delay(50);
	BMA180_SPI_CS_HIGH();
	BMA180_Delay(20);
}
/**
 * @brief: Init the BMA180
 * @param: void
 * @return: void
*/
void BMA180_Init(BMA180_ConfigTypeDef *BMA180_Config_Struct)
{
	u8 byte;
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef MIX_SPI
	BMA180_SPI->CR1 &= 0xffbf;
	BMA180_SPI->CR1 |= ~0xfffc;
	BMA180_SPI->CR1 |= ~0xffbf;
#endif

	// UNLOCK WRITE
	BMA180_SPI_BufferRead(&byte,CTRL_REG0_ADDR,1);
	byte |= EE_W;                           // Have to set ee_w to	
    	BMA180_SPI_ByteWrite(byte,CTRL_REG0_ADDR);
	
	// DISABLE I2C
	BMA180_SPI_BufferRead(&byte,HIGH_DUR_ADDR,1);
	byte |= DIS_I2C_MASK;                       
	BMA180_SPI_ByteWrite(byte,HIGH_DUR_ADDR);
	
	// DISABLE LOW & HIGH INT
	BMA180_SPI_BufferRead(&byte,HIGH_LOW_INFO_ADDR,1);
	byte &= ~0xFF;                       
	BMA180_SPI_ByteWrite(byte,HIGH_LOW_INFO_ADDR);

	// DISABLE SLOPE & TAPSENS INT
	//default	
	
	// ENABLE NEW DATA INT & DISABLE ADV INT
	byte = 0x02;
	BMA180_SPI_ByteWrite(byte,CTRL_REG3_ADDR);

	//Set Mode
	BMA180_SPI_BufferRead(&byte,TCO_Z_ADDR,1);
	byte &= 0xfc;
	byte |= (BMA180_Config_Struct->mode);
	BMA180_SPI_ByteWrite(byte,TCO_Z_ADDR);

	// Set bandwidth
	BMA180_SPI_BufferRead(&byte,BW_TCS_ADDR,1);
	byte &= 0x0f;
	byte |= (BMA180_Config_Struct->bw)<<4;
	BMA180_SPI_ByteWrite(byte,BW_TCS_ADDR);

	//Set range
	SetRange=BMA180_Config_Struct->range;
	BMA180_SPI_BufferRead(&byte,EE_OFFSET_LSB1_ADDR,1);
	byte &=0xf1;
	byte |= BMA180_Config_Struct->range;
	BMA180_SPI_ByteWrite(byte,EE_OFFSET_LSB1_ADDR);

#ifdef MIX_SPI
	BMA180_SPI->CR1 &= 0xffbf;
	BMA180_SPI->CR1 &= 0xfffc;
	BMA180_SPI->CR1 |= ~0xffbf;	
#endif

	GPIO_InitStructure.GPIO_Pin = BMA180_INT_Pin | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BMA180_INT_Pin_Port,&GPIO_InitStructure);	
}

/*
 * @brief: read product ID to test SPI
 * @para: none
 * @return: if SPI is correct config, it should be 0x03
 */
u8 BMA180_Read_PID(void)
{
	u8 pid;
#ifdef MIX_SPI
	BMA180_SPI->CR1 &= 0xffbf;
	BMA180_SPI->CR1 |= ~0xfffc;
	BMA180_SPI->CR1 |= ~0xffbf;
#endif
	BMA180_SPI_BufferRead(&pid,ID_ADDR,1);
#ifdef MIX_SPI
	BMA180_SPI->CR1 &= 0xffbf;
	BMA180_SPI->CR1 &= 0xfffc;
	BMA180_SPI->CR1 |= ~0xffbf;
#endif
	return pid;
}

/**
 * @brief: Reset the BMA180
 * @param: void
 * @return: void
*/
void BMA180_SoftReset(void)
{
	BMA180_SPI_ByteWrite(RESET_VALUE,RESET_ADDR); // page 48
    //DelayMs(10);                        // wait 10 ms, see page 49
}

void BMA180_Read_RawData(s16* out)
{
	u8 buffer[6];
	u8 i;
	s16 temp;

#ifdef MIX_SPI
	BMA180_SPI->CR1 &= 0xffbf;
	BMA180_SPI->CR1 |= ~0xfffc;
	BMA180_SPI->CR1 |= ~0xffbf;
#endif

	BMA180_SPI_BufferRead(buffer,ACCXLSB_ADDR,6);

#ifdef MIX_SPI
	BMA180_SPI->CR1 &= 0xffbf;
	BMA180_SPI->CR1 &= 0xfffc;
	BMA180_SPI->CR1 |= ~0xffbf;
#endif

	for(i=0;i<3;i++)
	{
		temp=(((buffer[2*i+1]<<8) | (buffer[2*i]))>>2)&0x3fff;
		if(temp >= 0x2000)
			out[i]=temp | 0xc000;
		else
			out[i]=temp;
	}
}

void BMA180_Read_Acc(float* out)
{
	s16 rawData[3];

	BMA180_Read_RawData(rawData);
	switch(SetRange)
	{
		case RANGE_1G:
			out[0]=-(float)rawData[1]*0.0012758;
			out[1]=(float)rawData[0]*0.0012758;
			out[2]=(float)rawData[2]*0.0012758;
			break;
		case RANGE_1_5G:
			out[0]=-(float)rawData[1]*0.0018647;
			out[1]=(float)rawData[0]*0.0018647;
			out[2]=(float)rawData[2]*0.0018647;
			break;
		case RANGE_2G:
			out[0]=-(float)rawData[1]*0.0024535;
			out[1]=(float)rawData[0]*0.0024535;
			out[2]=(float)rawData[2]*0.0024535;
			break;
		case RANGE_3G:
			out[0]=-(float)rawData[1]*0.0037293;
			out[1]=(float)rawData[0]*0.0037293;
			out[2]=(float)rawData[2]*0.0037293;
			break;
		case RANGE_4G:
			out[0]=-(float)rawData[1]*0.004907;
			out[1]=(float)rawData[0]*0.004907;
			out[2]=(float)rawData[2]*0.004907;
			break;
		case RANGE_8G:
			out[0]=-(float)rawData[1]*0.0097159;
			out[1]=(float)rawData[0]*0.0097159;
			out[2]=(float)rawData[2]*0.0097159;
			break;
		case RANGE_16G:
			out[0]=-(float)rawData[1]*0.01943172;
			out[1]=(float)rawData[0]*0.01943172;
			out[2]=(float)rawData[2]*0.01943172;
			break;
		default:
			break;
	}
}

/**
 * @brief: Get sample axe X
 * @param: void
 * @return: short sample
*/
short Accel_getX()
{
	return Accel_get(ACCELERATION_X);
//	DBPRINTF("Acceleration X = %d\n", ret);	
}

/**
 * @brief: Get sample axe Y
 * @param: void
 * @return: short sample
 */
short Accel_getY()
{
	return Accel_get(ACCELERATION_Y);
//	DBPRINTF("Acceleration Y = %d\n", ret);
}

/**
 * @brief: Get sample axe Z
 * @param: void
 * @return: short sample
 */
short Accel_getZ()
{
	return Accel_get(ACCELERATION_Z);
//	DBPRINTF("Acceleration Z = %d\n\n", ret);
}

/**
 * @brief: Get sample axe x
 * @param: TAxe select the axe
 * @return: short sample
 */
short Accel_get(TAxe axe)
{
	
	unsigned char rx_lsb, rx_msb, txDataLsb, txDataMsb;
	unsigned short temp;
	short sample;

	switch(axe){
		case ACCELERATION_X:
			txDataLsb=ACCXLSB_ADDR;
			txDataMsb=ACCXMSB_ADDR;
			break;
		case ACCELERATION_Y:
			txDataLsb=ACCYLSB_ADDR;
			txDataMsb=ACCYMSB_ADDR;
			break;
		case ACCELERATION_Z:
			txDataLsb=ACCZLSB_ADDR;
			txDataMsb=ACCZMSB_ADDR;
			break;
		default:
			return 0;
	}	
	BMA180_SPI_BufferRead(&rx_lsb,txDataLsb,1);
	BMA180_SPI_BufferRead(&rx_msb,txDataMsb,1);

	temp = (((rx_msb << 8) | (rx_lsb))>>2)&0x3FFF;
	if(temp >= 0x2000)
		sample = temp | 0xC000;
	else
		sample = temp;

	//DBPRINTF("\r\naccel = %f\tresult = 0x%x\t lsb = 0x%x\t msb = 0x%x", sample*0.00025, sample, rx_lsb, rx_msb);
	
	return sample;
}

