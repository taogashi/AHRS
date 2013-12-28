#include "communication.h"
#include "spi.h"

//u8 byteBuffer[30];
//
//void GetSensorData(SensorDataType *sd)
//{
//	u8 buffer[30];
//	u8 i;
//	u32 k;
//	SPI_CS_LOW();
//	Delay(1000);
//	SPI_ADNS_SendByte(0x51);
//	for(i=0;i<30;i++)
//	{
//	   buffer[i]=SPI_ADNS_ReadByte();
//	   k=1000;
//	   while(k--);
//	}
//	SPI_CS_HIGH();
//
//	u8ToFloat(sd->gyr,buffer,3);
//	u8ToFloat(sd->acc,buffer+12,3);
//	u8ToS16(sd->mag,buffer+24,3);
//}
//
//void SendSensorData(SensorDataType *sd)
//{
//	u8 i;
//	*(float *)(byteBuffer)=sd->gyr[0];
//	*(float *)(byteBuffer+4)=sd->gyr[1];
//	*(float *)(byteBuffer+8)=sd->gyr[2];
//
//	*(float *)(byteBuffer+12)=sd->acc[0];
//	*(float *)(byteBuffer+16)=sd->acc[1];
//	*(float *)(byteBuffer+20)=sd->acc[2];
//
//	*(s16 *)(byteBuffer+24)=sd->mag[0];
//	*(s16 *)(byteBuffer+26)=sd->mag[1];
//	*(s16 *)(byteBuffer+28)=sd->mag[2];
//
//	SPI_CS_LOW();
//	Delay(1000);
//	SPI_ADNS_SendByte(0xAA);
//	Delay(50);
//	SPI_ADNS_SendByte(0xAA);
//	Delay(50);
//	for(i=0;i<30;i++)
//	{
//		SPI_ADNS_SendByte(byteBuffer[i]);
//		Delay(50);
//	}
//	SPI_ADNS_SendByte(0xBB);
//	Delay(50);
//	SPI_ADNS_SendByte(0xBB);
//	Delay(50);
//	SPI_CS_HIGH();
//}

void floatToU8(u8 *dst,float *rsc,int num)
{
	u16 i;
	for(i=0;i<num;i++)
	{
		*(float *)(dst+i*4)=rsc[i];
	}		
}

void u8ToFloat(float *dst,u8 *rsc,int num)
{
	u16 i;
	for(i=0;i<num;i++)
	{
		dst[i]=*(float *)(rsc+i*4);
	}
}

void s16ToU8(u8 *dst,s16 *rsc,int num)
{
	u16 i;
	for(i=0;i<num;i++)
	{
		*(s16 *)(dst+i*2)=rsc[i];
	}	
}

void u8ToS16(s16 *dst,u8 *rsc,int num)
{
	u16 i;
	for(i=0;i<num;i++)
	{
		dst[i]=*(s16 *)(rsc+i*2);
	}
}

void u16ToU8(u8 *dst,u16 *rsc,int num)
{
	u16 i;
	for(i=0;i<num;i++)
	{
		*(u16 *)(dst+i*2)=rsc[i];
	}
}

void u8ToU16(u16 *dst,u8 *rsc,int num)
{
	u16 i;
	for(i=0;i<num;i++)
	{
		dst[i]=*(u16 *)(rsc+i*2);
	}
}
