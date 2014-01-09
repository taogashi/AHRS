#include <string.h>

#include "spi.h"
#include "stm32f10x.h"
#include "AHRS.h"
#include "AHRSEKF.h"
#include "OSConfig.h"

u8 spi_mid_buffer[MAX_BUFFER_LEN];
u8 spi_dma_tx_buffer[SPI_DMA_BUFFER_LEN]={0};
u8 spi_dma_rx_buffer[SPI_DMA_BUFFER_LEN]={0};
volatile u8 buffer_lock_global;

void SPI_DMA_Config(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//MOSI PA7 | MISO PA6 | SCK  PA5 | NSS PA4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

	SPI_Cmd(SPI1, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7 ;
	SPI_Init(SPI1, &SPI_InitStructure);

	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(spi_dma_rx_buffer);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)SPI_DMA_BUFFER_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(spi_dma_tx_buffer);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)SPI_DMA_BUFFER_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
	
	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	
	spi_dma_tx_buffer[SPI_DMA_BUFFER_LEN-1] = 0xbb;
	spi_dma_tx_buffer[SPI_DMA_BUFFER_LEN-2] = 0xbb;
}

void SPI_DMA_IT_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Channel2_IRQHandler(void)
{
	/*DMA1 Streamer6 transmit complete*/
	if(DMA_GetITStatus(DMA1_IT_TC2) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC2);
	}	
}

void DMA1_Channel3_IRQHandler(void)
{
	/*DMA1 Streamer6 transmit complete*/
	if(DMA_GetITStatus(DMA1_IT_TC3) != RESET)
	{
		if(buffer_lock_global == 0)
			memcpy(spi_dma_tx_buffer, spi_mid_buffer, AHRS_FRAME_LEN);
		DMA_ClearITPendingBit(DMA1_IT_TC3);
	}	
}

