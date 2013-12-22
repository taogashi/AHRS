#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f10x.h"

#define MAX_BUFFER_LEN 50
#define AHRS_FRAME_LEN 24
#define SPI_DMA_BUFFER_LEN 26

extern volatile u8 buffer_lock_global;
extern u8 spi_mid_buffer[MAX_BUFFER_LEN];

void SPI_DMA_Config(void);
void SPI_DMA_IT_Config(void);

#endif
