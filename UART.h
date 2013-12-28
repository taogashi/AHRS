#ifndef _UART_H_
#define _UART_H_

#include <stdio.h>

void USART_Config(void); //串口设置
void USART_IT_Config(void);
int fputc(int ch, FILE *f);    //fputc重定向
int fgetc(FILE *f); //fgetc重定向

//------------RTOS layer-------------------
void vUARTIsrHandler(void * pvParameters);
#endif
