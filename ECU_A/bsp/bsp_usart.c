#include "bsp_usart.h"
#include <stdio.h>
#include "stm32f10x.h"


void USART1_Init(void)
{
	//1.开GPIOA/USART1时钟
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<14);
	
	//2.配置PA9--TX PA10--RX
	GPIOA->CRH &= ~(0XF<<4);
	GPIOA->CRH |= (0XB<<4);
	
	GPIOA->CRH &=  ~(0XF<<8);
	GPIOA->CRH |= (0X4<<8);
	
	//3.配置波特率 115200
	USART1->BRR = 0X0270;
	
	//4.开启USART/接收/发送使能
	USART1->CR1 |= (1<<2);
	USART1->CR1 |= (1<<3);
	USART1->CR1 |= (1<<13);
}


void USART1_SendByte(uint8_t data)
{
	//等待发送寄存器空
	while(!(USART1->SR & (1<<7)));
	USART1->DR = data;
}


//const--只可被读，不能被修改
void USART1_SendString(const char *str)
{
	while(*str)
	{
		USART1_SendByte((uint8_t)*str++);
	}
}


int fputc(int ch, FILE *f)
{
    USART1_SendByte((uint8_t)ch);
    return ch;
}












