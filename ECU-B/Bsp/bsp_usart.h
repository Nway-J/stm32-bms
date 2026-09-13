#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Init(void);
void USART1_SendByte(uint8_t data);
void USART1_SendString(const char *str);

#endif

