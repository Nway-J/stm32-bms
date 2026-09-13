/*
 * 文件名称：bsp_usart.h
 *
 * 模块名称：USART1板级收发驱动公共接口
 *
 * 模块职责：
 * 1. 初始化USART1的GPIO、波特率和接收中断；
 * 2. 使用静态环形缓冲区暂存中断收到的字节；
 * 3. 提供阻塞发送和非阻塞读取接口；
 * 4. 提供接收字节、硬件错误和缓冲区溢出诊断计数。
 *
 * 模块边界：
 * 本模块只负责可靠收发字节，不拼接命令、不解析CLI，
 * 不在中断中执行printf、Flash或BMS业务逻辑。
 */
#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Init(void);
void USART1_SendByte(uint8_t data);
void USART1_SendString(const char *str);
uint8_t USART1_ReadByteNonBlocking(uint8_t *data);
void USART1_WaitSendComplete(void);

/* USART1硬件中断入口调用的最小接收处理函数。 */
void USART1_RxIRQHandler(void);

/* 供Keil Watch观察的USART1接收诊断计数。 */
extern volatile uint32_t g_usart1_rx_byte_count;
extern volatile uint32_t g_usart1_rx_hw_error_count;
extern volatile uint32_t g_usart1_rx_buffer_overflow_count;

#endif

