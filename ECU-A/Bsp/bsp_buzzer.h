// 防止头文件被重复包含。
#ifndef __BSP_BUZZER_H

// 定义头文件保护宏。
#define __BSP_BUZZER_H

// 引入STM32F103寄存器定义。
#include "stm32f10x.h"

// 声明蜂鸣器初始化函数。
void BUZZER_Init(void);

// 声明蜂鸣器开启函数。
void BUZZER_On(void);

// 声明蜂鸣器关闭函数。
void BUZZER_Off(void);

// 结束头文件保护。
#endif
