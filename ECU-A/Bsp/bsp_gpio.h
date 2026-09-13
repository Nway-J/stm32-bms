#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "stm32f10x.h"

//声明LED初始化函数
void LED_Init(void);

// 声明按键GPIO初始化函数。
void KEY_Init(void);

// 声明按键读取函数。
uint8_t KEY_Read(uint8_t key_id);


#define LED_ON()    GPIOC->ODR &= ~(1U << 13)
#define LED_OFF()   GPIOC->ODR |=  (1U << 13)
#define LED_Toggle() GPIOC->ODR ^= (1U << 13)

// 定义启动按键的逻辑编号。
#define KEY_START 0

// 定义故障复位按键的逻辑编号。
#define KEY_FAULT_RESET 1

#endif


