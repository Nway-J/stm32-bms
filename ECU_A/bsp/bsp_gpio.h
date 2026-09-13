#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "stm32f10x.h"

void LED_Init(void);

void KEY_Init(void);

uint8_t KEY_Read(uint8_t pin);

#define LED_ON()    GPIOC->ODR &= ~(1 << 13)
#define LED_OFF()   GPIOC->ODR |=  (1 << 13)
#define LED_Toggle() GPIOC->ODR ^= (1 << 13)

#endif


