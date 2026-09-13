#include "stm32f10x.h"
#include "bsp_gpio.h"

void LED_Init(void)
{
    // 1.开GPIOC时钟
    RCC->APB2ENR |= (1 << 4);

    // 2.配置PC13
    GPIOC->CRH &= ~(0xf << 20);
    GPIOC->CRH |= (0x2 << 20);
    GPIOC->ODR |= (1 << 13);
}

#define DASH_LED_ON() GPIOC->ODR &= ~(1 << 13)
#define DASH_LED_OFF() GPIOC->ODR |= (1 << 13)
#define DASH_LED_Toggle() GPIOC->ODR ^= (1 << 13)
