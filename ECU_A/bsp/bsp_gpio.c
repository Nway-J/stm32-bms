#include "stm32f10x.h"
#include "bsp_gpio.h"


void LED_Init(void)
{
	//1.开GPIOC时钟
	RCC->APB2ENR |= (1<<4);
	
	//2.配置PC13
	GPIOC->CRH &= ~(0xf<<20);
	GPIOC->CRH |= (0x2<<20);
	
}

void KEY_Init(void)
{
    RCC->APB2ENR |= (1 << 3);
    GPIOB->CRL &= ~(0xF << 0);
    GPIOB->CRL |=  (0x8 << 0);
    GPIOB->ODR |= (1 << 0);
    GPIOB->CRL &= ~(0xF << 4);
    GPIOB->CRL |=  (0x8 << 4);
    GPIOB->ODR |= (1 << 1);
}

uint8_t KEY_Read(uint8_t pin)
{
	uint8_t bit = (pin == 0) ? (1<<0) : (1<<1);
	// 检测按下
	if((GPIOB->IDR & bit) == 0)
	{
		for(volatile int i=0;i<10000;i++); // 消抖
		if((GPIOB->IDR & bit) == 0)
		{
			return 1; // 按键按下
		}
	}
	return 0; // 无按键
}















