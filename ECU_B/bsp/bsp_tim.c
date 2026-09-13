#include "stm32f10x.h"
#include "bsp_tim.h"

static volatile uint32_t g_sys_tick = 0;

void TIM3_Init(void)
{
    // 1.开启TIM3时钟
    RCC->APB1ENR |= (1 << 1);

    // 2.配置时间 1ms
    TIM3->PSC = 71;
    TIM3->ARR = 999;

    // 3.产生更新事件
    TIM3->CR2 &= ~(0X7 << 4);
    TIM3->CR2 |= (0X2 << 4); // MMS--更新事件（TRGO）

    // 运行中断，中断优先级最高
    TIM3->DIER |= (1 << 0);

    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 0, 0));
    NVIC_EnableIRQ(TIM3_IRQn);

    // 使能计数器
    TIM3->CR1 |= (1 << 0);
}

uint32_t GetTick(void) { return g_sys_tick; }

void TIM3_IRQHandler(void)
{
    // 1.等待UIF=1 更新中断标志
    if (TIM3->SR & (1 << 0))
    {
        // 清除更新中断标志位
        TIM3->SR &= ~(1 << 0);

        // g_sys_tick ++
        g_sys_tick++;
    }
}
