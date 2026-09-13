// 引入STM32F103寄存器定义。
#include "stm32f10x.h"

// 引入蜂鸣器驱动接口。
#include "bsp_buzzer.h"

// 定义TIM1自动重装载值，对应2kHz PWM。
#define BUZZER_PWM_ARR 499U

// 定义蜂鸣器开启时的50%占空比比较值。
#define BUZZER_PWM_DUTY 125U

// 定义蜂鸣器关闭时的恒高电平比较值。
#define BUZZER_PWM_OFF 500U

// 初始化PA8和TIM1_CH1 PWM。
void BUZZER_Init(void)
{
    // 开启GPIOA外设时钟。
    RCC->APB2ENR |= (1U << 2);

    // 开启TIM1外设时钟。
    RCC->APB2ENR |= (1U << 11);

    // 清除PA8原来的GPIO配置。
    GPIOA->CRH &= ~(0xFU << 0);

    // 将PA8配置为2MHz复用推挽输出。
    GPIOA->CRH |= (0xAU << 0);

    // 停止TIM1计数器，避免配置过程中产生异常波形。
    TIM1->CR1 &= ~(1U << 0);

    // 将TIM1预分频器设置为71，使72MHz定时器时钟分频到1MHz。
    TIM1->PSC = 71U;

    // 设置自动重装载值为499，使PWM周期为500us。
    TIM1->ARR = BUZZER_PWM_ARR;

    // 设置TIM1通道1比较值为500，使初始化后PA8保持恒高电平。
    TIM1->CCR1 = BUZZER_PWM_OFF;

    // 清除TIM1通道1输出比较模式配置。
    TIM1->CCMR1 &= ~(0xFFU << 0);

    // 将TIM1通道1配置为PWM模式1。
    TIM1->CCMR1 |= (0x6U << 4);

    // 开启TIM1通道1比较值预装载功能。
    TIM1->CCMR1 |= (1U << 3);

    // 使能TIM1通道1输出。
    TIM1->CCER |= (1U << 0);

    // 开启TIM1自动重装载预装载功能。
    TIM1->CR1 |= (1U << 7);

    // 使能高级定时器TIM1的主输出。
    TIM1->BDTR |= (1U << 15);

    // 产生一次更新事件，使PSC、ARR和CCR配置立即生效。
    TIM1->EGR |= (1U << 0);

    // 启动TIM1计数器。
    TIM1->CR1 |= (1U << 0);
}

// 开启蜂鸣器。
void BUZZER_On(void)
{
    // 将TIM1_CH1占空比设置为50%，产生2kHz方波。
    TIM1->CCR1 = BUZZER_PWM_DUTY;
}

// 关闭蜂鸣器。
void BUZZER_Off(void)
{
    // 将TIM1_CH1设置为恒高电平，使蜂鸣器停止振动。
    TIM1->CCR1 = BUZZER_PWM_OFF;
}

