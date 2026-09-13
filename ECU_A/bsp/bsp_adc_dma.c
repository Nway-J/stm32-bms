//ADC+DMA双缓冲驱动（TIM3触发，2通道：电压+温度）
#include "stm32f10x.h"
#include "bsp_adc_dma.h"
#include "bsp_tim.h"


#define ADC_BUF_SIZE    4       // 2通道 x 2组
__IO uint16_t adc_buf[ADC_BUF_SIZE];

volatile uint8_t g_adc_half_ready = 0;
volatile uint8_t g_adc_full_ready = 0;

void ADC_DMA_Init(void)
{
	//1.开启ADC1/DMA1/GPIOA时钟
	RCC->APB2ENR |= (1<<9);
	RCC->AHBENR |= (1<<0);
	RCC->APB2ENR |= (1<<2);
	
	//2.配置PA0/PA1模拟输入
	GPIOA->CRL &= ~(0XF<<0);
	GPIOA->CRL &= ~(0XF<<4);
	
	//3.ADC时钟分频
	RCC->CFGR &= ~(0X3<<14);
	RCC->CFGR |= (0X2<<14);
	
	// 4. 采样时间 239.5周期
    ADC1->SMPR2 |= (0x7 << 0);   // CH0
    ADC1->SMPR2 |= (0x7 << 3);   // CH1
	
	//5.多通道扫描--SCAN
	ADC1->CR1 |= (1<<8);
	
	//6.2个转换
	ADC1->SQR1 |= (1<<20);
	
	//7.转换顺序
	ADC1->SQR3 |= (0<<0);
	ADC1->SQR3 |= (1<<5);
	
	//8.定时器触发 TIM3_TRGO
	ADC1->CR2 &= ~(0X7<<17);
	ADC1->CR2 |= (0X4<<17);//TIM3的TRGO事件 100
	ADC1->CR2 |= (1<<20);//允许外部触发
	
	//9.DMA配置
	DMA1_Channel1->CPAR = (uint32_t) & ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t)adc_buf;
	DMA1_Channel1->CNDTR = ADC_BUF_SIZE;
	
	DMA1_Channel1->CCR |= (1 << 7);     // MINC--地址递增
  DMA1_Channel1->CCR |= (1 << 5);     // CIRC--循环
  DMA1_Channel1->CCR |= (1 << 8);     // PSIZE=16--外设数据宽度16位
  DMA1_Channel1->CCR |= (1 << 10);    // MSIZE=16--存储数据宽度16位

	//10.DMA传输完成中断/半传输中断
	DMA1_Channel1->CCR |= (1<<2);
	DMA1_Channel1->CCR |= (1<<1);
	
	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 0, 1));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	//11.开启DMA
	DMA1_Channel1->CCR |= (1<<0);
	
	//12.ADC允许DMA
	ADC1->CR2 |= (1<<8);
	
	//13.开启ADC
	ADC1->CR2 |= (1<<0);
	
	//14.校准
	ADC1->CR2 |= (1 << 3);
  while(ADC1->CR2 & (1 << 3));
  ADC1->CR2 |= (1 << 2);
  while(ADC1->CR2 & (1 << 2));
  // 第一次转换由TIM3启动后自动触发
}

uint8_t ADC_ReadSafe(uint16_t *ch0, uint16_t *ch1)
{
    if(g_adc_half_ready)
    {
        g_adc_half_ready = 0;
        *ch0 = adc_buf[0];    // Cell电压（前半区）
        *ch1 = adc_buf[1];    // 温度（前半区）
        return 1;
    }
    if(g_adc_full_ready)
    {
        g_adc_full_ready = 0;
        *ch0 = adc_buf[2];    // Cell电压（后半区）
        *ch1 = adc_buf[3];    // 温度（后半区）
        return 1;
    }
    return 0;
}

void DMA1_Channel1_IRQHandler(void)
{
    if(DMA1->ISR & (1 << 2))
    {
        DMA1->IFCR |= (1 << 2);
        g_adc_half_ready = 1;
    }
    if(DMA1->ISR & (1 << 1))
    {
        DMA1->IFCR |= (1 << 1);
        g_adc_full_ready = 1;
    }
}
