#ifndef __BSP_ADC_DMA_H
#define __BSP_ADC_DMA_H

#include "stm32f10x.h"

extern __IO uint16_t adc_buf[4];
extern volatile uint8_t g_adc_half_ready;
extern volatile uint8_t g_adc_full_ready;

extern volatile uint32_t g_dma_half_irq_count;
extern volatile uint32_t g_dma_full_irq_count;

void ADC_DMA_Init(void);
uint8_t ADC_ReadSafe(uint16_t *ch0, uint16_t *ch1);



#endif

