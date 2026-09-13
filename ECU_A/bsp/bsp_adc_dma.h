#ifndef __BSP_ADC_DMA_H
#define __BSP_ADC_DMA_H

#include "stm32f10x.h"

extern __IO uint16_t adc_buf[4];
extern volatile uint8_t g_adc_half_ready;
extern volatile uint8_t g_adc_full_ready;

void ADC_DMA_Init(void);
uint8_t ADC_ReadSafe(uint16_t *ch0, uint16_t *ch1);



#endif

