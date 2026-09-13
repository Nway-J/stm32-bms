// bsp_oled.h
#ifndef __BSP_OLED_H
#define __BSP_OLED_H
#include "stm32f10x.h"

extern uint8_t OLED_Buffer[128 * 8];

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);

#endif
