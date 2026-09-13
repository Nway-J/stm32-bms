#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f10x.h"

void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_SendAddr(uint8_t addr_7bit, uint8_t rw);
void I2C1_SendData(uint8_t data);
uint8_t I2C1_WaitAck(void);
void I2C1_Stop(void);
void I2C1_WriteBuffer(uint8_t dev_addr, uint8_t *buffer, uint16_t len);

#endif
