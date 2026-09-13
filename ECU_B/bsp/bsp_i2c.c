// bsp_i2c.c — I2C1驱动（硬件I2C，100kHz）
#include "bsp_i2c.h"
#include "stm32f10x.h"

void I2C1_Init(void)
{
    RCC->APB2ENR |= (1 << 3);  // GPIOB
    RCC->APB1ENR |= (1 << 21); // I2C1

    // PB6=SCL, PB7=SDA 复用开漏50MHz
    GPIOB->CRL &= ~(0xFU << 24);
    GPIOB->CRL |= (0xFU << 24);
    GPIOB->CRL &= ~(0xFU << 28);
    GPIOB->CRL |= (0xFU << 28);

    I2C1->CR1 |= (1 << 15); // SWRST
    I2C1->CR1 &= ~(1 << 15);

    I2C1->CR2 |= (36 << 0);   // FREQ=36
    I2C1->CCR |= (180 << 0);  // 标准模式100kHz
    I2C1->TRISE |= (37 << 0); // TRISE=37
    I2C1->CR1 |= (1 << 0);    // PE=1
}

void I2C1_Start(void)
{
    while (I2C1->SR2 & (1 << 1))
        ;
    I2C1->CR1 |= (1 << 8);
    while (!(I2C1->SR1 & (1 << 0)))
        ;
}

void I2C1_SendAddr(uint8_t addr_7bit, uint8_t rw)
{
    uint8_t addr_byte = (addr_7bit << 1) | rw;
    I2C1->DR = addr_byte;
    while (!(I2C1->SR1 & (1 << 1)))
        ;
    uint32_t tmp = I2C1->SR1;
    tmp = I2C1->SR2;
    (void)tmp;
}

void I2C1_SendData(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE))
        ;

    I2C1->DR = data;

    while (!(I2C1->SR1 & I2C_SR1_BTF))
        ;
}

void I2C1_Stop(void) { I2C1->CR1 |= (1 << 9); }

void I2C1_WriteBuffer(uint8_t dev_addr, uint8_t *buffer, uint16_t len)
{
    I2C1_Start();
    I2C1_SendAddr(dev_addr, 0);
    for (uint16_t i = 0; i < len; i++)
        I2C1_SendData(buffer[i]);

    while (!(I2C1->SR1 & I2C_SR1_BTF))
        ;

    I2C1_Stop();
}
