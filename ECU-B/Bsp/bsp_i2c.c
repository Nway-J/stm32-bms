#include "bsp_i2c.h"
#include "stm32f10x.h"

/* PA4作为SCL，PA3作为SDA。 */
#define I2C_SCL_PIN        4U
#define I2C_SDA_PIN        3U

/* 简单软件延时，用于控制I2C时序。 */
static void I2C1_Delay(void)
{
    volatile uint32_t i;

    for(i = 0U; i < 30U; i++)
    {
        __NOP();
    }
}

/* SCL拉高：开漏模式下实际为释放总线。 */
static void I2C1_SCL_High(void)
{
    GPIOA->BSRR = (1U << I2C_SCL_PIN);
}

/* SCL拉低。 */
static void I2C1_SCL_Low(void)
{
    GPIOA->BRR = (1U << I2C_SCL_PIN);
}

/* SDA拉高：开漏模式下实际为释放总线。 */
static void I2C1_SDA_High(void)
{
    GPIOA->BSRR = (1U << I2C_SDA_PIN);
}

/* SDA拉低。 */
static void I2C1_SDA_Low(void)
{
    GPIOA->BRR = (1U << I2C_SDA_PIN);
}

/* 读取SDA当前实际电平。 */
static uint8_t I2C1_ReadSDA(void)
{
    if((GPIOA->IDR & (1U << I2C_SDA_PIN)) != 0U)
    {
        return 1U;
    }

    return 0U;
}

void I2C1_Init(void)
{
    /* 使能GPIOA时钟。 */
    RCC->APB2ENR |= (1U << 2);

    /* PA4配置为50MHz通用开漏输出。 */
    GPIOA->CRL &= ~(0xFU << 16);
    GPIOA->CRL |=  (0x7U << 16);

    /* PA3配置为50MHz通用开漏输出。 */
    GPIOA->CRL &= ~(0xFU << 12);
    GPIOA->CRL |=  (0x7U << 12);

    /* I2C空闲状态：SCL和SDA均为高电平。 */
    I2C1_SCL_High();
    I2C1_SDA_High();

    I2C1_Delay();
}

void I2C1_Start(void)
{
    /* 先释放SDA和SCL。 */
    I2C1_SDA_High();
    I2C1_SCL_High();

    I2C1_Delay();

    /* SCL为高时，SDA由高变低，产生START。 */
    I2C1_SDA_Low();

    I2C1_Delay();

    /* 拉低SCL，进入数据传输阶段。 */
    I2C1_SCL_Low();

    I2C1_Delay();
}

void I2C1_Stop(void)
{
    /* 先保证SDA为低。 */
    I2C1_SDA_Low();

    I2C1_Delay();

    /* 释放SCL。 */
    I2C1_SCL_High();

    I2C1_Delay();

    /* SCL为高时，SDA由低变高，产生STOP。 */
    I2C1_SDA_High();

    I2C1_Delay();
}

void I2C1_SendData(uint8_t data)
{
    uint8_t i;

    /* I2C数据从最高位开始发送。 */
    for(i = 0U; i < 8U; i++)
    {
        /* 发送当前最高位。 */
        if((data & 0x80U) != 0U)
        {
            I2C1_SDA_High();
        }
        else
        {
            I2C1_SDA_Low();
        }

        I2C1_Delay();

        /* SCL拉高，从机在此期间采样SDA。 */
        I2C1_SCL_High();

        I2C1_Delay();

        /* 一个时钟周期结束。 */
        I2C1_SCL_Low();

        I2C1_Delay();

        /* 下一位移动到最高位。 */
        data <<= 1;
    }

    /* 发送完成后释放SDA，准备读取ACK。 */
    I2C1_SDA_High();
}

uint8_t I2C1_WaitAck(void)
{
    uint8_t ack;

    /* 释放SDA，让从机控制SDA。 */
    I2C1_SDA_High();

    I2C1_Delay();

    /* 第9个时钟拉高。 */
    I2C1_SCL_High();

    I2C1_Delay();

    /* SDA=0表示从机产生ACK。 */
    ack = I2C1_ReadSDA();

    /* 结束ACK时钟。 */
    I2C1_SCL_Low();

    I2C1_Delay();

    if(ack == 0U)
    {
        return 0U;
    }

    return 1U;
}

void I2C1_SendAddr(uint8_t addr_7bit, uint8_t rw)
{
    uint8_t addr_byte;

    /* 7位地址左移，最低位加入读写位。 */
    addr_byte = (uint8_t)((addr_7bit << 1) | (rw & 0x01U));

    /* 发送地址字节。 */
    I2C1_SendData(addr_byte);

    /* 等待从机ACK。 */
    (void)I2C1_WaitAck();
}

void I2C1_WriteBuffer(uint8_t dev_addr, uint8_t *buffer, uint16_t len)
{
    uint16_t i;

    /* 参数保护。 */
    if((buffer == 0) || (len == 0U))
    {
        return;
    }

    /* 产生START。 */
    I2C1_Start();

    /* 发送设备地址，rw=0表示写。 */
    I2C1_SendAddr(dev_addr, 0U);

    /* 连续发送数据。 */
    for(i = 0U; i < len; i++)
    {
        I2C1_SendData(buffer[i]);

        /* 每发送一个字节都等待ACK。 */
        (void)I2C1_WaitAck();
    }

    /* 产生STOP。 */
    I2C1_Stop();
}
