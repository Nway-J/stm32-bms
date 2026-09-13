/*
 * 文件名称：bsp_usart.c
 *
 * 模块名称：USART1中断接收与环形缓冲驱动
 *
 * 模块职责：
 * 1. 配置PA9/PA10和USART1，波特率为115200；
 * 2. RXNE中断到来时立即读取DR，避免任务轮询不及时造成硬件溢出；
 * 3. 使用64字节静态环形缓冲区保存收到的字节；
 * 4. 让任务通过非阻塞接口逐字节读取缓冲区；
 * 5. 记录接收字节、硬件错误和软件缓冲区满的次数。
 *
 * 数据流：
 * USART1 RXNE中断
 *        -> USART1_RxIRQHandler()
 *        -> 静态环形缓冲区
 *        -> USART1_ReadByteNonBlocking()
 *        -> CLI_Process()
 *
 * 并发规则：
 * 中断是唯一生产者，只修改写入位置；
 * CLI任务是唯一消费者，只修改读取位置；
 * 中断中不执行printf、Flash访问或命令解析。
 */
#include "bsp_usart.h"
#include <stdio.h>
#include "stm32f10x.h"

/*
 * 64字节足以保存多条当前CLI短命令，并且完全使用静态RAM，
 * 不占用当前已经较紧张的FreeRTOS Heap。
 */
#define USART1_RX_BUFFER_SIZE        64U

static uint8_t g_usart1_rx_buffer[USART1_RX_BUFFER_SIZE];

/* head指向下一个写入位置，由USART1中断修改。 */
static volatile uint16_t g_usart1_rx_head = 0U;

/* tail指向下一个读取位置，由CLI任务修改。 */
static volatile uint16_t g_usart1_rx_tail = 0U;

volatile uint32_t g_usart1_rx_byte_count = 0U;
volatile uint32_t g_usart1_rx_hw_error_count = 0U;
volatile uint32_t g_usart1_rx_buffer_overflow_count = 0U;


/* 返回环形缓冲区中的下一个位置。 */
static uint16_t USART1_RxNextIndex(uint16_t index)
{
    index++;

    if (index >= USART1_RX_BUFFER_SIZE)
    {
        index = 0U;
    }

    return index;
}


void USART1_Init(void)
{
	volatile uint32_t clear_value;

	//1.开GPIOA/USART1时钟
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<14);
	
	//2.配置PA9--TX PA10--RX
	GPIOA->CRH &= ~(0XF<<4);
	GPIOA->CRH |= (0XB<<4);
	
	GPIOA->CRH &=  ~(0XF<<8);
	GPIOA->CRH |= (0X4<<8);
	
	//3.配置波特率 115200
	USART1->BRR = 0X0270;
	
	/* 清空软件接收状态和诊断计数。 */
	g_usart1_rx_head = 0U;
	g_usart1_rx_tail = 0U;
	g_usart1_rx_byte_count = 0U;
	g_usart1_rx_hw_error_count = 0U;
	g_usart1_rx_buffer_overflow_count = 0U;

	//4.开启USART/接收/发送使能
	USART1->CR1 |= (1<<2);
	USART1->CR1 |= (1<<3);
	USART1->CR1 |= (1<<13);

	/*
	 * 先读SR再读DR，清除初始化前可能残留的接收和错误状态。
	 */
	clear_value = USART1->SR;
	clear_value = USART1->DR;
	(void)clear_value;

	/*
	 * USART中断不调用FreeRTOS API，但仍使用较低抢占优先级6，
	 * 为以后改用FromISR接口保留安全余量。
	 */
	NVIC_SetPriority(
		USART1_IRQn,
		NVIC_EncodePriority(
			NVIC_PriorityGroup_4,
			6U,
			0U));
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);

	/* 最后开启接收非空中断，避免初始化未完成时进入中断。 */
	USART1->CR1 |= USART_CR1_RXNEIE;
}


void USART1_SendByte(uint8_t data)
{
    // 等待发送数据寄存器为空，避免覆盖上一个尚未发送的字节。
    while ((USART1->SR & (1U << 7)) == 0U)
    {
    }

    // 将新字节写入发送数据寄存器。
    USART1->DR = data;
}


//const--只可被读，不能被修改
void USART1_SendString(const char *str)
{
	while(*str)
	{
		USART1_SendByte((uint8_t)*str++);
	}
}


/*
 * 从静态环形缓冲区尝试读取一个字符。
 *
 * 本函数不再直接读取USART1->DR；DR只由接收中断读取。
 * 缓冲区为空时立即返回0，不阻塞CLI任务。
 */
uint8_t USART1_ReadByteNonBlocking(uint8_t *data)
{
    uint16_t tail;

    if (data == 0)
    {
        return 0U;
    }

    tail = g_usart1_rx_tail;

    if (tail == g_usart1_rx_head)
    {
        return 0U;
    }

    *data = g_usart1_rx_buffer[tail];
    g_usart1_rx_tail = USART1_RxNextIndex(tail);

    return 1U;
}


/*
 * USART1接收中断的板级处理函数。
 *
 * ISR必须先读取SR，再读取DR：
 * 这样既取得接收字节，也清除ORE、FE、NE和PE等硬件错误状态。
 */
void USART1_RxIRQHandler(void)
{
    uint32_t status;
    uint8_t data;
    uint16_t head;
    uint16_t next_head;

    status = USART1->SR;

    if ((status & (USART_SR_ORE |
                   USART_SR_NE |
                   USART_SR_FE |
                   USART_SR_PE)) != 0U)
    {
        g_usart1_rx_hw_error_count++;
    }

    if ((status & USART_SR_RXNE) != 0U)
    {
        data = (uint8_t)USART1->DR;
        head = g_usart1_rx_head;
        next_head = USART1_RxNextIndex(head);

        /*
         * next_head等于tail表示软件缓冲区已满。
         * 丢弃新字节并记录次数，不能覆盖尚未被CLI读取的数据。
         */
        if (next_head == g_usart1_rx_tail)
        {
            g_usart1_rx_buffer_overflow_count++;
            return;
        }

        g_usart1_rx_buffer[head] = data;
        g_usart1_rx_head = next_head;
        g_usart1_rx_byte_count++;
    }
    else if ((status & (USART_SR_ORE |
                        USART_SR_NE |
                        USART_SR_FE |
                        USART_SR_PE)) != 0U)
    {
        /* 没有RXNE但存在错误时，读取DR完成错误状态清除。 */
        data = (uint8_t)USART1->DR;
        (void)data;
    }
}


/* 等待最后一个字节完全离开移位寄存器，用于复位前保证提示已发完。 */
void USART1_WaitSendComplete(void)
{
    while ((USART1->SR & USART_SR_TC) == 0U)
    {
    }
}


int fputc(int ch, FILE *f)
{
    USART1_SendByte((uint8_t)ch);
    return ch;
}


