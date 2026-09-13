#include "bsp_can.h"
#include "can_protocol.h"
#include "stm32f10x.h"
#include "app_dash.h"
#include "bsp_usart.h"

// CAN初始化
void CAN1_Init(void)
{
    // 1.开GPIOA/AFIO复用/CAN1时钟
    RCC->APB2ENR |= (1 << 2);
    RCC->APB2ENR |= (1 << 0);
    RCC->APB1ENR |= (1 << 25);

    // 2.配置PA11-CAN_RX上拉输入 / PA12--CAN_TX复用推挽
    GPIOA->CRH &= ~(0xF << 12);
    GPIOA->CRH |= (0x8 << 12);
    GPIOA->ODR |= (1 << 11);

    GPIOA->CRH &= ~(0xF << 16);
    GPIOA->CRH |= (0xB << 16);

    // 3.进入初始化模式
    CAN1->MCR |= (1 << 0); // INRQ初始化请求
    while ((CAN1->MSR & (1 << 0)) == 0)
        ; // INAK初始确认

    // 离线恢复
    CAN1->MCR |= (1 << 6);

    CAN1->IER |= CAN_IER_FMPIE0;

    // 4.配置位时序 波特率500kbps
    CAN1->BTR = 0;

    CAN1->BTR |= (0 << 24);  // SJW=1TQ->0
    CAN1->BTR |= (3 << 20);  // BS2=4TQ->3
    CAN1->BTR |= (12 << 16); // BS1=13TQ->12
    CAN1->BTR |= (3 << 0);   // BRP=4TQ->3

    // 5.配置过滤器

    // 当前过滤器配置
    CAN1->FMR |= 1;
    CAN1->FM1R &= ~(1 << 0);          // 屏蔽位模式
    CAN1->FS1R |= (1 << 0);           // 32位
    CAN1->sFilterRegister[0].FR1 = 0; // ID=0
    CAN1->sFilterRegister[0].FR2 = 0; // MASK=0 → 全部接收
    CAN1->FA1R |= (1 << 0);
    CAN1->FFA1R &= ~(1 << 0);
    CAN1->FMR &= ~1;

    // 6. 退出初始化
    CAN1->MCR &= ~(1 << 1);
    CAN1->MCR &= ~(1 << 0);
    while (CAN1->MSR & (1 << 0))
        ;

    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 1, 0));

    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

uint8_t CAN1_Send(CAN_Frame_t *frame) { return 0; } // 仪表节点只收不发

uint8_t CAN1_Receive(CAN_Frame_t *frame)
{
    uint32_t temp;

    /* FIFO0无数据 */
    if ((CAN1->RF0R & 0x03) == 0)
    {
        return 0;
    }

    /* FIFO溢出 */
    if (CAN1->RF0R & (1 << 4))
    {
        CAN1->RF0R |= (1 << 4);
    }

    /* 仅读取标准数据帧 */
    if (CAN1->sFIFOMailBox[0].RIR & (1 << 2))
    {
        CAN1->RF0R |= (1 << 5);
        return 0;
    }

    /* 远程帧 */
    if (CAN1->sFIFOMailBox[0].RIR & (1 << 1))
    {
        CAN1->RF0R |= (1 << 5);
        return 0;
    }

    frame->id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

    frame->dlc = CAN1->sFIFOMailBox[0].RDTR & 0x0F;

    if (frame->dlc > 8)
    {
        frame->dlc = 8;
    }

    temp = CAN1->sFIFOMailBox[0].RDLR;

    frame->data[0] = temp & 0xFF;
    frame->data[1] = (temp >> 8) & 0xFF;
    frame->data[2] = (temp >> 16) & 0xFF;
    frame->data[3] = (temp >> 24) & 0xFF;

    temp = CAN1->sFIFOMailBox[0].RDHR;

    frame->data[4] = temp & 0xFF;
    frame->data[5] = (temp >> 8) & 0xFF;
    frame->data[6] = (temp >> 16) & 0xFF;
    frame->data[7] = (temp >> 24) & 0xFF;

    /* 释放FIFO */
    CAN1->RF0R |= (1 << 5);

    return 1;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{

    CAN_Frame_t frame;
    if (CAN1_Receive(&frame))
    {
        Dash_UpdateData(&frame);
        Dash_RefreshDisplay();
    }
    CAN1->RF0R &= ~(1 << 3);
    CAN1->RF0R &= ~(1 << 4);
}
