// CAN驱动（500kbps，双滤波器配置共ECU-A/B使用）

#include "bsp_can.h"
#include "can_protocol.h"
#include "stm32f10x.h"
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

    // 4.配置位时序 波特率500kbps
    CAN1->BTR = 0;

    CAN1->BTR |= (0 << 24);  // SJW=1TQ->0
    CAN1->BTR |= (3 << 20);  // BS2=4TQ->3
    CAN1->BTR |= (12 << 16); // BS1=13TQ->12
    CAN1->BTR |= (3 << 0);   // BRP=4TQ->3

    // 5.配置过滤器
    CAN1->FMR |= 1; // 进入配置模式

    // Filter0: 0x301 + 0x302
    CAN1->FM1R &= ~(1 << 0);
    CAN1->FM1R |= (1 << 0); // 列表模式
    CAN1->FS1R &= ~(1 << 0);
    CAN1->FS1R |= (1 << 0); // 32位
    CAN1->sFilterRegister[0].FR1 = (0x301 << 21);
    CAN1->sFilterRegister[0].FR2 = (0x302 << 21);
    CAN1->FA1R |= (1 << 0); // 激活Filter0

    // Filter1: 0x303 + 0x100
    CAN1->FM1R |= (1 << 1); // 列表模式
    CAN1->FS1R |= (1 << 1); // 32位
    CAN1->sFilterRegister[1].FR1 = (0x303 << 21);
    CAN1->sFilterRegister[1].FR2 = (0x100 << 21);
    CAN1->FA1R |= (1 << 1); // 激活Filter1

    CAN1->FFA1R &= ~((1 << 0) | (1 << 1)); // 都进FIFO0

    CAN1->FMR &= ~1; // 退出配置模式

    CAN1->MCR |= (1 << 6); // ABOM=1 自动离线恢复

    // 6. 退出初始化
    CAN1->MCR &= ~(1 << 1);
    CAN1->MCR &= ~(1 << 0);
    CAN1->MCR &= ~(1 << 4);
    while (CAN1->MSR & (1 << 0))
        ;

    printf("[CAN] Init done. MCR=0x%08X MSR=0x%08X BTR=0x%08X\r\n", CAN1->MCR, CAN1->MSR,
           CAN1->BTR);
}

// ECU-A发送
uint8_t CAN1_Send(CAN_Frame_t *frame)
{

    uint8_t mailbox = 0;
    uint32_t dl, dh;

    uint32_t timeout;

    uint32_t rqcp_mask;
    uint32_t txok_mask;
    uint32_t alst_mask;
    uint32_t terr_mask;
    // 先读ESR判断总线状态

    if (CAN1->ESR & ((1 << 1) | (1 << 2))) // EPVF(错误被动)或BOFF(总线关闭)
    {
        // 进入初始化模式
        CAN1->MCR |= (1 << 0);
        while (!(CAN1->MSR & (1 << 0)))
            ;

        // 退出初始化模式（复位错误计数器）
        CAN1->MCR &= ~(1 << 0);
        while (CAN1->MSR & (1 << 0))
            ;

        // 等待总线恢复
        for (volatile int i = 0; i < 5000; i++)
            ;
    }

    uint32_t esr = CAN1->ESR;
    if (esr & (1 << 2)) // BOFF=1 总线关闭
    {
        printf("[CAN] Bus Off! Recovering...\r\n");
        // 请求退出总线关闭
        CAN1->MCR |= (1 << 0); // INRQ=1 进初始化
        while (!(CAN1->MSR & (1 << 0)))
            ;
        CAN1->MCR &= ~(1 << 0); // 退出初始化
        while (CAN1->MSR & (1 << 0))
            ;
        printf("[CAN] Recovered\r\n");
    }
    else if (esr & (1 << 0)) // EWGF 警告
    {
        printf("[CAN] Warning ESR=0x%08X\r\n", esr);
    }

    /* DLC最大8字节 */
    if (frame->dlc > 8)
    {
        frame->dlc = 8;
    }

    /* 仅允许标准帧ID */
    if (frame->id > 0x7FF)
    {
        return 5;
    }

    /*--------------------------------------------------
      查找空闲发送邮箱
      TME0 -> bit26
      TME1 -> bit27
      TME2 -> bit28
    --------------------------------------------------*/

    if (CAN1->TSR & (1 << 26))
    {
        mailbox = 0;
    }
    else if (CAN1->TSR & (1 << 27))
    {
        mailbox = 1;
    }
    else if (CAN1->TSR & (1 << 28))
    {
        mailbox = 2;
    }
    else
    {
        return 0;
    }

    /*--------------------------------------------------
      清空邮箱控制寄存器
    --------------------------------------------------*/
    CAN1->sTxMailBox[mailbox].TIR = 0;

    /*--------------------------------------------------
      设置标准帧ID
      标准ID位于[31:21]
    --------------------------------------------------*/
    CAN1->sTxMailBox[mailbox].TIR |= (frame->id << 21);

    /*--------------------------------------------------
      设置数据长度
    --------------------------------------------------*/
    CAN1->sTxMailBox[mailbox].TDTR = frame->dlc;

    /*--------------------------------------------------
      打包前4字节
    --------------------------------------------------*/
    dl = ((uint32_t)frame->data[0]) | ((uint32_t)frame->data[1] << 8) |
         ((uint32_t)frame->data[2] << 16) | ((uint32_t)frame->data[3] << 24);

    /*--------------------------------------------------
      打包后4字节
    --------------------------------------------------*/
    dh = ((uint32_t)frame->data[4]) | ((uint32_t)frame->data[5] << 8) |
         ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);

    CAN1->sTxMailBox[mailbox].TDLR = dl;
    CAN1->sTxMailBox[mailbox].TDHR = dh;

    /*--------------------------------------------------
      根据邮箱选择状态位
    --------------------------------------------------*/
    if (mailbox == 0)
    {
        rqcp_mask = (1 << 0);
        txok_mask = (1 << 1);
        alst_mask = (1 << 2);
        terr_mask = (1 << 3);
    }
    else if (mailbox == 1)
    {
        rqcp_mask = (1 << 8);
        txok_mask = (1 << 9);
        alst_mask = (1 << 10);
        terr_mask = (1 << 11);
    }
    else
    {
        rqcp_mask = (1 << 16);
        txok_mask = (1 << 17);
        alst_mask = (1 << 18);
        terr_mask = (1 << 19);
    }

    /*--------------------------------------------------
      请求发送
      TXRQ = 1
    --------------------------------------------------*/

    CAN1->sTxMailBox[mailbox].TIR |= 1;

    /*--------------------------------------------------
      等待发送完成
    --------------------------------------------------*/
    timeout = 500000;

    while (!(CAN1->TSR & rqcp_mask))
    {
        if (--timeout == 0)
        {

            printf("TIMEOUT TSR=0x%08X ESR=0x%08X\r\n", CAN1->TSR, CAN1->ESR);

            /* 超时：用ABRQ中止发送，释放邮箱 */
            if (mailbox == 0)
                CAN1->TSR |= (1 << 7); // ABRQ0
            if (mailbox == 1)
                CAN1->TSR |= (1 << 15); // ABRQ1
            if (mailbox == 2)
                CAN1->TSR |= (1 << 23); // ABRQ2

            /* 等待中止完成（硬件自动恢复TME位） */
            for (volatile int i = 0; i < 5000; i++)
                ;

            /* 清状态位 */
            CAN1->TSR |= rqcp_mask | txok_mask;

            return 2;
        }
    }

    /*--------------------------------------------------
      判断发送结果
    --------------------------------------------------*/
    if (CAN1->TSR & txok_mask)
    {
        /* 清状态位 */
        CAN1->TSR |= rqcp_mask | txok_mask | alst_mask | terr_mask;

        return 1;
    }

    if (CAN1->TSR & alst_mask)
    {
        /* 仲裁失败 */
        CAN1->TSR |= rqcp_mask | txok_mask | alst_mask | terr_mask;

        return 3;
    }

    if (CAN1->TSR & terr_mask)
    {
        /* 发送错误 */
        CAN1->TSR |= rqcp_mask | txok_mask | alst_mask | terr_mask;

        return 4;
    }

    /* 未知错误 */
    CAN1->TSR |= rqcp_mask | txok_mask | alst_mask | terr_mask;

    return 5;
}

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
