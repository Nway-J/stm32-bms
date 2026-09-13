/*
 * 文件名称：bsp_can.c
 *
 * 模块作用：
 * 配置STM32F103的CAN1，并提供标准数据帧的同步发送和接收。
 *
 * 当前配置：
 * 波特率500kbps，使用PA11作为RX、PA12作为TX。
 *
 * Bus-Off处理原则：
 * 初始化时开启ABOM硬件自动恢复；
 * 发送函数只检测Bus-Off并返回结果，不在发送过程中强制重启CAN。
 *
 * 当前发送仍采用有限次数的同步等待。
 * 后续迁移FreeRTOS时，再由独立CAN发送任务管理报文队列。
 */

#include "bsp_can.h"
#include "can_protocol.h"
#include "stm32f10x.h"
#include "bsp_usart.h"



/*
 * CAN诊断数据。
 * 全局变量上电后由启动代码清零，后续由CAN驱动更新。
 */
volatile CAN_SendResult_t g_can_last_send_result = CAN_SEND_OK;
volatile uint32_t g_can_tx_ok_count = 0U;
volatile uint32_t g_can_tx_fail_count = 0U;
volatile uint32_t g_can_bus_off_count = 0U;
volatile uint32_t g_can_recovery_count = 0U;
volatile uint32_t g_can_last_esr = 0U;
volatile uint8_t g_can_bus_off_active = 0U;


/*
 * 统一记录一次CAN发送结果，并把原结果返回给调用者。
 *
 * 这样CAN1_Send中的每个出口都通过同一个函数统计，
 * 避免某一种失败情况忘记更新计数器。
 */
static CAN_SendResult_t CAN1_RecordSendResult(CAN_SendResult_t result)
{
    g_can_last_send_result = result;
    g_can_last_esr = CAN1->ESR;

    if (result == CAN_SEND_OK)
    {
        g_can_tx_ok_count++;
    }
    else
    {
        g_can_tx_fail_count++;
    }

    return result;
}


//CAN初始化
void CAN1_Init(void)
{ 
	//1.开GPIOA/AFIO复用/CAN1时钟
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<0);
	RCC->APB1ENR |= (1<<25);
	
	//2.配置PA11-CAN_RX上拉输入 / PA12--CAN_TX复用推挽
	GPIOA->CRH &= ~(0xF << 12);
    GPIOA->CRH |=  (0x8 << 12);
    GPIOA->ODR |= (1 << 11);

    GPIOA->CRH &= ~(0xF << 16);
    GPIOA->CRH |=  (0xB << 16);
	
	//3.进入初始化模式
	CAN1->MCR |= (1<<0);//INRQ初始化请求
	while((CAN1->MSR & (1<<0)) == 0);//INAK初始确认
	
	//4.配置位时序 波特率500kbps
	CAN1->BTR = 0;
	
	CAN1->BTR |= (0<<24);//SJW=1TQ->0
	CAN1->BTR |= (3<<20);//BS2=4TQ->3
	CAN1->BTR |= (12<<16);//BS1=13TQ->12
	CAN1->BTR |= (3<<0); //BRP=4TQ->3
	
	//5.配置过滤器
	CAN1->FMR |= 1;  // 进入配置模式
	

	// Filter0: 0x301 + 0x302
	CAN1->FM1R &= ~(1 << 0);
	CAN1->FM1R |= (1 << 0);   // 列表模式
	CAN1->FS1R &= ~(1 << 0);
	CAN1->FS1R |= (1 << 0);   // 32位
	CAN1->sFilterRegister[0].FR1 = (0x301 << 21);
	CAN1->sFilterRegister[0].FR2 = (0x302 << 21);
	CAN1->FA1R |= (1 << 0);   // 激活Filter0

	// Filter1: 0x303 + 0x100
	CAN1->FM1R |= (1 << 1);   // 列表模式
	CAN1->FS1R |= (1 << 1);   // 32位
	CAN1->sFilterRegister[1].FR1 = (0x303 << 21);
	CAN1->sFilterRegister[1].FR2 = (0x100 << 21);
	CAN1->FA1R |= (1 << 1);   // 激活Filter1

	CAN1->FFA1R &= ~((1 << 0) | (1 << 1));  // 都进FIFO0
	
	CAN1->FMR &= ~1;  // 退出配置模式
	
	CAN1->MCR |= (1 << 6);  // ABOM=1 自动离线恢复

	// 6. 退出初始化
    CAN1->MCR &= ~(1 << 1);
    CAN1->MCR &= ~(1 << 0);
	CAN1->MCR &= ~(1 << 4);
    while(CAN1->MSR & (1 << 0));
	
	printf("[CAN] Init done. MCR=0x%08X MSR=0x%08X BTR=0x%08X\r\n",
       CAN1->MCR, CAN1->MSR, CAN1->BTR);

}






//ECU-A发送
CAN_SendResult_t CAN1_Send(CAN_Frame_t *frame)
{
	
	uint32_t esr;
    uint8_t mailbox = 0;
    uint32_t dl, dh;

    uint32_t timeout;

    uint32_t rqcp_mask;
    uint32_t txok_mask;
    uint32_t alst_mask;
    uint32_t terr_mask;

    /*
    * 每次发送前只读取一次错误状态寄存器。
    * 如果控制器仍处于Bus-Off，本次不操作发送邮箱。
    *
    * ABOM会在总线满足恢复条件后自动清除Bus-Off，
    * 后续周期调用CAN1_Send时会再次尝试发送。
    */
    esr = CAN1->ESR;
    g_can_last_esr = esr;

    /*
    * ESR第2位BOFF为1，表示CAN控制器已经退出总线通信。
    * 只在状态从“正常”变成“Bus-Off”时累计一次。
    */
    if ((esr & (1U << 2)) != 0U)
    {
        if (g_can_bus_off_active == 0U)
        {
            g_can_bus_off_active = 1U;
            g_can_bus_off_count++;
        }

        return CAN1_RecordSendResult(CAN_SEND_BUS_OFF);
    }

    /*
    * 上一次检查还处于Bus-Off，而这次BOFF已经清零，
    * 说明ABOM已经完成了一次自动恢复。
    */
    if (g_can_bus_off_active != 0U)
    {
        g_can_bus_off_active = 0U;
        g_can_recovery_count++;
    }
		
    /* DLC最大8字节 */
    if(frame->dlc > 8)
    {
        frame->dlc = 8;
    }

    /* 仅允许标准帧ID */
    if(frame->id > 0x7FF)
    {
        return CAN1_RecordSendResult(CAN_SEND_INVALID);
    }

    /*--------------------------------------------------
      查找空闲发送邮箱
      TME0 -> bit26
      TME1 -> bit27
      TME2 -> bit28
    --------------------------------------------------*/
          
    if(CAN1->TSR & (1 << 26))
    {
        mailbox = 0;
    }
    else if(CAN1->TSR & (1 << 27))
    {
        mailbox = 1;
    }
    else if(CAN1->TSR & (1 << 28))
    {
        mailbox = 2;
    }
    else
    {
        return  CAN1_RecordSendResult(CAN_SEND_NO_MAILBOX);
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
    dl =
        ((uint32_t)frame->data[0]) |
        ((uint32_t)frame->data[1] << 8) |
        ((uint32_t)frame->data[2] << 16) |
        ((uint32_t)frame->data[3] << 24);

    /*--------------------------------------------------
      打包后4字节
    --------------------------------------------------*/
    dh =
        ((uint32_t)frame->data[4]) |
        ((uint32_t)frame->data[5] << 8) |
        ((uint32_t)frame->data[6] << 16) |
        ((uint32_t)frame->data[7] << 24);

    CAN1->sTxMailBox[mailbox].TDLR = dl;
    CAN1->sTxMailBox[mailbox].TDHR = dh;

    /*--------------------------------------------------
      根据邮箱选择状态位
    --------------------------------------------------*/
    if(mailbox == 0)
    {
        rqcp_mask = (1 << 0);
        txok_mask = (1 << 1);
        alst_mask = (1 << 2);
        terr_mask = (1 << 3);
    }
    else if(mailbox == 1)
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

    while(!(CAN1->TSR & rqcp_mask))
    {
       /*
        * 发送等待期间也检查Bus-Off。
        * 没有ACK时，CAN控制器可能在本次发送过程中进入Bus-Off。
        */
        esr = CAN1->ESR;
        g_can_last_esr = esr;

        if ((esr & (1U << 2)) != 0U)
        {
            /*
            * 只在正常到Bus-Off的变化瞬间累计一次，
            * 后续重复检查不会重复增加。
            */
            if (g_can_bus_off_active == 0U)
            {
                g_can_bus_off_active = 1U;
                g_can_bus_off_count++;
            }

            /*
            * 撤销当前邮箱中尚未完成的报文，
            * 避免总线恢复后又自动发送这条旧报文。
            */
            if (mailbox == 0U)
            {
                CAN1->TSR = (1U << 7);   // ABRQ0
            }
            else if (mailbox == 1U)
            {
                CAN1->TSR = (1U << 15);  // ABRQ1
            }
            else
            {
                CAN1->TSR = (1U << 23);  // ABRQ2
            }

            return CAN1_RecordSendResult(CAN_SEND_BUS_OFF);
        }
        
        if(--timeout == 0)
					{
						
						printf("TIMEOUT TSR=0x%08X ESR=0x%08X\r\n",
           CAN1->TSR,
           CAN1->ESR);
						
						/* 超时：用ABRQ中止发送，释放邮箱 */
						if(mailbox == 0) CAN1->TSR |= (1 << 7);   // ABRQ0
						if(mailbox == 1) CAN1->TSR |= (1 << 15);  // ABRQ1
						if(mailbox == 2) CAN1->TSR |= (1 << 23);  // ABRQ2
						
						/* 等待中止完成（硬件自动恢复TME位） */
						for(volatile int i = 0; i < 5000; i++);
						
						/* 清状态位 */
						CAN1->TSR |= rqcp_mask | txok_mask;
						
						return  CAN1_RecordSendResult(CAN_SEND_TIMEOUT);
					}
			
    }

    /*--------------------------------------------------
      判断发送结果
    --------------------------------------------------*/
    if(CAN1->TSR & txok_mask)
    {
        /* 清状态位 */
        CAN1->TSR |=
            rqcp_mask |
            txok_mask |
            alst_mask |
            terr_mask;

        return  CAN1_RecordSendResult(CAN_SEND_OK);
    }

    if(CAN1->TSR & alst_mask)
    {
        /* 仲裁失败 */
        CAN1->TSR |=
            rqcp_mask |
            txok_mask |
            alst_mask |
            terr_mask;

        return CAN1_RecordSendResult(CAN_SEND_ARB_LOST);
    }

    if(CAN1->TSR & terr_mask)
    {
        /* 发送错误 */
        CAN1->TSR |=
            rqcp_mask |
            txok_mask |
            alst_mask |
            terr_mask;

        return CAN1_RecordSendResult(CAN_SEND_TX_ERROR);
    }

    /* 未知错误 */
    CAN1->TSR |=
        rqcp_mask |
        txok_mask |
        alst_mask |
        terr_mask;

    return CAN1_RecordSendResult(CAN_SEND_INVALID);
}






uint8_t CAN1_Receive(CAN_Frame_t *frame)
{
    uint32_t temp;

    /* FIFO0无数据 */
    if((CAN1->RF0R & 0x03) == 0)
    {
        return 0;
    }

    /* FIFO溢出 */
    if(CAN1->RF0R & (1 << 4))
    {
        CAN1->RF0R |= (1 << 4);
    }

    /* 仅读取标准数据帧 */
    if(CAN1->sFIFOMailBox[0].RIR & (1 << 2))
    {
        CAN1->RF0R |= (1 << 5);
        return 0;
    }

    frame->id =
        (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

    frame->dlc =
        CAN1->sFIFOMailBox[0].RDTR & 0x0F;

    if(frame->dlc > 8)
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


