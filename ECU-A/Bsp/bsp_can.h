/*
 * 文件名称：bsp_can.h
 *
 * 模块作用：
 * 提供CAN1初始化、同步发送和接收接口。
 *
 * CAN1_Send通过明确的返回值告诉上层：
 * 本次发送成功，或者失败在什么阶段。
 */

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f10x.h"
#include "can_protocol.h"


/*
 * CAN发送结果。
 * 使用名称代替0、1、2等数字，避免调用者记忆“魔法数字”。
 */
typedef enum
{
    CAN_SEND_NO_MAILBOX = 0, // 三个发送邮箱都忙，本次没有提交
    CAN_SEND_OK         = 1, // 报文发送成功
    CAN_SEND_TIMEOUT    = 2, // 等待发送完成超时
    CAN_SEND_ARB_LOST   = 3, // 发送过程中仲裁失败
    CAN_SEND_TX_ERROR   = 4, // CAN控制器报告发送错误
    CAN_SEND_INVALID    = 5, // 参数、ID或发送结果异常
    CAN_SEND_BUS_OFF    = 6  // 控制器当前处于Bus-Off
    
} CAN_SendResult_t;

/*
 * CAN运行诊断变量。
 * 这些变量只用于观察和后续故障判断，不参与报文内容。
 */
extern volatile CAN_SendResult_t g_can_last_send_result; // 最近一次发送结果
extern volatile uint32_t g_can_tx_ok_count;              // 发送成功累计次数
extern volatile uint32_t g_can_tx_fail_count;            // 发送失败累计次数
extern volatile uint32_t g_can_bus_off_count;             // 进入Bus-Off累计次数
extern volatile uint32_t g_can_recovery_count;            // 从Bus-Off恢复累计次数
extern volatile uint32_t g_can_last_esr;                  // 最近读取的CAN错误状态寄存器
extern volatile uint8_t g_can_bus_off_active;             // 当前是否处于Bus-Off

void CAN1_Init(void);

CAN_SendResult_t CAN1_Send(CAN_Frame_t *frame);

uint8_t CAN1_Receive(CAN_Frame_t *frame);


#endif
