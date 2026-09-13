#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f10x.h"
#include "can_protocol.h"

void CAN1_Init(void);
uint8_t CAN1_Send(CAN_Frame_t *frame);
// 返回值：1=成功, 0=无空邮箱, 2=超时, 3=发送失败
uint8_t CAN1_Receive(CAN_Frame_t *frame);

#endif
