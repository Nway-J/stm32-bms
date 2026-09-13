#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "stm32f10x.h"
#include "can_protocol.h"

void CAN1_Init(void);
uint8_t CAN1_Send(CAN_Frame_t *frame);
uint8_t CAN1_Receive(CAN_Frame_t *frame);
void USB_LP_CAN1_RX0_IRQHandler(void);

#endif
