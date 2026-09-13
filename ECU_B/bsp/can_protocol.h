// can_protocol.h — CAN报文协议定义（ECU-A和ECU-B共用）
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "stm32f10x.h"

#define CAN_ID_BMS_VOLTAGE 0x301
#define CAN_ID_BMS_TEMP 0x302
#define CAN_ID_BMS_FAULT 0x303
#define CAN_ID_CTRL_CMD 0x100

typedef struct
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_Frame_t;

#define CHG_STATE_IDLE 0
#define CHG_STATE_DISCHARGE 1
#define CHG_STATE_CHARGE 2

static inline void BMS_PackVoltage(CAN_Frame_t *f, uint16_t mv, uint8_t soc, uint8_t chg_state)
{
    f->id = CAN_ID_BMS_VOLTAGE;
    f->dlc = 8;
    f->data[0] = (mv >> 8) & 0xFF;
    f->data[1] = mv & 0xFF;
    f->data[2] = soc;
    f->data[3] = chg_state;
    for (int i = 4; i < 8; i++)
        f->data[i] = 0;
}

static inline void BMS_PackTemp(CAN_Frame_t *f, uint8_t temp)
{
    f->id = CAN_ID_BMS_TEMP;
    f->dlc = 8;
    f->data[0] = temp;
    for (int i = 1; i < 8; i++)
        f->data[i] = 0;
}

static inline void BMS_PackFault(CAN_Frame_t *f, uint8_t fault)
{
    f->id = CAN_ID_BMS_FAULT;
    f->dlc = 8;
    f->data[0] = fault;
    for (int i = 1; i < 8; i++)
        f->data[i] = 0;
}

#endif
