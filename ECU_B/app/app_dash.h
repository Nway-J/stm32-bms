// app_dash.h — 仪表显示逻辑
#ifndef __APP_DASH_H
#define __APP_DASH_H
#include "stm32f10x.h"
#include "can_protocol.h"

typedef struct
{
    uint16_t cell_mv; // 电压(mV)

    uint8_t chg_state; // 充放电状态

    uint8_t temp; // 温度

    uint8_t soc; // SOC

    uint8_t fault; // 故障码

    volatile uint8_t valid; // 是否收到过数据

    uint32_t last_rx_tick; // 最后一次收到CAN时间
} BMS_DisplayData_t;

extern BMS_DisplayData_t g_dash_data;

void Dash_Init(void);
void Dash_UpdateData(CAN_Frame_t *frame);
void Dash_RefreshDisplay(void);

#endif
