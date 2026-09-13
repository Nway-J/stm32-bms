#ifndef __APP_BMS_STATE_H
#define __APP_BMS_STATE_H

#include "stm32f10x.h"
#include "app_event.h" // 引入Event_t枚举定义

extern uint8_t g_fault_code; // 当前故障码

typedef enum
{
    BMS_POWER_OFF = 0, // 上电初始化
    BMS_SELF_CHECK,    // 自检
    BMS_STANDBY,       // 待机（等待启动）
    BMS_DISCHARGE,     // 放电/正常运行
    BMS_FAULT,         // 故障保护
    BMS_CHARGE         // 充电
} BMS_State_t;

void BMS_SetState(BMS_State_t state); // 设置当前状态
BMS_State_t BMS_GetState(void);       // 获取当前状态
void BMS_StateMachine(void);          // 状态机入口（由调度器调用）
void BMS_EventHandle(Event_t event);  // 事件处理（由Event_Process调用）

#endif
