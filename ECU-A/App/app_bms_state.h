#ifndef __APP_BMS_STATE_H
#define __APP_BMS_STATE_H

#include "stm32f10x.h"
#include "app_event.h"     // 引入Event_t枚举定义


/*
 * 模块：BMS故障位定义。
 * 作用：统一状态机使用的故障码，避免直接书写十六进制数字。
 * 编码：每一种故障占用一个独立的二进制位，可以通过按位或组合。
 * 兼容：保留原有过压、欠压、过温和自检失败的数值。
 * 注意：新增NTC故障位仅定义编码，检测与保护逻辑将在后续接入。
 */

// 没有故障。
#define BMS_FAULT_NONE          0x00U

// Bit0：电池过压。
#define BMS_FAULT_OV            0x01U

// Bit1：电池欠压。
#define BMS_FAULT_UV            0x02U

// Bit2：电池过温。
#define BMS_FAULT_OT            0x04U

// Bit3：NTC开路故障。
#define BMS_FAULT_NTC_OPEN      0x08U

// Bit4：NTC短路故障。
#define BMS_FAULT_NTC_SHORT     0x10U

// Bit7：自检失败。
#define BMS_FAULT_SELF_CHECK    0x80U

extern uint16_t g_fault_code;  // 当前16位活动故障掩码，与CAN 0x303一致

/*
 * 模块：电池监测系统状态定义。
 * 作用：描述初始化、自检、待机、监测和故障报警阶段。
 * 本硬件没有充放电控制通路，因此不定义充电、放电执行状态。
 * 保留原有有效状态的编号
 */
typedef enum
{
    // 上电初始化，不表示硬件断电。
    BMS_INIT = 0,

    // 检查系统是否满足进入待机的条件。
    BMS_SELF_CHECK = 1,

    // 等待启动监测的请求。
    BMS_STANDBY = 2,

    // 执行电压、温度监测和数据上报。
    BMS_MONITOR = 3,

    // 执行故障报警与上报，不代表切断电池电流。
    BMS_FAULT = 4
} BMS_State_t;



void BMS_SetState(BMS_State_t state);           // 设置当前状态
BMS_State_t BMS_GetState(void);                 // 获取当前状态
void BMS_StateMachine(void);                    // 状态机入口（由调度器调用）
void BMS_EventHandle(Event_t event);            // 事件处理（由Event_Process调用）



#endif

