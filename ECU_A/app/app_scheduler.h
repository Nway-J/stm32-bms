// app_scheduler.h — 周期任务调度器
#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H
#include "stm32f10x.h"

typedef enum
{
    TASK_BMS_10MS = 0,  // BMS核心业务：ADC读取+换算+故障检测+CAN发送
    TASK_EVENT_PROCESS, // 事件队列处理：取出事件交给BMS_EventHandle
    TASK_LED_HEARTBEAT, // LED心跳：翻转PC13，指示系统运行中
    TASK_COUNT          // 任务总数，用于定义数组大小
} TaskID_t;

void Scheduler_Init(void);
void Scheduler_Run(void);

#endif
