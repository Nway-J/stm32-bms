/*
 * 模块：FreeRTOS迁移期间的兼容调度器公共接口。
 *
 * 当前只定义尚未迁移的LED周期功能编号。
 *
 * Sensor、BMS、CAN、FaultLog和Health功能
 * 已经迁移到各自的独立FreeRTOS任务。
 *
 * 任务编号顺序必须与app_scheduler.c中的
 * g_task_period数组顺序完全一致。
 */
#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H
#include "stm32f10x.h"

typedef enum
{
    TASK_LED_HEARTBEAT = 0,            // LED运行心跳
    TASK_COUNT
} TaskID_t;


void Scheduler_Init(void);
void Scheduler_Run(void);

#endif
