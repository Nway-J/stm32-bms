/*
 * 文件名称：app_event.h
 *
 * 模块名称：BMS状态事件队列公共接口
 *
 * 模块作用：
 * 保存“启动监测、停止监测、进入故障、请求恢复”等简短命令，
 * 再由Event_Process()把命令交给BMS_EventHandle()处理状态切换。
 *
 * 模块边界：
 * 本模块不判断电压或温度是否故障，也不保存故障现场数据。
 * 详细故障记录由app_fault_event负责，故障条件判断由app_fault_manager负责。
 *
 * 当前并发限制：
 * 裸机版本只按主循环单线程调用设计，不应从中断和主循环同时操作队列。
 * 迁移FreeRTOS后将使用FreeRTOS Queue保证任务间传递安全。
 */

#ifndef __APP_EVENT_H
#define __APP_EVENT_H
#include "stm32f10x.h"

/*
 * 每个枚举值代表一个状态处理请求。
 * 原编号5是已经删除的模拟充满事件；为避免改变已有编号，编号5继续空置。
 */
typedef enum
{
    // 没有事件。
    EVENT_NONE = 0,

    // 请求从待机进入监测。
    EVENT_START = 1,

    // 请求从监测返回待机。
    EVENT_STOP = 2,

    // 通知状态机发生故障。
    EVENT_FAULT = 3,

    // 请求检查故障恢复条件。
    EVENT_RECOVER = 4,

    // 保留CAN命令扩展事件。
    EVENT_CAN_CMD = 6,

    // 保留超时扩展事件。
    EVENT_TIMEOUT = 7
} Event_t;


void Event_Init(void);						//初始化
void Event_Push(Event_t event);		//压入事件到尾部
Event_t Event_Pop(void);					//从队列头部取出事件
void Event_Process(void);					//处理队列中所有待处理事件
uint8_t Event_Available(void);		//判断队列是否空

#endif
