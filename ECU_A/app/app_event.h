// app_event.h — 事件队列
#ifndef __APP_EVENT_H
#define __APP_EVENT_H
#include "stm32f10x.h"

// 事件类型枚举
// 每个枚举值代表一种"发生了什么事"
// 这些事件由各个Task检测条件后Push到队列
// 再由Event_Process取出交给BMS_EventHandle处理

typedef enum
{
    EVENT_NONE = 0,         // 空事件，不处理
    EVENT_START,            // 启动BMS（PB0按下 / CAN命令）
    EVENT_STOP,             // 停止BMS（CAN命令）
    EVENT_FAULT,            // 发生故障（CheckFault检测到过压/欠压/过温）
    EVENT_RECOVER,          // 故障恢复（PB1按下，请求重新检测）
    EVENT_CHARGE_DONE,      // 充满电（charge_soc >= 95）
    EVENT_CAN_CMD,          // CAN命令到达（扩展用）
    EVENT_TIMEOUT,          // 超时事件（扩展用）
} Event_t;



void Event_Init(void);						//初始化
void Event_Push(Event_t event);		//压入事件到尾部
Event_t Event_Pop(void);					//从队列头部取出事件
void Event_Process(void);					//处理队列中所有待处理事件
uint8_t Event_Available(void);		//判断队列是否空

#endif
