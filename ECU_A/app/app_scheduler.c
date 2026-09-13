// app_scheduler.c — 周期任务调度器实现
#include "app_scheduler.h"
#include "app_bms_state.h"
#include "app_event.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"

static const uint16_t g_task_period[TASK_COUNT] = {
    10,     // TASK_BMS_10MS     — ADC读取+换算(核心业务)
    20,     // TASK_EVENT_PROCESS— 事件队列处理
    500,    // TASK_LED_HEARTBEAT— LED心跳闪烁
};



// g_task_last_run — 每个任务上次执行的时间戳（ms）
// 与GetTick()的返回值比较，判断是否到了执行时间
static uint32_t g_task_last_run[TASK_COUNT] = {0};


// Scheduler_Init — 调度器初始化
// 把所有任务的上次执行时间清零
// 这样启动后所有任务会立即执行第一次
void Scheduler_Init(void)
{
	for(uint8_t i = 0; i< TASK_COUNT; i++)
	{
		g_task_last_run[i] = 0;
	}
}




// Scheduler_Run — 调度器主函数
// 在main循环中不断调用，每次遍历所有任务
// 检查每个任务是否到了执行时间
// 到了就执行对应函数，并更新上次执行时间
//
// 执行策略：
//   10ms → BMS_StateMachine()   ADC读取+换算+故障检测+CAN发送
//   20ms → Event_Process()      处理事件队列
//   50ms → BMS_StateMachine()   故障检测(快速响应)
//  100ms → BMS_StateMachine()   CAN发送0x301(电压+SOC)
//  200ms → BMS_StateMachine()   CAN发送0x302(温度)
//  500ms → BMS_StateMachine()   LED心跳
//
// 注意：
//   Event_Process 和 LED_Toggle 不调用 BMS_StateMachine
//   避免状态机被过度调用导致充电SOC飙升
void Scheduler_Run(void)
{
	//获取当前时间
	uint32_t now = GetTick();
	
	for(uint8_t i = 0; i < TASK_COUNT; i++)
	{
		//判断：当前时间 - 上次执行时间 >= 任务周期
		if(now - g_task_last_run[i] >= g_task_period[i])
		{
			//更新时间戳
			g_task_last_run[i] = now;
			
			//根据任务ID执行对应操作
			switch(i)
			{
				//10ms任务--BMS核心
				case TASK_BMS_10MS:
					BMS_StateMachine();
        break;
					//20ms--处理事件队列
				case TASK_EVENT_PROCESS:
					Event_Process();
					break;
				
					//500ms--LED心跳
				case TASK_LED_HEARTBEAT:
					{
						if(BMS_GetState() != BMS_FAULT)
							{
								LED_Toggle();
							}
						break;
					}
			}
		}
	}
}














