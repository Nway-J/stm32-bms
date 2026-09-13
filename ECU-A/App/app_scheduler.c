/*
 * 文件名称：app_scheduler.c
 *
 * 模块职责：
 * 1. 暂时运行尚未迁移的LED周期功能；
 * 2. 每迁移一个独立任务，就删除本模块中的对应入口；
 * 3. 全部业务迁移完成后删除本模块。
 *
 * 已迁移功能：
 * 1. SensorTask负责传感器数据；
 * 2. BmsTask负责故障判断和状态机；
 * 3. CanTask负责CAN发送；
 * 4. FaultLogTask负责故障事件持久化。
 * 5. HealthTask负责关键任务心跳检查和IWDG刷新。
 *
 * 重要约束：
 * 1. 本模块不得再次调用Sensor_ServiceUpdate()；
 * 2. 本模块不得再次调用BMS_StateMachine()；
 * 3. 本模块不得再次调用Event_Process()；
 * 4. 防止同一个业务模块出现两个执行者。
 */

#include "app_scheduler.h"
#include "app_bms_state.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"




static const uint16_t g_task_period[TASK_COUNT] = {
    500U,    // TASK_LED_HEARTBEAT — LED心跳闪烁
};



// g_task_last_run — 每个任务上次执行的时间戳（ms）
// 与GetTick()的返回值比较，判断是否到了执行时间
static uint32_t g_task_last_run[TASK_COUNT] = {0};


// Scheduler_Init — 调度器初始化
// 把所有任务的上次执行时间清零
// 各任务将在系统时间达到自身第一个周期时执行第一次
void Scheduler_Init(void)
{
	for(uint8_t i = 0; i< TASK_COUNT; i++)
	{
		g_task_last_run[i] = 0;
	}
}




/*
 * Scheduler_Run — FreeRTOS迁移期间的兼容调度入口。
 *
 * CompatibilityTask每1ms调用本函数一次，
 * 本函数根据GetTick()判断剩余周期业务是否到期。
 *
 * 当前执行策略：
 * 500ms：在非故障状态切换LED心跳。
 *
 * Sensor、BMS、CAN、FaultLog和Health已经迁移到独立任务，
 * 本函数不得再次调用这些模块的周期执行入口。
 */
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
			switch (i)
				{
					/*
					* 每500ms处理一次普通运行LED。
					* BMS处于故障状态时，LED由故障指示逻辑控制，
					* 这里不再切换它。
					*/
					case TASK_LED_HEARTBEAT:
					{
						if (BMS_GetState() != BMS_FAULT)
						{
							LED_Toggle();
						}

						break;
					}

					default:
						break;
				}
		}
	}
}














