/*
 * 文件名称：app_rtos.h
 *
 * 模块名称：FreeRTOS任务组织公共接口
 *
 * 模块职责：
 * 1. 对外提供应用任务创建入口；
 * 2. 定义可供Keil Watch观察的RTOS运行诊断变量；
 * 3. 隔离main函数与具体任务实现。
 *
 * 当前迁移阶段：
 * 1. HealthTask独立检查关键任务心跳并管理IWDG刷新；
 * 2. SensorTask独立处理ADC/DMA数据并发布传感器快照；
 * 3. BmsTask独立运行故障判断、状态机和状态事件；
 * 4. CanTask独立发送电压、温度和故障状态报文；
 * 5. FaultLogTask独立等待故障事件并写入Flash；
 * 6. DiagnosticTask处理CLI和资源诊断。
 */

#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdint.h>


/*
 * 诊断任务已经完整运行的累计次数。
 * 该变量持续增加，说明FreeRTOS任务正在被周期调度。
 */
extern volatile uint32_t g_rtos_diagnostic_run_count;
/*
 * HealthTask已经完成的健康检查次数。
 * 正常情况下大约每100ms增加一次。
 */
extern volatile uint32_t g_rtos_health_run_count;
/*
 * SensorTask已经完整运行的累计次数。
 *
 * 该变量记录任务被调度的次数；
 * SensorSample中的sequence记录成功取得新ADC数据的次数。
 * 两者含义不同。
 */
extern volatile uint32_t g_rtos_sensor_run_count;

/*
 * BmsTask已经完整运行的累计次数。
 *
 * 正常情况下每10ms增加一次；
 * 如果状态机、事件处理或心跳上报卡住，该计数会停止。
 */
extern volatile uint32_t g_rtos_bms_run_count;

/*
 * CanTask已经完成的100ms发送周期数。
 * 正常情况下大约每秒增加10次。
 */
extern volatile uint32_t g_rtos_can_run_count;
/*
 * CanTask因故障状态变化而执行紧急发送的次数。
 */
extern volatile uint32_t g_rtos_can_urgent_send_count;

/*
 * FaultLogTask已经处理的故障事件数量。
 */
extern volatile uint32_t g_rtos_fault_log_processed_count;

/*
 * 所有应用任务创建完成后剩余的FreeRTOS Heap字节数。
 */
extern volatile uint32_t g_rtos_free_heap_after_create;

/*
 * 各任务历史最小剩余栈空间，单位为32位栈元素。
 */
extern volatile uint32_t g_rtos_health_stack_free_words;
extern volatile uint32_t g_rtos_sensor_stack_free_words;
extern volatile uint32_t g_rtos_bms_stack_free_words;
extern volatile uint32_t g_rtos_can_stack_free_words;
extern volatile uint32_t g_rtos_fault_log_stack_free_words;
extern volatile uint32_t g_rtos_diagnostic_stack_free_words;

/*
 * 运行期间当前剩余的FreeRTOS Heap字节数。
 */
extern volatile uint32_t g_rtos_free_heap_current;


/*
 * 创建当前阶段需要的全部FreeRTOS任务。
 *
 * 返回值：
 * 1：全部任务创建成功；
 * 0：至少一个任务创建失败。
 *
 * 本函数只创建任务，不启动调度器。
 */
uint8_t AppRTOS_CreateTasks(void);


#endif

