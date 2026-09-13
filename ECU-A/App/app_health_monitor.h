/*
 * 文件名称：app_health_monitor.h
 *
 * 模块名称：系统健康监控与独立看门狗公共接口
 *
 * 模块职责：
 * 1. 定义关键任务的健康心跳位；
 * 2. 接收Sensor、BMS和CAN任务的运行心跳；
 * 3. 对外提供100ms健康检查入口；
 * 4. 只有关键任务全部正常时才允许刷新IWDG。
 *
 * 模块边界：
 * 本模块不处理传感器数据、不判断电池故障、
 * 不发送CAN报文，也不执行具体业务任务。
 *
 * 当前并发规则：
 * 多个FreeRTOS任务可以调用HealthMonitor_Report()；
 * 心跳位的并发修改由模块内部临界区保护。
 */
#ifndef APP_HEALTH_MONITOR_H
#define APP_HEALTH_MONITOR_H

#include <stdint.h>


/* Sensor Service成功产生新快照时上报。 */
#define HEALTH_SOURCE_SENSOR       ((uint8_t)0x01U)

/* BMS 10ms核心任务完整执行返回后上报。 */
#define HEALTH_SOURCE_BMS          ((uint8_t)0x02U)

/*
 * CanTask完成一次100ms周期发送后上报。
 */
#define HEALTH_SOURCE_CAN          ((uint8_t)0x04U)


/*
 * 每个100ms检查窗口中必须出现的全部关键任务心跳。
 * Sensor = 0x01  二进制：0000 0001
 * BMS    = 0x02  二进制：0000 0010
 * CAN    = 0x04  二进制：0000 0100
 * --------------------------------
 * 合计   = 0x07  二进制：0000 0111
 */
#define HEALTH_REQUIRED_MASK       \
    ((uint8_t)(HEALTH_SOURCE_SENSOR | \
               HEALTH_SOURCE_BMS | \
               HEALTH_SOURCE_CAN))


/* 当前检查窗口中已经收到的心跳。 */
extern volatile uint8_t g_health_seen_mask;

/* 上一个检查窗口实际收到的心跳。 */
extern volatile uint8_t g_health_last_window_mask;

/* 成功刷新IWDG的累计次数。 */
extern volatile uint32_t g_health_feed_count;

/* 心跳不完整、没有刷新IWDG的累计次数。 */
extern volatile uint32_t g_health_missed_count;

/*
 * 看门狗测试开关。
 * 设为1后Health Monitor停止刷新IWDG，用于验证硬件复位。
 */
extern volatile uint8_t g_health_test_stop_feed;


/* 初始化健康状态和STM32独立看门狗。 */
void HealthMonitor_Init(void);

/*
 * 关键任务完整执行成功后上报对应心跳位。
 *
 * 当前接口使用FreeRTOS任务临界区，只允许在任务上下文调用，
 * 不允许直接在中断中调用。
 */
void HealthMonitor_Report(uint8_t source_mask);

/* 每100ms检查一次心跳，并决定是否刷新IWDG。 */
void HealthMonitor_Run(void);


#endif

