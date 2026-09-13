/*
 * 文件名称：app_health_monitor.c
 *
 * 模块名称：系统健康监控与独立看门狗
 *
 * 工作流程：
 * 关键功能执行成功
 *       ↓
 * HealthMonitor_Report()设置对应心跳位
 *       ↓
 * 每100ms调用HealthMonitor_Run()
 *       ↓
 * 检查本窗口是否收齐Sensor、BMS和CAN心跳
 *       ↓
 * 收齐：刷新IWDG
 * 缺失：不刷新IWDG
 *
 * IWDG使用独立的LSI时钟。
 * 即使主时钟、TIM3或主循环停止，IWDG仍然可以复位系统。
 *
 * 当前看门狗典型超时时间约4秒。
 * LSI频率本身存在误差，因此4秒是近似值，不是精确定时。
 * 
 *
 * 并发规则：
 * HealthMonitor_Report()会由多个FreeRTOS任务调用；
 * 心跳位的设置以及“读取后清零”使用短临界区保护，
 * 防止不同任务同时修改时丢失心跳。
 */
#include "app_health_monitor.h"

/*
 * Health心跳位由多个任务并发修改，
 * 使用FreeRTOS任务临界区保护复合读写操作。
 */
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"


/*
 * IWDG使用约40kHz的LSI时钟：
 *
 * 40kHz / 64 = 625Hz
 * (2499 + 1) / 625 = 4秒
 */
#define HEALTH_IWDG_PRESCALER_DIV64    ((uint32_t)0x04U)
#define HEALTH_IWDG_RELOAD_VALUE       ((uint32_t)2499U)

/* IWDG关键字。 */
#define HEALTH_IWDG_KEY_WRITE_ENABLE   ((uint32_t)0x5555U)
#define HEALTH_IWDG_KEY_RELOAD         ((uint32_t)0xAAAAU)
#define HEALTH_IWDG_KEY_START          ((uint32_t)0xCCCCU)


volatile uint8_t g_health_seen_mask = 0U; // 记录已见的心跳位
volatile uint8_t g_health_last_window_mask = 0U;// 记录上一个检查窗口的心跳位
volatile uint32_t g_health_feed_count = 0U;// 记录成功刷新IWDG的累计次数
volatile uint32_t g_health_missed_count = 0U;// 记录心跳不完整、没有刷新IWDG的累计次数
volatile uint8_t g_health_test_stop_feed = 0U;// 看门狗测试开关，设为1后Health Monitor停止刷新IWDG，用于验证硬件复位


/* 诊断计数达到最大值后保持，避免长期运行回绕成0。 */
static void HealthMonitor_IncrementSaturated(
    volatile uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}


void HealthMonitor_Init(void)
{
    g_health_seen_mask = 0U;
    g_health_last_window_mask = 0U;
    g_health_feed_count = 0U;
    g_health_missed_count = 0U;
    g_health_test_stop_feed = 0U;

    /*
     * 调试器暂停CPU时同时暂停IWDG。
     * 避免程序停在断点时被看门狗复位。
     */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    /*
     * 写入0x5555，允许修改IWDG预分频器和重装值。
     */
    IWDG->KR = HEALTH_IWDG_KEY_WRITE_ENABLE;

    /*
     * LSI约40kHz，64分频后约625Hz。
     */
    IWDG->PR = HEALTH_IWDG_PRESCALER_DIV64;

    /*
     * 计数2500次，典型超时时间约4秒。
     */
    IWDG->RLR = HEALTH_IWDG_RELOAD_VALUE;

    /*
     * 装入第一次计数值。
     */
    IWDG->KR = HEALTH_IWDG_KEY_RELOAD;

    /*
     * 启动IWDG，同时启动其所需的LSI时钟。
     *
     * 不在启动前等待IWDG->SR清零：
     * PR和RLR更新依赖LSI时钟，启动前等待可能永久卡住。
     */
    IWDG->KR = HEALTH_IWDG_KEY_START;
}

// 关键功能执行成功后上报对应心跳位。
void HealthMonitor_Report(uint8_t source_mask)
{
    uint8_t valid_source_mask;
    /*
     * 只接受系统定义的健康位。
     * 即使调用者错误传入其他位，也不会污染检查结果。
     */
    valid_source_mask =
        (uint8_t)(source_mask & HEALTH_REQUIRED_MASK);

    /*
     * “读取旧值、按位或、写回”不是一条不可分割的指令。
     * 使用短临界区，防止两个任务同时上报时互相覆盖。
     */
    taskENTER_CRITICAL();
    g_health_seen_mask |= valid_source_mask;
    taskEXIT_CRITICAL();

}

// 每100ms检查一次心跳，并决定是否刷新IWDG。
void HealthMonitor_Run(void)
{
    uint8_t window_mask;

    /*
    * “取得上一窗口结果”和“开始下一窗口”必须是一个整体。
    *
    * 如果读取后、清零前被其他任务上报心跳，
    * 随后的清零会把刚到达的新心跳一起删除。
    */
    taskENTER_CRITICAL();

    window_mask = g_health_seen_mask;
    g_health_seen_mask = 0U;

    taskEXIT_CRITICAL();

    /*
    * last_window_mask只由Health Monitor写入，
    * 用于Watch观察，不需要包含在临界区中。
    */
    g_health_last_window_mask = window_mask;

    /*
     * 测试开关打开时故意不刷新IWDG。
     * 它只用于验证看门狗，正常运行必须保持为0。
     */
    if (g_health_test_stop_feed != 0U)
    {
        HealthMonitor_IncrementSaturated(
            &g_health_missed_count);
        return;
    }

    /*
    * 按位与以后仍等于完整必需掩码，
    * 说明Sensor、BMS和CAN三个关键任务的心跳都已出现。
    */
    if ((window_mask & HEALTH_REQUIRED_MASK) ==
        HEALTH_REQUIRED_MASK)
    {
        IWDG->KR = HEALTH_IWDG_KEY_RELOAD;

        HealthMonitor_IncrementSaturated(
            &g_health_feed_count);
    }
    else
    {
        /*
         * 本窗口心跳不完整，不刷新看门狗。
         * 下一窗口恢复正常时仍有机会重新刷新。
         */
        HealthMonitor_IncrementSaturated(
            &g_health_missed_count);
    }
}
