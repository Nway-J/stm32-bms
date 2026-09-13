/*
 * 文件名称：app_fault_event.c
 *
 * 模块名称：故障边沿事件RAM队列
 *
 * 模块职责：
 * 1. 在故障确认或清除时保存一份固定的现场快照；
 * 2. 使用FreeRTOS Queue把故障事件从BMS执行路径传给日志处理路径；
 * 3. 保证不同任务并发读写队列时，队列索引和事件内容不会被破坏；
 * 4. 每次从队列取出一条事件，写入Flash V2并输出串口诊断。
 *
 * 当前数据流：
 * FaultManager发现故障变化
 *          ↓
 * FaultEvent_Push()非阻塞入队
 *          ↓
 * FreeRTOS Queue保存事件副本
 *          ↓
 * FaultEvent_ProcessOne()出队
 *          ↓
 * Flash日志和USART诊断
 *
 * 可靠性策略：
 * 队列满时丢弃新事件并增加dropped_count；
 * 故障保护路径不会等待Flash，也不会因为日志队列满而阻塞。
 *
 * 模块边界：
 * 本模块不判断故障条件、不切换BMS状态、不控制蜂鸣器，
 * 也不负责传感器采集和CAN发送。
 */ 
#include "app_fault_event.h" 
 
#include "FreeRTOS.h"
#include "queue.h"

#include <stdio.h> 
 
#include "app_flash_log.h"
#include "app_fault_manager.h"

/*
 * FreeRTOS静态队列需要三部分：
 *
 * 1. g_fault_event_queue_control：
 *    FreeRTOS保存队列的读写位置、当前数量等管理信息。
 *
 * 2. g_fault_event_queue_storage：
 *    真正保存8条FaultEvent_t事件的RAM空间。
 *
 * 3. g_fault_event_queue_handle：
 *    其他函数操作该队列时使用的句柄。
 *
 * 队列使用静态内存，不会从FreeRTOS Heap动态申请空间。
 */
static StaticQueue_t g_fault_event_queue_control;

static uint8_t g_fault_event_queue_storage[
    FAULT_EVENT_QUEUE_CAPACITY * sizeof(FaultEvent_t)];

static QueueHandle_t g_fault_event_queue_handle = 0;
 
// 非static是为了让Keil Watch能够直接观察，业务模块只能读取。 
volatile FaultEventDebug_t g_fault_event_debug; 
 
// 诊断计数达到最大值后保持不变，避免长期运行后回绕成0。 
static void FaultEvent_IncrementSaturated(volatile uint32_t *value) 
{ 
    if (*value < 0xFFFFFFFFU) 
    { 
        (*value)++; 
    } 
} 
 
// 把电压从V转换为mV，并限制在uint16_t范围内。 
static uint16_t FaultEvent_ToMillivolts(float voltage_v) 
{ 
    float value = voltage_v * 1000.0f; 
 
    if (value <= 0.0f) 
    { 
        return 0U; 
    } 
 
    if (value >= 65535.0f) 
    { 
        return 0xFFFFU; 
    } 
 
    return (uint16_t)(value + 0.5f); 
} 
 
// 把温度从℃转换为0.1℃，正负数都执行四舍五入并限制范围。 
static int16_t FaultEvent_ToDeciCelsius(float temperature_c) 
{ 
    float value = temperature_c * 10.0f; 
 
    if (value <= -32768.0f) 
    { 
        return (int16_t)-32768; 
    } 
 
    if (value >= 32767.0f) 
    { 
        return (int16_t)32767; 
    } 
 
    return (value >= 0.0f) ? 
           (int16_t)(value + 0.5f) : 
           (int16_t)(value - 0.5f); 
} 
 
// 逐字段清除事件，避免依赖memset和不同运行库实现。 
static void FaultEvent_ClearEvent(volatile FaultEvent_t *event) 
{ 
    event->timestamp_ms = 0U; 
    event->changed_mask = 0U; 
    event->active_mask = 0U; 
    event->cell_voltage_mv = 0U; 
    event->temperature_dC = 0; 
    event->event_type = 0U; 
    event->bms_state = 0U; 
    event->valid_flags = 0U; 
} 
 
void FaultEvent_Init(void)
{
    /*
     * 第一次初始化时，为队列绑定固定的控制结构和存储空间。
     *
     * 队列长度：
     * FAULT_EVENT_QUEUE_CAPACITY，也就是8条。
     *
     * 每条数据大小：
     * sizeof(FaultEvent_t)。
     */
    if (g_fault_event_queue_handle == 0)
    {
        g_fault_event_queue_handle =
            xQueueCreateStatic(
                FAULT_EVENT_QUEUE_CAPACITY, //队列最多保存多少条
                sizeof(FaultEvent_t),   //每条数据占多少字节
                g_fault_event_queue_storage, //数据存储空间
                &g_fault_event_queue_control); //队列管理结构
    }
    else
    {
        /*
         * 如果以后再次调用初始化函数，
         * 清空队列中未处理的旧事件，但不重新申请内存。
         */
        (void)xQueueReset(g_fault_event_queue_handle);
    }

    FaultEvent_ClearEvent(
        &g_fault_event_debug.last_pushed);

    FaultEvent_ClearEvent(
        &g_fault_event_debug.last_processed);

    //便于KEIL调试查看的变量
    g_fault_event_debug.pushed_count = 0U;
    g_fault_event_debug.processed_count = 0U;
    g_fault_event_debug.dropped_count = 0U;
    g_fault_event_debug.depth = 0U;
    g_fault_event_debug.last_pushed_valid = 0U;
    g_fault_event_debug.last_processed_valid = 0U;
}

/*
故障刚确认或刚清除时，把当时的故障、电压、温度、BMS状态和时间复制到RAM队列
检查参数
   ↓
检查队列是否已满
   ↓
找到tail指向的空位置
   ↓
填写事件基本信息
   ↓
填写有效的电压和温度
   ↓
更新调试信息
   ↓
移动tail
   ↓
count增加
   ↓
返回成功
*/
uint8_t FaultEvent_Push(FaultEventType_t event_type, //故障确认还是故障清除
                        uint16_t changed_mask, //本次新发生或新清除的故障
                        uint16_t active_mask, //变化后仍然存在的全部故障
                        const SensorSample_t *sample, //传入的测量快照，可能为NULL
                        uint8_t bms_state, //BMS状态机状态，取值范围0~255
                        uint32_t timestamp_ms) //事件发生时间，单位ms
{ 
    FaultEvent_t event; 
 
    /*
     * 没有故障变化，或者事件类型非法时，
     * 不生成没有实际意义的故障记录。
     */
    if ((changed_mask == 0U) ||
        ((event_type != FAULT_EVENT_CONFIRMED) &&
         (event_type != FAULT_EVENT_CLEARED)))
    {
        return 0U;
    }

    /*
     * 队列没有成功初始化时不能继续操作。
     */
    if (g_fault_event_queue_handle == 0)
    {
        return 0U;
    }

    /*
     * 先在当前函数的局部变量中组装完整事件。
     * 所有字段填写完成以后，才把事件整体复制到队列。
     */
    event.timestamp_ms = timestamp_ms;
    event.changed_mask = changed_mask;
    event.active_mask = active_mask;
    event.cell_voltage_mv = 0U;
    event.temperature_dC = 0;
    event.event_type = (uint8_t)event_type;
    event.bms_state = bms_state;
    event.valid_flags = 0U;

    /*
     * 电压和温度分别判断有效性。
     * 例如NTC故障导致温度无效时，仍然可以保存可信的电池电压。
     */
    if (sample != 0)
    {
        if (sample->voltage_valid != 0U)
        {
            event.cell_voltage_mv =
                FaultEvent_ToMillivolts(    //转换电压
                    sample->cell_voltage_v);

            event.valid_flags |=
                FAULT_EVENT_VALID_VOLTAGE; //确认数据有效
        }

        if (sample->temperature_valid != 0U)
        {
            event.temperature_dC =
                FaultEvent_ToDeciCelsius(
                    sample->temperature_c);

            event.valid_flags |=
                FAULT_EVENT_VALID_TEMPERATURE;
        }
    }

    /*
     * 故障确认事件命中当前测试注入掩码时，
     * 在日志中标记为测试注入来源。
     */
    if ((event_type == FAULT_EVENT_CONFIRMED) &&
        ((changed_mask &
          FaultManager_GetInjectionMask()) != 0U))
    {
        event.valid_flags |=
            FAULT_EVENT_FLAG_INJECTED;
    }

    /*
     * 等待时间使用0，表示立即尝试入队。
     *
     * 队列有空间：复制event并返回pdPASS。
     * 队列已满：立即失败，不阻塞BMS保护流程。
     */
    if (xQueueSend(
            g_fault_event_queue_handle,
            &event,
            0U) != pdPASS)
    {
        //诊断计数达到最大值后保持不变，避免长期运行后回绕成0
        FaultEvent_IncrementSaturated(
            &g_fault_event_debug.dropped_count);

        return 0U;
    }

    /*
     * 只有真正成功入队后，才更新成功诊断信息。
     */
    g_fault_event_debug.last_pushed = event;
    g_fault_event_debug.last_pushed_valid = 1U;

    FaultEvent_IncrementSaturated(
        &g_fault_event_debug.pushed_count);

    g_fault_event_debug.depth =
        (uint8_t)uxQueueMessagesWaiting(
            g_fault_event_queue_handle);

    return 1U;
}
 
/*
它负责从队列中取出最早的一条故障事件。
检查调用参数
    ↓
检查队列是否为空
    ↓
从head位置复制事件
    ↓
head移动到下一个位置
    ↓
队列事件数量减1
    ↓
更新Watch诊断变量
    ↓
返回成功
*/
uint8_t FaultEvent_Pop(
    FaultEvent_t *event,
    TickType_t wait_ticks)
{
    /*
     * 调用者没有提供接收地址，
     * 或者队列没有初始化时，直接返回失败。
     */
    if ((event == 0) ||
        (g_fault_event_queue_handle == 0))
    {
        return 0U;
    }

    /*
    * wait_ticks决定队列为空时的行为：
    *
    * 0：
    * 立即返回，不阻塞当前任务。
    *
    * 有限Tick数：
    * 最多等待指定时间。
    *
    * portMAX_DELAY：
    * 一直阻塞，直到生产者写入新事件。
    *
    * 任务阻塞期间不占用CPU。
    */
    if (xQueueReceive(
            g_fault_event_queue_handle, //队列句柄
            event,                  //接收数据地址
            wait_ticks) != pdPASS) //最长等待时间
    {
        return 0U;
    }

    /*
     * 出队成功后再更新诊断数据。
     */
    g_fault_event_debug.last_processed = *event;
    g_fault_event_debug.last_processed_valid = 1U;

    FaultEvent_IncrementSaturated(
        &g_fault_event_debug.processed_count);

    //把FreeRTOS内部的当前消息数量复制出来供Watch观察
    g_fault_event_debug.depth =
        (uint8_t)uxQueueMessagesWaiting(
            g_fault_event_queue_handle);

    return 1U;
}
 

/*
 * 每次最多消费一条故障边沿事件。
 *
 * 执行顺序：
 * 从RAM队列取出最早事件
 *        ↓
 * 写入Flash V2并回读验证
 *        ↓
 * 输出事件诊断信息
 *
 * 队列为空时立即返回，不占用调度循环时间。
 */
void FaultEvent_ProcessOne(TickType_t wait_ticks)
{
    FaultEvent_t event;
    uint8_t flash_write_ok;

    /*
     * 返回1：成功取出一条事件。
     * 返回0：队列为空，不需要做任何处理。
     */
    if (FaultEvent_Pop(
            &event,
            wait_ticks) == 0U)
    {
        /*
        * 队列为空或接收失败时，event中没有有效数据，
        * 必须立即返回，不能继续访问它。
        */
        return;
    }

    /*
     * 事件已经从RAM队列中复制到局部变量event，
     * 所以后续Flash写入期间原队列位置可以安全复用。
     */
    flash_write_ok = FlashLog_WriteEvent(&event);

    /*
     * Flash失败不能让状态机卡住。
     * 失败次数由g_flash_log_write_fail_count累计，
     * 这里额外打印是哪一条故障事件保存失败。
     */
    if (flash_write_ok == 0U)
    {
        printf("[FAULT_EVT] Flash write failed: type=%u changed=0x%04X\r\n",
               (unsigned int)event.event_type,
               (unsigned int)event.changed_mask);
    }

    printf("[FAULT_EVT] type=%u changed=0x%04X active=0x%04X time=%lu flash=%u\r\n",
           (unsigned int)event.event_type,
           (unsigned int)event.changed_mask,
           (unsigned int)event.active_mask,
           (unsigned long)event.timestamp_ms,
           (unsigned int)flash_write_ok);
}
