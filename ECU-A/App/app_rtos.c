/*
 * 文件名称：app_rtos.c
 *
 * 模块名称：FreeRTOS任务组织与迁移入口
 *
 * 模块职责：
 * 1. 集中定义和创建本项目的FreeRTOS任务；
 * 2. 集中管理任务栈大小、优先级和任务句柄；
 * 3. 定义各任务的周期运行方式和阻塞方式；
 * 4. 提供任务运行次数、栈高水位和剩余Heap等诊断数据；
 * 5. 组织DiagnosticTask处理CLI命令和系统资源诊断。
 *
 * 当前任务结构：
 *
 * HealthTask，优先级5：
 * 启动后先等待150ms，随后每100ms检查一次
 * Sensor、BMS和CAN心跳，并决定是否刷新IWDG。
 *
 * SensorTask，优先级4：
 * 每1ms检查ADC/DMA数据，完成传感器换算，
 * 并发布最新传感器快照。
 *
 * BmsTask，优先级3：
 * 每10ms读取最新传感器快照，运行故障管理器、
 * BMS状态机和状态事件处理。
 *
 * CanTask，优先级2：
 * 每100ms发送电压和故障状态，每500ms发送温度状态；
 * 故障状态变化时，可以通过任务通知立即发送故障报文。
 *
 * FaultLogTask，优先级1：
 * 阻塞等待FaultEvent Queue中的故障事件，
 * 收到事件后写入外部SPI Flash。
 *
 * DiagnosticTask，优先级1：
 * 每1ms轮询CLI，每1秒更新任务栈高水位和剩余Heap。
 *
 * 当前主要数据流：
 *
 * TIM3触发ADC和DMA
 *      ↓
 * SensorTask换算并发布传感器快照
 *      ↓
 * BmsTask读取快照，判断故障并运行状态机
 *      ↓
 * ┌──────────────────┬──────────────────┐
 * ↓                  ↓                  ↓
 * 通知CanTask     FaultEvent Queue    上报BMS心跳
 * ↓                  ↓
 * 发送CAN报文     FaultLogTask写Flash
 *
 * SensorTask、BmsTask和CanTask完成关键工作后，
 * 分别向Health模块上报心跳。
 * HealthTask周期检查心跳，决定是否刷新IWDG。
 *
 * 模块边界：
 * 本文件只负责“任务由谁执行、何时执行、如何阻塞”。
 * 传感器换算、故障判断、CAN报文打包和Flash存储等
 * 具体业务逻辑仍由各自应用模块完成。
 */

#include "app_rtos.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_can_service.h"
#include "app_cli.h"
#include "app_bms_state.h"
#include "app_event.h"
#include "app_fault_event.h"
/*
 * SensorTask使用Sensor Service处理ADC数据，
 * 使用GetTick()保持Sensor快照与BmsTask故障计时使用同一个时间来源，
 * 成功发布快照后向Health Monitor上报心跳。
 */
#include "app_sensor.h"
#include "app_health_monitor.h"
#include "bsp_tim.h"

/*
 * DiagnosticTask负责CLI命令解析、串口打印和资源诊断。
 *
 * 当前分配256个32位栈元素：
 * 256 × 4字节 = 1024字节。
 *
 * 压力验证覆盖status、fault、can stats、log和log clear等命令，
 * 栈高水位仍保留了足够余量。
 */
#define APP_RTOS_DIAGNOSTIC_STACK_WORDS       256U

/*
 * SensorTask负责检查ADC/DMA数据，
 * 并执行电压、NTC温度和浮点数换算。
 *
 * 当前分配256个32位栈元素：
 * 256 × 4字节 = 1024字节。
 *
 * 栈大小已经通过正常采样和故障测试后的
 * 栈高水位数据进行验证。
 */
#define APP_RTOS_SENSOR_STACK_WORDS       256U

/*
 * BmsTask负责运行故障管理器、BMS状态机和状态事件。
 *
 * 该执行路径包含FaultManagerSnapshot_t等局部结构体，
 * 状态变化时还可能执行诊断打印，因此比SensorTask
 * 和CanTask实际使用更多栈。
 *
 * 当前分配256个32位栈元素：
 * 256 × 4字节 = 1024字节。
 */
#define APP_RTOS_BMS_STACK_WORDS          256U

/*
 * CanTask负责读取业务快照、组装CAN报文、
 * 计算Alive Counter和CRC8，并调用CAN驱动发送。
 *
 * 当前分配192个32位栈元素：
 * 192 × 4字节 = 768字节。
 */
#define APP_RTOS_CAN_STACK_WORDS          192U

/*
 * FaultLogTask负责从故障事件队列取出事件，
 * 获取Flash Mutex，写入外部SPI Flash并回读校验。
 *
 * 当前分配192个32位栈元素：
 * 192 × 4字节 = 768字节。
 *
 * 故障注入、故障恢复和日志读取测试已经覆盖其主要路径。
 */
#define APP_RTOS_FAULT_LOG_STACK_WORDS    192U

/*
 * HealthTask只负责周期检查关键任务心跳并决定是否刷新IWDG。
 *
 * 当前分配192个32位栈元素：
 * 192 × 4字节 = 768字节。
 *
 * 正常运行、故障和CLI压力测试中，
 * 实测历史最大使用约28 words。
 * 暂时保留当前配置，后续统一优化任务栈时再决定是否缩小。
 */
#define APP_RTOS_HEALTH_STACK_WORDS       192U

/*
 * HealthTask是系统监督任务，优先级设为5。
 *
 * 它每100ms只执行一次短时间检查，
 * 不打印日志、不访问Flash，也不处理复杂业务。
 *
 * 高优先级保证CLI或Flash繁忙时，
 * 健康检查仍能按时执行。
 */
#define APP_RTOS_HEALTH_PRIORITY          \
    (tskIDLE_PRIORITY + 5U)
/*
 * SensorTask优先级为4，仅低于HealthTask。
 *
 * ADC数据是BMS故障判断和CAN状态发送的数据源，
 * 因此需要及时完成换算并发布快照。
 * 当CLI或Flash任务运行时，SensorTask可以抢占它们。
 */
#define APP_RTOS_SENSOR_PRIORITY          \
    (tskIDLE_PRIORITY + 4U)

/*
 * BmsTask优先级为3。
 *
 * SensorTask优先级4：
 * 先生成最新可信传感器快照。
 *
 * BmsTask优先级3：
 * 再使用快照进行故障判断和状态切换。
 *
 * CanTask优先级2：
 * 最后发送BMS处理后的业务状态。
 */
#define APP_RTOS_BMS_PRIORITY             \
    (tskIDLE_PRIORITY + 3U)

/*
 * CanTask优先级为2。
 *
 * 它低于SensorTask和BmsTask，避免通信发送阻塞核心计算；
 * 它高于FaultLogTask和DiagnosticTask，
 * 保证周期报文和紧急故障报文能够及时发送。
 */
#define APP_RTOS_CAN_PRIORITY             \
    (tskIDLE_PRIORITY + 2U)

/*
 * DiagnosticTask优先级为1。
 *
 * 它处理CLI和资源诊断，
 * 这些功能不会直接决定BMS保护和CAN状态发送，
 * 因此放在核心业务任务之后运行。
 */
#define APP_RTOS_DIAGNOSTIC_PRIORITY          \
    (tskIDLE_PRIORITY + 1U)

/*
 * FaultLogTask优先级为1。
 *
 * Flash擦写和串口日志输出耗时较长，
 * 因此它低于Sensor、BMS和CAN任务。
 * 队列为空时任务保持阻塞，不占用CPU。
 */
#define APP_RTOS_FAULT_LOG_PRIORITY       \
    (tskIDLE_PRIORITY + 1U)

/*
 * 各应用任务的句柄。
 *
 * CanTask句柄还用于接收BmsTask的故障变化通知；
 * 其余句柄目前只用于查询任务栈高水位。
 */
static TaskHandle_t g_health_task_handle = 0;
static TaskHandle_t g_sensor_task_handle = 0;
static TaskHandle_t g_bms_task_handle = 0;
static TaskHandle_t g_can_task_handle = 0;
static TaskHandle_t g_fault_log_task_handle = 0;
static TaskHandle_t g_diagnostic_task_handle = 0;


/*
 * 供Keil Watch观察。
 * 每完整运行一次诊断任务循环就加1。
 */
volatile uint32_t g_rtos_diagnostic_run_count = 0U;
/*
 * HealthTask已经完成的健康检查次数。
 *
 * 第一次在系统启动150ms后增加，
 * 以后正常情况下大约每100ms增加一次。
 */
volatile uint32_t g_rtos_health_run_count = 0U;
/*
 * SensorTask每完成一次循环就增加一次。
 * 非static定义便于Keil Watch直接观察。
 */
volatile uint32_t g_rtos_sensor_run_count = 0U;
/*
 * BmsTask每完整执行一轮状态机、事件处理和心跳上报后加1。
 * 供Keil Watch确认BMS任务是否按照10ms周期运行。
 */
volatile uint32_t g_rtos_bms_run_count = 0U;
/*
 * CanTask每完成一个100ms基础周期后增加一次。
 * 正常情况下大约每秒增加10次。
 */
volatile uint32_t g_rtos_can_run_count = 0U;
/*
 * CanTask因故障状态变化而执行紧急发送的次数。
 * 仅用于Keil Watch验证任务通知是否生效。
 */
volatile uint32_t g_rtos_can_urgent_send_count = 0U;
/*
 * FaultLogTask成功等到并处理的故障事件数量。
 *
 * 启动BOOT日志不经过故障事件队列，
 * 因此系统刚启动时该值仍然为0。
 */
volatile uint32_t g_rtos_fault_log_processed_count = 0U;

/*
 * 所有应用任务创建完成后剩余的FreeRTOS Heap字节数。
 * 用于判断继续创建任务时是否还有内存余量。
 */
volatile uint32_t g_rtos_free_heap_after_create = 0U;


/*
 * 各任务历史上最少剩余的栈空间：
 * 从任务创建以来，最危险时刻仍然剩下多少栈。
 * 
 * 单位是StackType_t栈元素，不是字节。
 * STM32F103中一个栈元素为4字节。
 */
volatile uint32_t g_rtos_health_stack_free_words = 0U;
volatile uint32_t g_rtos_sensor_stack_free_words = 0U;
volatile uint32_t g_rtos_bms_stack_free_words = 0U;
volatile uint32_t g_rtos_can_stack_free_words = 0U;
volatile uint32_t g_rtos_fault_log_stack_free_words = 0U;
volatile uint32_t g_rtos_diagnostic_stack_free_words = 0U;

/*
 * 系统运行过程中当前剩余的FreeRTOS Heap字节数。
 */
volatile uint32_t g_rtos_free_heap_current = 0U;

/*
 * 更新FreeRTOS任务栈和Heap诊断数据。
 *
 * 本函数由低优先级DiagnosticTask每1秒调用一次，
 * 避免在SensorTask等高频任务中反复扫描栈空间
 * 
 * UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask )：
 * 任务栈高水位标记，用来查看任务运行过程中栈的最小剩余空间，评估栈是否够用
 */
static void AppRTOS_UpdateResourceDiagnostics(void)
{
    /*
     * 任务创建成功后，句柄才会变成有效值。
     * 通过句柄查询HealthTask历史最少剩余栈空间。
     */
    if (g_health_task_handle != 0)
    {
        g_rtos_health_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_health_task_handle);
    }

    if (g_sensor_task_handle != 0)
    {
        g_rtos_sensor_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_sensor_task_handle);
    }

    if (g_bms_task_handle != 0)
    {
        g_rtos_bms_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_bms_task_handle);
    }

    if (g_can_task_handle != 0)
    {
        g_rtos_can_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_can_task_handle);
    }

    if (g_fault_log_task_handle != 0)
    {
        g_rtos_fault_log_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_fault_log_task_handle);
    }

    if (g_diagnostic_task_handle != 0)
    {
        g_rtos_diagnostic_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_diagnostic_task_handle);
    }

    g_rtos_free_heap_current =
        (uint32_t)xPortGetFreeHeapSize();
}







/*
 * 独立传感器任务。
 * 负责读取、换算和发布
 * 当前采用1ms周期检查DMA ready标志，
 * 不在这个阶段引入DMA到任务的直接通知。
 */
static void AppRTOS_SensorTask(void *argument)
{
    TickType_t last_wake_time;
    uint32_t now_ms;
    uint8_t sample_updated;

    (void)argument;

    /*
     * 记录FreeRTOS任务的初始周期基准。
     */
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        /*
         * 暂时使用TIM3时间作为传感器快照时间，
         * 保证SensorTask与BmsTask的故障计时使用同一时间来源。
         */
        now_ms = GetTick();

        /*
         * 有新DMA数据时完成换算并发布快照。
         * 没有新数据时不覆盖上一帧。
         */
        sample_updated =
            Sensor_ServiceUpdate(now_ms);

        /*
         * 只有真正发布新快照后才上报Sensor心跳。
         *
         * 如果任务仍在运行但ADC/DMA停止，
         * sample_updated会持续为0，Health Monitor就不会收到Sensor心跳，
         * 最终由IWDG复位系统。
         */
        if (sample_updated != 0U)
        {
            HealthMonitor_Report(
                HEALTH_SOURCE_SENSOR);
        }

        /*
         * 这个计数证明SensorTask本身完成了一次循环。
         * 即使DMA没有新数据，它仍然会增加。
         */
        g_rtos_sensor_run_count++;

        /*
         * 保持固定1ms任务周期。
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(1U));
    }
}


/*
 * 独立BMS核心任务。
 *
 * 模块职责：
 * 1. 每10ms执行一次BMS状态机；
 * 2. 使用SensorTask发布的最新快照运行故障判断；
 * 3. 处理本轮产生的启动、故障和恢复状态事件；
 * 4. 完整执行成功后向Health Monitor报告BMS心跳。
 *
 * 本任务不负责：
 * 1. ADC采集和传感器数值换算；
 * 2. CAN周期发送；
 * 3. Flash日志写入；
 * 4. CLI命令解析；
 * 5. 最终决定是否刷新IWDG。
 */
static void AppRTOS_BmsTask(void *argument)
{
    TickType_t last_wake_time;
    uint16_t previous_fault_mask;

    /*
     * 当前没有向BmsTask传递创建参数。
     */
    (void)argument;

    /*
     * 保存任务的初始周期基准。
     *
     * vTaskDelayUntil()以后会在该值的基础上每次增加10ms，
     * 从而保持稳定周期。
     */
    last_wake_time = xTaskGetTickCount();
    /*
    * 保存进入任务时的故障状态。
    * 后续只有故障掩码真正变化时才通知CanTask。
    */
    previous_fault_mask = g_fault_code;

    for (;;)
    {
        /*
         * 第一步：执行BMS状态机。
         *
         * 它会读取最新Sensor快照、更新FaultManager，
         * 根据当前状态检查启动键或恢复键，并产生状态事件。
         */
        BMS_StateMachine();

        /*
         * 第二步：处理本轮状态事件。
         *
         * 例如状态机确认关键故障并压入EVENT_FAULT后，
         * 这里会在同一个10ms任务周期内处理该事件，
         * 使状态进入BMS_FAULT。
         */
        Event_Process();
        /*
        * 检查本轮BMS处理前后，活动故障掩码是否发生变化。
        *
        * 这里只发送通知，不直接发送CAN，
        * 从而保持CanTask是CAN硬件的唯一使用者。
        */
        if (g_fault_code != previous_fault_mask)
        {
            previous_fault_mask = g_fault_code;

            if (g_can_task_handle != 0)
            {
                xTaskNotifyGive(g_can_task_handle);
            }
        }

        /*
         * 第三步：报告BMS任务心跳。
         *
         * 必须放在状态机和事件处理全部返回以后。
         * 如果前面的核心逻辑卡死，程序执行不到这里，
         * Health Monitor就不会误认为BMS仍然健康。
         */
        HealthMonitor_Report(
            HEALTH_SOURCE_BMS);

        /*
         * 只有整轮核心逻辑执行完成，运行计数才增加。
         */
        g_rtos_bms_run_count++;

        /*
         * 以固定10ms周期运行。
         *
         * BmsTask进入阻塞后，CPU可以运行其他Ready任务；
         * 如果没有应用任务就绪，则运行IdleTask。
         * 任务阻塞期间不会像空循环一样持续占用CPU。
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(10U));
    }
}

/*
 * 独立CAN发送任务。
 *
 * 两种唤醒原因：
 * 1. 等待达到100ms周期：发送周期状态报文；
 * 2. 收到BmsTask通知：立即发送最新故障状态。
 * ulTaskNotifyTake()：负责等待“故障变化”或“周期到期”。
 * next_periodic_time：负责保证100ms报文周期不会因为故障通知而重新计时。
 * 
 * 无论哪种情况，CAN硬件始终只由本任务访问。
 */
static void AppRTOS_CanTask(void *argument)
{
    TickType_t next_periodic_time;
    TickType_t now;
    TickType_t wait_ticks;
    uint32_t notify_count;
    uint8_t temperature_cycle = 0U;

    (void)argument;

    /*
     * 第一轮周期发送安排在任务启动100ms之后。
     */
    next_periodic_time =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(100U);

    for (;;)
    {
        now = xTaskGetTickCount();

        /*
         * 计算距离下一个100ms周期还剩多长时间。
         *
         * 如果已经到期，等待时间设为0，
         * 任务直接执行周期发送。
         */
        if ((int32_t)(next_periodic_time - now) > 0)
        {
            wait_ticks = next_periodic_time - now;
        }
        else
        {
            wait_ticks = 0U;
        }

        /*
         * 等待两种事件之一：
         *
         * 1. BmsTask发送任务通知；
         * 2. wait_ticks超时，说明周期发送时间到达。
         *
         * pdTRUE表示取出通知后把通知计数清零。
         */
        notify_count =
            ulTaskNotifyTake(
                pdTRUE,
                wait_ticks);

        /*
         * 返回值非0，说明故障掩码发生了变化。
         * 立即发送最新的完整故障状态。
         */
        if (notify_count != 0U)
        {
            AppCAN_SendFaultStatus();
            g_rtos_can_urgent_send_count++;
        }

        now = xTaskGetTickCount();

        /*
         * 无论中途是否收到过故障通知，
         * 原来的100ms周期发送仍然独立保持。
         */
        if ((int32_t)(now - next_periodic_time) >= 0)
        {
            AppCAN_SendVoltageStatus();
            AppCAN_SendFaultStatus();

            temperature_cycle++;

            if (temperature_cycle >= 5U)
            {
                temperature_cycle = 0U;
                AppCAN_SendTemperatureStatus();
            }
            /*
            * 只有本轮周期CAN处理全部返回后，才报告CanTask心跳。
            *
            * 如果CAN发送路径卡死，程序无法执行到这里，
            * Health Monitor将收不到CAN心跳并停止喂狗。
            */
            HealthMonitor_Report(
                HEALTH_SOURCE_CAN);

            g_rtos_can_run_count++;

            /*
             * 在原周期基准上增加100ms，
             * 防止任务运行时间逐渐造成周期漂移。
             */
            next_periodic_time +=
                pdMS_TO_TICKS(100U);
        }
    }
}

/*
 * 独立故障日志任务。
 *
 * 数据来源：
 * BmsTask产生的FaultEvent Queue。
 *
 * 执行方式：
 * 队列为空时阻塞，不占用CPU；
 * 队列出现事件时由FreeRTOS自动唤醒；
 * 每次按先入先出顺序处理一条事件。
 *
 * 本任务不判断故障、不修改BMS状态，
 * 也不直接控制蜂鸣器和CAN。
 * 
 * 运行逻辑：
队列为空
   ↓
FaultLogTask阻塞
   ↓
BmsTask确认故障并入队
   ↓
FreeRTOS唤醒FaultLogTask
   ↓
取出事件
   ↓
取得Flash Mutex
   ↓
写入、回读校验、释放Mutex
   ↓
再次等待下一条事件
 */
static void AppRTOS_FaultLogTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /*
         * portMAX_DELAY表示队列为空时一直阻塞。
         *
         * BmsTask成功写入一条FaultEvent_t后，
         * FreeRTOS会自动把本任务从Blocked状态唤醒。
         */
        FaultEvent_ProcessOne(
            portMAX_DELAY);

        /*
         * ProcessOne返回时，说明已经从队列中取得并处理了一条事件。
         */
        g_rtos_fault_log_processed_count++;
    }
}

/*
 * 独立系统健康监控任务。
 *
 * 模块职责：
 * 1. 每100ms检查Sensor、BMS和CAN任务的心跳；
 * 2. 心跳完整时刷新独立看门狗IWDG；
 * 3. 心跳持续缺失时停止喂狗，由IWDG复位系统。
 *
 * 本任务不负责：
 * 1. 产生各业务任务的心跳；
 * 2. 执行ADC采集、BMS状态机或CAN发送；
 * 3. 打印日志或处理CLI命令。
 *
 * 第一次检查安排在启动后150ms，
 * 避开CanTask在100ms、200ms、300ms执行的周期边界。
 */
static void AppRTOS_HealthTask(void *argument)
{
    TickType_t last_wake_time;

    /*
     * 当前创建任务时不会传入参数。
     */
    (void)argument;

    /*
     * 记录HealthTask第一次运行时的FreeRTOS节拍。
     */
    last_wake_time = xTaskGetTickCount();

    /*
     * 第一次健康检查延迟到启动后150ms。
     *
     * CanTask第一次周期发送发生在100ms，
     * 因此它有时间完成CAN发送并上报心跳，
     * 避免HealthTask在同一个100ms边界上抢先检查。
     */
    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(150U));

    for (;;)
    {
        /*
         * 读取并清空本窗口的心跳位，
         * 然后根据心跳是否完整决定是否刷新IWDG。
         */
        HealthMonitor_Run();

        /*
         * 只有HealthMonitor_Run()完整返回后才增加。
         * 用于证明HealthTask自身一直在周期运行。
         */
        g_rtos_health_run_count++;

        /*
         * 第一次检查发生在150ms，
         * 后续检查发生在250ms、350ms、450ms……
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(100U));
    }
}


/*
 * 独立诊断与维护任务。
 *
 * 模块职责：
 * 1. 每1ms处理一次USART CLI接收缓冲区；
 * 2. 每轮最多处理16个字符，限制单次执行时间；
 * 3. 每1秒更新各任务栈高水位和剩余Heap；
 * 4. 提供状态查询、故障注入、日志和看门狗测试入口。
 *
 * 本任务不执行传感器采集、BMS故障判断、
 * CAN周期发送、故障日志写入或IWDG健康决策。
 *
 * 当前使用优先级1，核心业务任务可以随时抢占它。
 */
static void AppRTOS_DiagnosticTask(void *argument)
{
    TickType_t last_wake_time;
    TickType_t resource_check_time;
    TickType_t now;

    (void)argument;

    /*
     * 保存任务第一次运行时的FreeRTOS节拍。
     * 后续vTaskDelayUntil会以它为周期基准。
     */
    last_wake_time = xTaskGetTickCount();
    resource_check_time = last_wake_time;

    /*
    * 第一次进入DiagnosticTask时立即记录一次，
    * 后续运行和压力测试会继续刷新历史最小余量。
    */
    AppRTOS_UpdateResourceDiagnostics();

    for (;;)
    {
        /*
         * CLI原来由main主循环轮询，
         * 现在由DiagnosticTask周期轮询。
         */
        CLI_Process();
        /*
        * 每1秒更新一次任务栈高水位和剩余Heap。
        */
        now = xTaskGetTickCount();

        if ((now - resource_check_time) >=
            pdMS_TO_TICKS(1000U))
        {
            /*
            * 直接使用当前时间重新建立基准。
            * 即使CLI曾长时间打印，也不会连续补做多次扫描。
            */
            resource_check_time = now;

            AppRTOS_UpdateResourceDiagnostics();
        }
        /*
         * 必须放在两个主要函数都返回之后。
         * 如果其中一个函数卡住，计数将停止增加。
         */
        g_rtos_diagnostic_run_count++;

        /*
         * 让任务按照固定1ms周期运行。
         * 任务阻塞期间CPU可以执行其他就绪任务。
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(1U));
    }
}


/*
 * 创建当前迁移阶段的全部应用任务。
 *
 * 创建顺序：
 * 1. HealthTask：检查关键任务心跳并决定是否刷新IWDG；
 * 2. SensorTask：处理ADC/DMA数据并发布传感器快照；
 * 3. BmsTask：运行故障判断、状态机和状态事件；
 * 4. CanTask：统一发送周期报文和紧急故障报文；
 * 5. FaultLogTask：阻塞等待故障事件并写入Flash；
 * 6. DiagnosticTask：处理CLI和资源诊断。
 *
 * 创建顺序不等于任务运行顺序。
 * 调度器启动后，FreeRTOS根据任务优先级选择任务运行。
 *
 * 本函数只创建任务，不启动调度器。
 */
uint8_t AppRTOS_CreateTasks(void)
{
    BaseType_t create_result;

    g_health_task_handle = 0;
    g_sensor_task_handle = 0;
    g_bms_task_handle = 0;
    g_can_task_handle = 0;
    g_fault_log_task_handle = 0;
    g_diagnostic_task_handle = 0;

    //HealthTask完成了多少次检查
    g_rtos_health_run_count = 0U;

    //HealthTask历史最少剩余多少栈
    g_rtos_health_stack_free_words = 0U;
    g_rtos_sensor_stack_free_words = 0U;
    g_rtos_bms_stack_free_words = 0U;
    g_rtos_can_stack_free_words = 0U;
    g_rtos_fault_log_stack_free_words = 0U;
    g_rtos_diagnostic_stack_free_words = 0U;

    g_rtos_free_heap_current = 0U;

    /*
     * 创建独立HealthTask。
     *
     * 该任务启动后先等待150ms，
     * 随后每100ms检查一次关键任务心跳。
     */
    create_result = xTaskCreate(
        AppRTOS_HealthTask,          // 要运行的任务函数
        "Health",                    // 调试器中显示的任务名称
        APP_RTOS_HEALTH_STACK_WORDS, // 分配192 words任务栈
        0,                           // 不向任务传入参数
        APP_RTOS_HEALTH_PRIORITY,    // 优先级5
        &g_health_task_handle);      // 保存创建后的任务句柄

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
     * 创建独立SensorTask。
     */
    create_result = xTaskCreate(
        AppRTOS_SensorTask,
        "Sensor",
        APP_RTOS_SENSOR_STACK_WORDS,
        0,
        APP_RTOS_SENSOR_PRIORITY,
        &g_sensor_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
     * 创建独立BmsTask。
     */
    create_result = xTaskCreate(
        AppRTOS_BmsTask,
        "BMS",
        APP_RTOS_BMS_STACK_WORDS,
        0,
        APP_RTOS_BMS_PRIORITY,
        &g_bms_task_handle);

    if(create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * 创建独立CanTask。
    * 从此周期CAN发送由该任务统一执行。
    */
    create_result = xTaskCreate(
        AppRTOS_CanTask,
        "CAN",
        APP_RTOS_CAN_STACK_WORDS,
        0,
        APP_RTOS_CAN_PRIORITY,
        &g_can_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * 创建独立FaultLogTask。
    *
    * 任务启动后会立即阻塞在故障事件队列上，
    * 直到BmsTask产生第一条故障确认或清除事件。
    */
    create_result = xTaskCreate(
        AppRTOS_FaultLogTask,
        "FaultLog",
        APP_RTOS_FAULT_LOG_STACK_WORDS,
        0,
        APP_RTOS_FAULT_LOG_PRIORITY,
        &g_fault_log_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * 创建独立诊断任务。
    *
    * 任务负责CLI轮询和资源诊断，
    * 不参与BMS实时保护和CAN周期发送。
    */
    create_result = xTaskCreate(
        AppRTOS_DiagnosticTask,
        "Diag",
        APP_RTOS_DIAGNOSTIC_STACK_WORDS,
        0,
        APP_RTOS_DIAGNOSTIC_PRIORITY,
        &g_diagnostic_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * 记录当前剩余Heap，供Keil Watch检查。
    *
    * 该值不包含各任务已经分配出去的栈和TCB空间。
    */
    g_rtos_free_heap_after_create =
        (uint32_t)xPortGetFreeHeapSize();

    return 1U;
}
