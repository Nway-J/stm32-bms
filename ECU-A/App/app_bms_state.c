/*
 * 模块：电池监测系统状态机。
 * 状态：INIT、SELF_CHECK、STANDBY、MONITOR、FAULT。
 * 功能：组织初始化、自检、监测、故障报警、恢复请求和本地状态指示。
 * 边界：不控制真实充放电，不模拟充电SOC，也不能切断电池电流。
 * 数据边界：只读取Sensor Service发布的快照，不直接消费ADC/DMA标志。
 * 
 * 通信边界：
 * 本模块只产生当前BMS状态和活动故障状态，
 * 不组装CAN应用报文、不管理Alive Counter，也不调用CAN发送驱动。
 */
// 设计原则：
//   1. Task只检测条件，不直接改状态
//   2. 状态切换统一在 BMS_EventHandle 中完成
//   3. 通过 Event_Push/Event_Process 解耦检测和切换

#include "app_bms_state.h"
#include "app_event.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_usart.h"
#include "app_flash_log.h"
#include "app_sensor.h"
#include "bsp_buzzer.h"
#include "app_fault_manager.h"
#include "app_fault_event.h"


// ─── 状态机全局变量 ───
static BMS_State_t g_bms_state = BMS_INIT;  // 当前BMS状态
uint16_t g_fault_code = 0U;       // 当前16位活动故障掩码，与故障管理器active_mask一致
// 记录本次上电自检是否失败；自检失败后要求复位系统重新自检。
static uint8_t g_self_check_failed = 0U;









//====================================状态接口=====================================

// BMS_SetState — 设置BMS状态（直接修改状态变量）
// 由BMS_EventHandle调用，Task不直接调用（除INIT/SELF_CHECK外）
void BMS_SetState(BMS_State_t state)
 { 
    // 直接修改全局状态变量
	 g_bms_state = state; 

     //判断新状态是否为故障状态
     if(state == BMS_FAULT)
     {
         // 进入故障状态时，关闭LED，开启蜂鸣器
         LED_OFF();
         BUZZER_On();
     }
     else
     {
         // 非故障状态，关闭蜂鸣器
         BUZZER_Off();
     }
 }


// BMS_GetState — 获取当前BMS状态
// 用于Event_Process判断当前状态是否允许切换
BMS_State_t BMS_GetState(void) 
 {
		return g_bms_state;
 }


// 事件处理 — 所有状态切换集中在这里
// 由Event_Process(调度器20ms周期)调用
// Task只Push事件，不直接调BMS_SetState
void BMS_EventHandle(Event_t event)
{
    switch(event)
    {

        /*
         * 模块：启动、停止和故障事件处理。
         * 启动和停止只切换软件监测状态，不控制电池电流。
         * 待机或监测期间收到故障事件时，进入故障报警状态。
         */
         // 待机状态收到启动请求后，进入监测状态。
        case EVENT_START:
            if (BMS_GetState() == BMS_STANDBY)
            {
                BMS_SetState(BMS_MONITOR);
            }
            break;

        // 监测状态收到停止请求后，返回待机状态。
        case EVENT_STOP:
            if (BMS_GetState() == BMS_MONITOR)
            {
                BMS_SetState(BMS_STANDBY);
            }
            break;

        // 待机或监测状态收到故障事件后，进入故障状态。
        case EVENT_FAULT:
            if ((BMS_GetState() == BMS_STANDBY) ||
                (BMS_GetState() == BMS_MONITOR))
            {
                BMS_SetState(BMS_FAULT);
            }
            break;
        // ── 故障恢复：FAULT → STANDBY ──
       /*
        * 模块：统一故障管理器人工恢复处理。
        * 作用：PA2只提出清除请求；管理器根据锁存状态和恢复稳定状态
        *       决定哪些故障可以清除。仍有关键故障时继续保持BMS_FAULT。
        */
        case EVENT_RECOVER:
        {
            FaultManagerSnapshot_t fault_snapshot;//保存清除前或清除后的故障管理器结果
            SensorSample_t event_sample;    //保存清除故障时的电压、温度快照
            uint16_t cleared_mask;  //本次按键实际清除了哪些故障
            uint8_t event_sample_available; //是否成功取得传感器快照

            // 非故障状态不接受人工恢复请求。
            if (BMS_GetState() != BMS_FAULT)
            {
                break;
            }

            // 读取清除前的故障管理器快照。
            if (FaultManager_GetSnapshot(&fault_snapshot) == 0U)
            {
                break;
            }

            /*
            * 请求清除全部锁存故障。
            * 管理器内部只会清除已经进入recovery_ready_mask的故障，
            * 尚未恢复的故障不会被按键强制清除。
            */
            cleared_mask =
                FaultManager_RequestManualClear(fault_snapshot.latched_mask);

            // 重新读取清除后的实际故障状态。
            if (FaultManager_GetSnapshot(&fault_snapshot) == 0U)
            {
                break;
            }

            /*
             * 故障清除事件需要保存“清除发生时”的测量值。
             * 这里立即复制最近快照，后续串口或Flash消费时不再读取实时传感器。
             */
            event_sample_available = Sensor_GetLatest(&event_sample);

            if (fault_snapshot.newly_cleared_mask != FAULT_MASK_NONE)
            {
                (void)FaultEvent_Push(
                    FAULT_EVENT_CLEARED,
                    fault_snapshot.newly_cleared_mask,
                    fault_snapshot.active_mask,
                    (event_sample_available != 0U) ? &event_sample : 0,
                    (uint8_t)BMS_GetState(),
                    GetTick());
            }

            // 清除后立即同步完整16位活动故障掩码，供状态显示和0x303发送。
            g_fault_code = fault_snapshot.active_mask;

            /*
            * 本函数只更新g_fault_code，不直接发送CAN。
            *
            * BmsTask从状态机返回后会比较故障掩码是否变化。
            * 如果发生变化，BmsTask通过任务通知唤醒CanTask，
            * 由CanTask立即发送最新的完整故障状态。
            *
            * 这样可以保证CAN硬件始终只有CanTask一个执行者。
            */
            // 有故障真正被清除时输出诊断信息。
            if (cleared_mask != FAULT_MASK_NONE)
            {
                printf("[BMS] Fault cleared: 0x%04X\r\n",
                    (unsigned int)cleared_mask);
            }

            /*
            * 只有所有关键故障都已经清除，系统才允许离开FAULT。
            * 如果只清除了一部分，蜂鸣器和故障状态继续保持。
            */
            if (fault_snapshot.critical_mask == FAULT_MASK_NONE)
            {
                BMS_SetState(BMS_STANDBY);
            }

            break;
        }

    
        default:
            break;
    }
}

/*
 * 模块：状态机与统一故障管理器适配。
 * 作用：提交最新传感器快照和自检结果，读取汇总故障掩码，
 * 并在存在关键故障时向状态机事件队列发送FAULT事件。
 * 边界：本函数不直接开启蜂鸣器，也不直接切换BMS状态。
 * 
 * 从Sensor Service取得数据
            ↓
 *交给Fault Manager判断故障
            ↓
 *     取得判断结果
        ┌───┴─────────┐
        ↓             ↓
 *  保存故障记录      请求状态机进入FAULT
 * FaultEvent_Push     Event_Push
 * (一条完整故障记录)  (一个状态切换命令)
 */
static void BMS_UpdateSensorFaultManager(void)
{
    SensorSample_t sample;//保存本轮传感器数据
    FaultManagerSnapshot_t fault_snapshot;//保存本轮故障管理器判断结果
    uint8_t sample_available;//表示是否取得了传感器快照
    uint32_t now_ms;//保存当前系统时间（毫秒）
    /*
    * 保存上一次已经交给故障管理器处理的Bus-Off次数。
    * static使它在函数退出后仍保留数值。
    */
    static uint32_t last_seen_bus_off_count = 0U;

    uint32_t can_esr;             // 本轮CAN错误状态寄存器快照
    uint8_t can_degraded;         // 当前CAN是否处于通信降级
    uint8_t can_bus_off_event;    // 本轮是否新发生过Bus-Off
    uint8_t can_healthy;          // 当前CAN是否已经恢复健康

    now_ms = GetTick();

    // 获取Sensor Service最近发布的一帧快照。
    sample_available = Sensor_GetLatest(&sample);

    /*
    * 只读取一次ESR，确保本轮所有CAN判断使用同一个时刻的状态。
    */
    can_esr = CAN1->ESR;

    /*
    * EWGF：错误计数进入警告范围。
    * EPVF：控制器进入Error Passive。
    *
    * Bus-Off由独立故障项处理，因此BOFF存在时不重复报告CAN_DEGRADED。
    */
    can_degraded =
        (uint8_t)(
            ((can_esr & (CAN_ESR_EWGF | CAN_ESR_EPVF)) != 0U) &&
            ((can_esr & CAN_ESR_BOFF) == 0U)
        );

    /*
    * g_can_bus_off_count是累计值，不能直接拿“非零”作为触发条件。
    * 否则发生过一次Bus-Off后，触发条件会永久为1。
    *
    * 这里只在累计次数发生变化时产生一次触发脉冲。
    */
    if (g_can_bus_off_count != last_seen_bus_off_count)
    {
        can_bus_off_event = 1U;
        last_seen_bus_off_count = g_can_bus_off_count;
    }
    else
    {
        can_bus_off_event = 0U;
    }

    /*
    * CAN恢复健康必须同时满足：
    * 1. 不在警告、Error Passive或Bus-Off状态；
    * 2. 驱动当前不处于Bus-Off；
    * 3. 最近一次报文已经发送成功。
    */
    can_healthy =
        (uint8_t)(
            ((can_esr &
            (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) == 0U) &&
            (g_can_bus_off_active == 0U) &&
            (g_can_last_send_result == CAN_SEND_OK)
        );


    /*
     * 更新电压、温度、NTC、数据一致性和ADC超时故障。
     * 没有快照时传入空指针，由管理器判断ADC超时。
     */
    FaultManager_UpdateSensor(
        (sample_available != 0U) ? &sample : 0,
        sample_available,
        now_ms);

    /*
    * CAN通信降级：
    * 条件持续500ms才确认，恢复健康1000ms后自动清除。
    */
    FaultManager_UpdateCondition(
        FAULT_ID_CAN_DEGRADED,
        can_degraded,
        can_healthy,
        now_ms);

    /*
    * CAN Bus-Off：
    * 驱动累计次数变化时立即确认；
    * ABOM恢复并连续健康1000ms后，允许人工确认清除。
    */
    FaultManager_UpdateCondition(
        FAULT_ID_CAN_BUS_OFF,
        can_bus_off_event,
        can_healthy,
        now_ms);

    /*
     * 把上电自检结果也纳入同一个管理器。
     * 自检失败后trip保持成立，只能通过重新上电、重新自检来恢复。
     */
    FaultManager_UpdateCondition(
        FAULT_ID_SELF_CHECK,
        g_self_check_failed,
        (uint8_t)(g_self_check_failed == 0U),
        now_ms);

    /*
     * UpdateSensor内部已经完成一次汇总。
     * 加入CAN 和 SELF_CHECK条件后，再统一生成最终故障掩码。
     */
    FaultManager_EndEvaluation();

    // 取得本轮完整故障结果。
    if (FaultManager_GetSnapshot(&fault_snapshot) == 0U)
    {
        return;
    }

    /*
     * 只在故障第一次确认的边沿保存事件。
     * 持续存在的active_mask不会重复写入，避免同一故障每10ms生成一条日志。
     */
    if (fault_snapshot.newly_confirmed_mask != FAULT_MASK_NONE)
    {
        (void)FaultEvent_Push(
            FAULT_EVENT_CONFIRMED,
            fault_snapshot.newly_confirmed_mask,
            fault_snapshot.active_mask,
            (sample_available != 0U) ? &sample : 0,
            (uint8_t)BMS_GetState(),
            now_ms);
    }

    // 每轮同步完整16位活动故障掩码，不再截断高8位。
    g_fault_code = fault_snapshot.active_mask;

    /*
    * 本函数只更新故障状态，不直接发送CAN。
    *
    * 故障状态报文由CanTask统一发送：
    * 1. 正常情况下每100ms周期发送一次；
    * 2. 故障掩码发生变化时，由BmsTask通知CanTask立即发送。
    *
    * 统一执行者可以避免多个任务同时访问CAN驱动，
    * 也可以避免多个任务同时修改故障报文的Alive Counter。
    */

    /*
     * 只要关键故障持续存在且尚未进入FAULT，就重复提出FAULT事件。
     * 这样即使某一轮事件队列暂时满了，下一轮仍会再次尝试。
     * EVENT_FAULT是幂等事件，多收到一次不会造成错误状态切换。
     */
    if ((fault_snapshot.critical_mask != FAULT_MASK_NONE) &&
        (BMS_GetState() != BMS_FAULT))
    {
        Event_Push(EVENT_FAULT);
    }
}


/*
 * Task_Init — 执行BMS状态机的一次性软件初始化。
 *
 * 本函数只输出启动信息并进入SELF_CHECK，
 * 不使用空循环延时，避免高优先级BmsTask长时间占用CPU。
 *
 * 系统运行状态由后续LED心跳和串口启动信息表示。
 */
static void Task_Init(void)
{
    printf("[BMS] INIT - Initializing...\r\n");

    /*
     * INIT没有外部事件来源，
     * 完成本阶段初始化后直接进入SELF_CHECK。
     */
    BMS_SetState(BMS_SELF_CHECK);
}



// Task_SelfCheck — 自检任务
// 首次进入时记录时间戳，延时100ms等待ADC/DMA稳定
// 100ms后检查：
//   1. ADC+DMA是否正常工作（检查半/全传输中断标志）
//   2. CAN总线是否正常（检查ESR错误状态寄存器）
// 通过→STANDBY，失败→FAULT
static void Task_SelfCheck(void)
{
    uint32_t now_ms = GetTick();
    SensorSample_t sample;
    uint32_t can_msr;   // CAN当前工作模式
    uint32_t can_esr;   // CAN当前错误状态
    uint8_t fail = 0U;  // 本轮自检总结果

    // 自检启动时间戳（static，只初始化一次）
    static uint32_t selfcheck_start = 0;

    // ── 第一阶段：记录启动时间，等待ADC/DMA稳定 ──
    if(selfcheck_start == 0)
    {
        selfcheck_start = GetTick();  // 记录当前时间
        return;                       // 第一次不检测，等下次进入
    }

    // ── 第二阶段：等待100ms，让DMA至少完成一次半传输 ──
    if(GetTick() - selfcheck_start < 100)
    {
        return;  // 还没到100ms，继续等
    }

    // ── 第三阶段：100ms已到，开始检测 ──
    // 检查ADC+DMA：任一中断标志被置位过说明工作正常
    // 必须已经发布过快照，而且最后一帧不能超过100ms。
    if ((Sensor_GetLatest(&sample) == 0U) ||
        (Sensor_IsFresh(&sample, now_ms, 100U) == 0U))
    {
        printf("[BMS] SENSOR SNAPSHOT FAIL\r\n");
        fail = 1U;
    }


    /*
    * CAN上电自检只检查控制器是否处于可工作状态：
    *
    * INAK=1：控制器仍停留在初始化模式；
    * SLAK=1：控制器仍处于睡眠模式；
    * ABOM=0：没有开启Bus-Off自动恢复；
    * BOFF=1：控制器当前已经Bus-Off。
    *
    * LEC只是最近一次错误类型，不能代表当前CAN是否可工作，
    * 因此这里不再使用LEC作为上电自检失败条件。
    */
    can_msr = CAN1->MSR;
    can_esr = CAN1->ESR;

    if (((can_msr & (CAN_MSR_INAK | CAN_MSR_SLAK)) != 0U) ||
        ((CAN1->MCR & CAN_MCR_ABOM) == 0U) ||
        ((can_esr & CAN_ESR_BOFF) != 0U))
    {
        printf("[BMS] CAN SELF-CHECK FAIL MCR=0x%08X MSR=0x%08X ESR=0x%08X\r\n",
            CAN1->MCR,
            can_msr,
            can_esr);

        fail = 1U;
    }
    // 复位启动时间戳，为下次进入SELF_CHECK做准备
    selfcheck_start = 0;

    // ── 判断结果 ──
    // 根据本轮自检结果决定后续状态。
    if (fail != 0U)
    {
        /*
        * 自检失败属于上电完整性故障。
        * 标志保持到系统复位，避免按恢复键绕过失败的上电自检。
        */
        g_self_check_failed = 1U;
        g_fault_code = BMS_FAULT_SELF_CHECK;
        BMS_SetState(BMS_FAULT);
    }
    else
    {
        // 自检通过，明确清除自检失败条件。
        g_self_check_failed = 0U;

        printf("[BMS] SELF_CHECK PASS\r\n");
        BMS_SetState(BMS_STANDBY);
    }
}


/*
 * 模块：待机状态处理。
 * 当前作用：检测PB6启动按键的按下沿，发送EVENT_START。
 * 启动后进入MONITOR，不根据SOC启动充电，也不控制电池通路。
 * 待机期间的基础采样与故障监测将在后续接入。
 */
static void Task_Standby(void)
{
// 保存上一次启动按键状态，用于检测按下沿。
    static uint8_t last_start_pressed = 0U;

    // 读取当前PB6启动按键状态。
    uint8_t start_pressed = KEY_Read(KEY_START);

    // 判断启动按键是否刚刚从松开变成按下。
    if((start_pressed == 1U) && (last_start_pressed == 0U))
    {
        // 通过串口输出启动按键事件。
        printf("[BMS] START key pressed\r\n");

        // 向事件队列发送BMS启动事件。
        Event_Push(EVENT_START);
    }

    // 保存本次按键状态，供下一轮检测按下沿。
    last_start_pressed = start_pressed;
				
}


/*
 * 模块：监测状态业务。
 * 传感器快照和故障评估已由BMS_StateMachine公共流程处理；
 * 0x301、0x302和0x303由CanTask发送，因此这里不重复发送CAN。
 */
static void Task_Monitor(void)
{
    /* 当前监测状态没有额外的执行器动作。 */
}


/*
 * 根据BMS状态统一更新PC13状态指示灯。
 *
 * 非故障状态：每500ms翻转一次，表示系统正在运行；
 * 故障状态：每200ms翻转一次，表示系统处于报警状态。
 *
 * 只区分“故障”和“非故障”两种显示模式。
 * 模式发生变化时先关闭LED并重置计时基准，
 * 防止状态切换沿用旧模式的剩余时间。
 *
 * 本函数只由BmsTask执行，使PC13只有一个任务控制。
 */
static void BMS_UpdateLed(void)
{
    static uint32_t last_toggle_ms = 0U;
    static uint8_t last_fault_mode = 0U;

    uint32_t now_ms = GetTick();
    uint8_t fault_mode;
    uint32_t toggle_period_ms;

    fault_mode =
        (g_bms_state == BMS_FAULT) ? 1U : 0U;

    toggle_period_ms =
        (fault_mode != 0U) ? 200U : 500U;

    if (fault_mode != last_fault_mode)
    {
        last_fault_mode = fault_mode;
        last_toggle_ms = now_ms;
        LED_OFF();
        return;
    }

    if ((now_ms - last_toggle_ms) >= toggle_period_ms)
    {
        last_toggle_ms = now_ms;
        LED_Toggle();
    }
}







// Task_Fault — 检测PA2按下并提出故障恢复请求。
// 故障LED和蜂鸣器由统一的状态指示逻辑处理。
static void Task_Fault(void)
{
   /*
    * 检测PA2恢复按键的按下沿。
    * 使用边沿而不是持续电平，避免按住按键时每10ms重复塞入恢复事件。
    */
    {
        static uint8_t last_reset_pressed = 0U;
        uint8_t reset_pressed = KEY_Read(KEY_FAULT_RESET);

        if ((reset_pressed != 0U) &&
            (last_reset_pressed == 0U))
        {
            printf("[BMS] Manual recovery requested\r\n");
            Event_Push(EVENT_RECOVER);
        }

        last_reset_pressed = reset_pressed;
    }
}

/*
 * 模块：状态机周期调度入口。
 * 作用：根据当前状态执行对应处理函数。
 * 由BmsTask每10ms调用一次，本函数不会创建FreeRTOS任务。
 * 本阶段不包含充电或放电控制任务。
 */
void BMS_StateMachine(void)
{
    /*
    * 自检完成后持续评估故障。
    * FAULT状态也必须继续评估，否则无法累计恢复稳定时间。
    */
    if ((g_bms_state == BMS_STANDBY) ||
        (g_bms_state == BMS_MONITOR) ||
        (g_bms_state == BMS_FAULT))
    {
        BMS_UpdateSensorFaultManager();
    }
    // 根据当前状态选择本轮处理函数。
    switch (g_bms_state)
    {
        // 执行上电初始化。
        case BMS_INIT:
            Task_Init();
            break;

        // 执行系统自检。
        case BMS_SELF_CHECK:
            Task_SelfCheck();
            break;

        // 等待启动监测请求。
        case BMS_STANDBY:
            Task_Standby();
            break;

        // 执行电池监测业务。
        case BMS_MONITOR:
            Task_Monitor();
            break;

        // 执行故障报警和恢复请求处理。
        case BMS_FAULT:
            Task_Fault();
            break;

        // 暂时沿用原有异常状态处理方式：返回初始化状态。
        default:
            BMS_SetState(BMS_INIT);
            break;
    }

    /*
     * 状态业务处理结束后统一更新本地LED状态。
     * GPIO操作耗时很短，不会阻塞BMS任务。
     */
    BMS_UpdateLed();
}






