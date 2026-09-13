// app_bms_state.c — BMS状态机实现
// 功能：6状态状态机 + 事件处理 + 状态任务函数
// 设计原则：
//   1. Task只检测条件，不直接改状态
//   2. 状态切换统一在 BMS_EventHandle 中完成
//   3. 通过 Event_Push/Event_Process 解耦检测和切换

#include "app_bms_state.h"
#include "app_event.h"
#include "bsp_gpio.h"
#include "bsp_adc_dma.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_usart.h"
#include "app_flash_log.h"

// ─── 状态机全局变量 ───
static BMS_State_t g_bms_state = BMS_POWER_OFF; // 当前BMS状态
uint8_t g_fault_code = 0;                       // 当前故障码
static uint8_t g_last_fault = 0;                // 上次故障码（去重用）

// ADC_To_CellVolt — ADC原始值 → Cell电压(V)
// 把电位器0~3.3V映射到3.0~4.2V
// 参数 adc：ADC原始值(0~4095)
// 返回：电压值(3.0~4.2V)
// 公式：adc/4095*1.2 + 3.0
static float ADC_To_CellVolt(uint16_t adc)
{
    float v = (float)adc * 3.3f / 4095.0f;
    return v / 3.3f * 1.2f + 3.0f;
}

// ADC_To_Temp — ADC原始值 → 温度(℃)
// 0V = -20℃, 3.3V = 70℃
// 参数 adc：ADC原始值(0~4095)
// 返回：温度值(-20~70℃)
static float ADC_To_Temp(uint16_t adc)
{
    float v = (float)adc * 3.3f / 4095.0f;
    return v / 3.3f * 90.0f - 20.0f;
}

// SOC_Simple — 从Cell电压计算SOC（简化线性法）
// 4.2V → 100%, 3.0V → 0%
// 边界校正：>=4.15V固定100%, <=3.05V固定0%
// 参数 v_cell：Cell电压(V)
// 返回：SOC(0~100)
static uint8_t SOC_Simple(float v_cell)
{
    if (v_cell >= 4.15f)
        return 100;
    if (v_cell <= 3.05f)
        return 0;
    float soc = (v_cell - 3.0f) / 1.2f * 100.0f;
    if (soc > 100)
        soc = 100;
    if (soc < 0)
        soc = 0;
    return (uint8_t)(soc + 0.5f);
}

// CheckFault — 故障检测
// 检测三个故障条件，返回故障码（位掩码）
// Bit0=过压, Bit1=欠压, Bit2=过温
// 参数 v_cell：Cell电压(V)
// 参数 temp_c：温度(℃)
// 返回：故障码（0=无故障）
static uint8_t CheckFault(float v_cell, float temp_c)
{
    uint8_t fault = 0;
    if (v_cell > 4.2f)
        fault |= 0x01; // 过压
    if (v_cell < 3.0f)
        fault |= 0x02; // 欠压
    if (temp_c > 60.0f)
        fault |= 0x04; // 过温
    return fault;
}

//====================================状态接口=====================================

// BMS_SetState — 设置BMS状态（直接修改状态变量）
// 由BMS_EventHandle调用，Task不直接调用（除POWER_OFF/SELF_CHECK外）
void BMS_SetState(BMS_State_t state) { g_bms_state = state; }

// BMS_GetState — 获取当前BMS状态
// 用于Event_Process判断当前状态是否允许切换
BMS_State_t BMS_GetState(void) { return g_bms_state; }

// 事件处理 — 所有状态切换集中在这里
// 由Event_Process(调度器20ms周期)调用
// Task只Push事件，不直接调BMS_SetState
void BMS_EventHandle(Event_t event)
{
    switch (event)
    {
    // ── 启动BMS：STANDBY → DISCHARGE ──
    case EVENT_START:                      // 启动BMS--PB0
        if (BMS_GetState() == BMS_STANDBY) // 待机
            BMS_SetState(BMS_DISCHARGE);   // 放电
        break;

    // ── 停止BMS：DISCHARGE → STANDBY ──
    case EVENT_STOP:                         // 停止
        if (BMS_GetState() == BMS_DISCHARGE) // 放电
            BMS_SetState(BMS_STANDBY);       // 待机
        break;

    // ── 故障发生：DISCHARGE/CHARGE → FAULT ──
    case EVENT_FAULT: // 发送故障
        if (BMS_GetState() == BMS_DISCHARGE || BMS_GetState() == BMS_CHARGE)
            BMS_SetState(BMS_FAULT); // 故障保护
        break;

    // ── 故障恢复：FAULT → STANDBY ──
    case EVENT_RECOVER:
    {
        if (BMS_GetState() == BMS_FAULT) // 故障保护
        {
            // 重新检测故障是否真的消失
            uint16_t adc0, adc1;
            if (!ADC_ReadSafe(&adc0, &adc1))
                break;
            float v = ADC_To_CellVolt(adc0);
            float t = ADC_To_Temp(adc1);
            if (CheckFault(v, t) == 0) // 故障已消失
            {
                g_fault_code = 0;
                BMS_SetState(BMS_STANDBY); // 待机
            }
            // else: 故障仍在，不恢复
        }
        break;
    }

    // ── 充满电：CHARGE → STANDBY ──
    case EVENT_CHARGE_DONE:
        if (BMS_GetState() == BMS_CHARGE) // 充电
        {
            printf("[BMS] Charge complete!\n");
            BMS_SetState(BMS_STANDBY); // 待机
        }
        break;

    default:
        break;
    }
}

// Task_PowerOff — 上电初始化
// LED闪烁2次表示启动
// 完成后直接切到SELF_CHECK（没有事件源触发首次启动）
static void Task_PowerOff(void)
{
    printf("[BMS] POWER_OFF - Initializing...\r\n");
    LED_ON();
    for (volatile uint32_t i = 0; i < 500000; i++)
        ;
    LED_OFF();
    for (volatile uint32_t i = 0; i < 500000; i++)
        ;
    LED_ON();
    for (volatile uint32_t i = 0; i < 500000; i++)
        ;
    LED_OFF();
    // 直接设状态（没有事件源触发首次启动）
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
    // 自检启动时间戳（static，只初始化一次）
    static uint32_t selfcheck_start = 0;

    // ── 第一阶段：记录启动时间，等待ADC/DMA稳定 ──
    if (selfcheck_start == 0)
    {
        selfcheck_start = GetTick(); // 记录当前时间
        return;                      // 第一次不检测，等下次进入
    }

    // ── 第二阶段：等待100ms，让DMA至少完成一次半传输 ──
    if (GetTick() - selfcheck_start < 100)
    {
        return; // 还没到100ms，继续等
    }

    // ── 第三阶段：100ms已到，开始检测 ──
    uint8_t fail = 0;

    // 检查ADC+DMA：任一中断标志被置位过说明工作正常
    if (!g_adc_half_ready && !g_adc_full_ready)
    {
        printf("[BMS] ADC/DMA FAIL\r\n");
        fail = 1;
    }

    // 检查CAN：读取ESR寄存器的LEC错误码
    // LEC=0表示无错误
    uint8_t lec = (CAN1->ESR >> 4) & 0x07;
    if (lec != 0)
    {
        printf("[BMS] CAN FAIL LEC=%d\r\n", lec);
        fail = 1;
    }

    // 复位启动时间戳，为下次进入SELF_CHECK做准备
    selfcheck_start = 0;

    // ── 判断结果 ──
    if (fail)
    {
        g_fault_code = 0x80; // 自检失败故障码
        BMS_SetState(BMS_FAULT);
    }
    else
    {
        printf("[BMS] SELF_CHECK PASS\r\n");
        BMS_SetState(BMS_STANDBY);
    }
}

// Task_Standby：此历史基线首次进入待机时自动提交启动事件。
static void Task_Standby(void)
{

    static uint8_t once = 0;

    if (!once)
    {
        once = 1;
        Event_Push(EVENT_START);
    }
}

// Task_Discharge — 放电/运行状态（核心业务）
// 每10ms执行一次：
//   1. ADC_ReadSafe读取最新电压和温度
//   2. 换算物理量，计算SOC
//   3. CheckFault检测故障
//   4. 有故障→FlashLog记录→Push EVENT_FAULT→return
//   5. 无故障→CAN1_Send(0x301电压+SOC) + CAN1_Send(0x302温度)
static void Task_Discharge(void)
{
    CAN_Frame_t tx;
    uint16_t adc0, adc1;

    // 1. 读取最新ADC值（双缓冲安全读取）
    if (!ADC_ReadSafe(&adc0, &adc1))
        return;

    // 2. 换算物理量
    float v_cell = ADC_To_CellVolt(adc0);
    float temp_c = ADC_To_Temp(adc1);
    uint8_t soc = SOC_Simple(v_cell);
    uint8_t fault = CheckFault(v_cell, temp_c);

    // 3. 故障处理
    if (fault)
    {
        g_fault_code = fault;
        // 去重：只有新故障才写Flash日志
        if (fault != g_last_fault)
        {
            g_last_fault = fault;
            uint16_t mv = (uint16_t)(v_cell * 100);
            FlashLog_Write(fault, mv, (uint8_t)(temp_c + 0.5f), soc);
        }
        printf("[BMS] FAULT! Code=0x%02X\r\n", fault);
        Event_Push(EVENT_FAULT);
        return;
    }
    g_last_fault = 0;

    // 4. 发送CAN 0x301（电压+SOC+放电状态）
    uint16_t mv = (uint16_t)(v_cell * 100);
    tx.id = 0x301;
    tx.dlc = 8;
    tx.data[0] = (mv >> 8) & 0xFF;
    tx.data[1] = mv & 0xFF;
    tx.data[2] = soc;
    tx.data[3] = CHG_STATE_DISCHARGE;
    for (int i = 4; i < 8; i++)
        tx.data[i] = 0;
    (void)CAN1_Send(&tx);

    // 5. 发送CAN 0x302（温度）
    tx.id = 0x302;
    tx.dlc = 8;
    tx.data[0] = (uint8_t)(temp_c + 0.5f);
    for (int i = 1; i < 8; i++)
        tx.data[i] = 0;

    (void)CAN1_Send(&tx);
}

// Task_Fault — 故障保护状态
// LED 200ms闪烁
// 每500ms发送一次0x303故障码
// 检测PB1按下→Push EVENT_RECOVER请求恢复
static void Task_Fault(void)
{
    CAN_Frame_t tx;
    uint32_t now = GetTick();

    // LED快速闪烁
    static uint32_t last_blink = 0;
    if (now - last_blink > 200)
    {
        last_blink = now;
        LED_Toggle();
    }

    // 定期发送故障码
    static uint32_t last_send = 0;
    if (now - last_send > 500)
    {
        last_send = now;
        BMS_PackFault(&tx, g_fault_code);
        CAN1_Send(&tx);
    }

    // 检测恢复按键
    if (KEY_Read(1))
    {
        printf("[BMS] Attempt recovery...\r\n");
        Event_Push(EVENT_RECOVER);
    }
}

// Task_Charge — 充电状态
// 模拟充电过程：charge_soc从50%开始每200ms+1%
// 充到≥95%→Push EVENT_CHARGE_DONE
// 电压和温度用ADC实际值，SOC用模拟充电值
static void Task_Charge(void)
{
    CAN_Frame_t tx;

    static uint8_t charge_soc = 0;
    if (charge_soc == 0)
        charge_soc = 50;

    // 模拟电量上升
    if (charge_soc < 95)
        charge_soc++;

    // 读取ADC实时值
    uint16_t adc0, adc1;
    ADC_ReadSafe(&adc0, &adc1);
    float v_cell = ADC_To_CellVolt(adc0);
    float temp_c = ADC_To_Temp(adc1);
    uint8_t soc = charge_soc; // SOC用充电模拟值

    // 发送0x301（电压+充电SOC+充电状态）
    uint16_t mv = (uint16_t)(v_cell * 100);
    tx.id = 0x301;
    tx.dlc = 8;
    tx.data[0] = (mv >> 8) & 0xFF;
    tx.data[1] = mv & 0xFF;
    tx.data[2] = soc;
    tx.data[3] = CHG_STATE_CHARGE;
    for (int i = 4; i < 8; i++)
        tx.data[i] = 0;
    CAN1_Send(&tx);

    // 发送0x302（温度）
    tx.id = 0x302;
    tx.dlc = 8;
    tx.data[0] = (uint8_t)(temp_c + 0.5f);
    for (int i = 1; i < 8; i++)
        tx.data[i] = 0;
    CAN1_Send(&tx);

    // 充满自动停止
    if (charge_soc >= 95)
    {
        printf("[BMS] Charge complete!\r\n");
        charge_soc = 0;
        LED_OFF();
        Event_Push(EVENT_CHARGE_DONE);
    }
}

// 状态机入口
// 由调度器在10ms/50ms/100ms/200ms周期中调用
// 根据当前g_bms_state，执行对应的Task函数
void BMS_StateMachine(void)
{
    switch (g_bms_state)
    {
    case BMS_POWER_OFF:
        Task_PowerOff();
        break;

    case BMS_SELF_CHECK:
        Task_SelfCheck();
        break;

    case BMS_STANDBY:
        Task_Standby();
        break;

    case BMS_DISCHARGE:
        Task_Discharge();
        break;

    case BMS_FAULT:
        Task_Fault();
        break;

    case BMS_CHARGE:
        Task_Charge();
        break;

    default:
        BMS_SetState(BMS_POWER_OFF);
        break;
    }
}
