/*
 * 文件名称：app_can_service.c
 *
 * 模块名称：CAN应用发送服务
 *
 * 模块职责：
 * 1. 读取Sensor Service发布的最新电压和温度快照；
 * 2. 读取BMS当前监测状态和活动故障状态；
 * 3. 把业务数据交给CAN Protocol组装成标准报文；
 * 4. 管理三类报文各自的Alive Counter；
 * 5. 调用CAN BSP把组装完成的报文发送到硬件。
 *
 * 当前数据流：
 * Sensor快照 / BMS状态
 *          ↓
 * AppCAN_Send...()
 *          ↓
 * CAN Protocol打包并计算CRC8
 *          ↓
 * CAN1_Send()
 *          ↓
 * CAN控制器发送
 *
 * 当前并发约束：
 * 只有CanTask允许调用本模块的CAN发送接口。
 *
 * BmsTask检测到故障状态变化时，只向CanTask发送任务通知，
 * 不直接调用CAN驱动，从而保证CAN硬件只有一个执行者。
 * 
 * 模块边界：
 * 本模块不判断过压、欠压和过温，不修改BMS状态，
 * 不写Flash，也不负责ECU-B接收与显示。
 */
#include "app_can_service.h"

#include "app_sensor.h"
#include "app_bms_state.h"

#include "bsp_can.h"
#include "bsp_tim.h"
#include "can_protocol.h"


/*
 * 电压状态报文下一帧使用的Alive Counter。
 * 每组装一帧后在0～15之间循环。
 */
volatile uint8_t g_can_301_alive_counter = 0U;

/*
 * 温度状态报文使用自己的Alive Counter，
 * 不受电压报文发送周期影响。
 */
volatile uint8_t g_can_302_alive_counter = 0U;

/*
 * 故障状态报文使用自己的Alive Counter，
 * 周期发送和后续故障快速发送都将共用它。
 */
volatile uint8_t g_can_303_alive_counter = 0U;


/*
 * 根据单体电池电压估算用于CAN显示的SOC。
 *
 * 当前采用简化线性估算：
 * 接近满电电压时显示100%，接近最低电压时显示0%。
 *
 * 该结果只用于当前演示系统的状态显示，
 * 不等同于真实BMS使用的库仑积分或模型SOC。
 */
static uint8_t AppCAN_EstimateSoc(float cell_voltage_v)
{
    float soc;

    if (cell_voltage_v >= 4.15f)
    {
        return 100U;
    }

    if (cell_voltage_v <= 3.05f)
    {
        return 0U;
    }

    soc =
        (cell_voltage_v - 3.0f) /
        1.2f *
        100.0f;

    if (soc > 100.0f)
    {
        soc = 100.0f;
    }

    if (soc < 0.0f)
    {
        soc = 0.0f;
    }

    return (uint8_t)(soc + 0.5f);
}

/*
 * 获取一帧存在且年龄小于100ms的Sensor快照。
 *
 * 本函数只判断快照是否存在、是否足够新鲜；
 * 电压和温度字段是否有效，由各发送函数分别判断。
 */
static uint8_t AppCAN_GetFreshSensorSample(
    SensorSample_t *sample)
{
    if ((sample == 0) ||
        (Sensor_GetLatest(sample) == 0U))
    {
        return 0U;
    }

    return Sensor_IsFresh(
        sample,
        GetTick(),
        100U);
}

/*
 * 使用最新有效快照发送0x301。
 * 无论处于待机、监测还是故障状态，电压采样都继续通过CAN更新。
 * Byte4低四位携带Alive Counter，每组装一帧后按0~15循环递增。
 */
void AppCAN_SendVoltageStatus(void)
{
    CAN_Frame_t tx;
    SensorSample_t sample;
    uint16_t cell_mv;
    uint8_t soc;
    uint8_t monitor_status;

    if ((AppCAN_GetFreshSensorSample(&sample) == 0U) ||
        (sample.voltage_valid == 0U))
    {
        return;
    }

    cell_mv = (uint16_t)(sample.cell_voltage_v * 1000.0f + 0.5f);
    soc = AppCAN_EstimateSoc(
        sample.cell_voltage_v);
    monitor_status = ((BMS_GetState() == BMS_MONITOR) ||
                      (BMS_GetState() == BMS_FAULT)) ?
                     CAN_STATUS_MONITOR : CHG_STATE_IDLE;

    BMS_PackVoltage(&tx,
                    cell_mv,
                    soc,
                    monitor_status,
                    g_can_301_alive_counter);

    // 只保留低四位：15加1后通过掩码自然回到0。
    g_can_301_alive_counter =
        (uint8_t)((g_can_301_alive_counter + 1U) & CAN_301_ALIVE_MASK);

    (void)CAN1_Send(&tx);
}


/*
 * AppCAN_SendTemperatureStatus — 发送当前有效温度状态。
 *
 * 温度快照无效时不发送伪造值；
 * NTC故障仍由故障状态报文上报。
 */
void AppCAN_SendTemperatureStatus(void)
{
    CAN_Frame_t tx;
    SensorSample_t sample;
    uint8_t temperature;

    if ((AppCAN_GetFreshSensorSample(&sample) == 0U) ||
        (sample.temperature_valid == 0U))
    {
        return;
    }

    temperature =
    (uint8_t)(sample.temperature_c + 0.5f);

    /*
    * 使用温度报文自己的计数器组装报文。
    * BMS_PackTemp内部会同时计算CRC8。
    */
    BMS_PackTemp(
        &tx,
        temperature,
        g_can_302_alive_counter);

    /*
    * 每组装一帧温度报文后计数器加1。
    * 15加1后通过低四位掩码回到0。
    */
    g_can_302_alive_counter =
        (uint8_t)(
            (g_can_302_alive_counter + 1U) &
            CAN_302_ALIVE_MASK);

    (void)CAN1_Send(&tx);
}


/*
 * AppCAN_SendFaultStatus — 发送当前活动故障状态。
 *
 * 当前只由CanTask调用：
 * 1. 正常情况下每100ms周期发送一次；
 * 2. 故障掩码变化时，由BmsTask通知CanTask立即发送。
 *
 * 统一由CanTask发送，可以避免多个任务同时访问CAN驱动，
 * 也可以避免故障报文Alive Counter被并发修改。
 */
void AppCAN_SendFaultStatus(void)
{
    CAN_Frame_t tx;

    /*
     * 把活动故障掩码、Alive Counter和CRC8
     * 一起组装成完整故障报文。
     */
    BMS_PackFault(
        &tx,
        g_fault_code,
        g_can_303_alive_counter);

    /*
     * 每组装一帧后序号加1。
     * 15加1后通过低四位掩码回到0。
     */
    g_can_303_alive_counter =
        (uint8_t)(
            (g_can_303_alive_counter + 1U) &
            CAN_303_ALIVE_MASK);

    (void)CAN1_Send(&tx);
}
