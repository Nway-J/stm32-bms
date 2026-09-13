/*
 * 模块：统一故障生命周期管理器。
 * 作用：让每种故障独立经历CLEAR、PENDING、CONFIRMED和RECOVERY_PENDING，
 *       并生成活动、锁存、关键、恢复就绪以及新确认/新清除掩码。
 * 机制：确认时间用于过滤瞬态，恢复时间用于过滤恢复抖动，人工清除不能
 *       绕过恢复条件。全部计时使用无符号时间差，兼容32位Tick回绕。
 * 边界：本模块只管理故障事实，不执行蜂鸣器、LED、CAN、Flash或状态切换。
 * 测试：CLI可通过受限注入掩码强制传感器类故障条件成立；
 *       注入不修改ADC和Sensor快照，复位后自动撤销。
 *
 *
 TIM3每1ms触发ADC
        ↓
ADC + DMA产生双通道数据
        ↓
Sensor Service发布最新快照
        ↓
BMS状态机每10ms运行
        ↓
BMS_UpdateSensorFaultManager()
        ↓
FaultManager_UpdateSensor()
        ↓
生成各故障的触发条件和恢复条件
        ↓
FaultManager_UpdateCondition()
        ↓
更新每种故障的生命周期
        ↓
FaultManager_EndEvaluation()
        ↓
生成active、latched、critical等汇总掩码
 */

#include "app_fault_manager.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * FaultConfig_t保存的是某种故障的固定规则，不是运行过程中不断变化的数据
    | 字段         | 过压配置示例 |
    | `mask`       | `0x0001`    |
    | `confirm_ms` | `500ms`     |
    | `recovery_confirm_ms`  |  `1000ms` |
    | `level`      | `CRITICAL` |
    | `manual_clear`  |  `1` |
 */
typedef struct
{
    // 该故障在16位总故障码中占用的位。
    uint16_t mask;
    // 触发条件必须连续成立多久才正式确认，单位ms；0表示立即确认。
    uint16_t confirm_ms;
    // 恢复条件必须连续成立多久才允许清除，单位ms。
    uint16_t recovery_confirm_ms;
    // WARNING只上报，CRITICAL要求状态机进入故障状态。
    FaultLevel_t level;
    // 1表示恢复稳定后仍需人工请求清除；0表示允许自动清除。
    uint8_t manual_clear;
} FaultConfig_t;

/*
 * 当前启用的传感器故障配置。
 * 数值是项目演示标定，不代表量产电池安全标定。
 */
static const FaultConfig_t g_fault_config[FAULT_ID_COUNT] =
{
    // mask                确认时间 恢复时间 等级                人工清除
    {FAULT_MASK_OV,          500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_UV,          500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_OT,          500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_NTC_OPEN,    500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_NTC_SHORT,   500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_SENSOR_DATA, 100U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_ADC_TIMEOUT,   0U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_SELF_CHECK,    0U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    {FAULT_MASK_CAN_DEGRADED,500U, 1000U, FAULT_LEVEL_WARNING,  0U},
    {FAULT_MASK_CAN_BUS_OFF,   0U, 1000U, FAULT_LEVEL_CRITICAL, 1U}
};

/*
 * 管理器由本模块独占。
 * static防止其他模块绕过接口直接修改状态；Keil仍可在Symbols窗口中查看。
 */
static FaultManagerSnapshot_t g_fault_manager;

/*
 * 非0位表示测试时强制对应故障的触发条件成立。
 * 非static便于Keil Watch确认当前是否仍开启注入。
 */
volatile uint16_t g_fault_inject_mask = FAULT_MASK_NONE;

// 将一个故障项恢复为CLEAR；保留occurrence_count，便于统计本次上电确认次数。
static void FaultItem_SetClear(FaultItem_t *item)
{
    item->state = FAULT_STATE_CLEAR;
    item->state_since_ms = 0U;
    item->pending_ms = 0U;
    item->recovery_ms = 0U;
    item->recovery_ready = 0U;
}

// 把一个故障从PENDING或立即触发状态转为正式确认，并产生一次“新确认”事件。
static void FaultItem_Confirm(FaultId_t id,
                              FaultItem_t *item,
                              uint32_t now_ms)
{
    item->state = FAULT_STATE_CONFIRMED;
    item->state_since_ms = now_ms;
    item->pending_ms = g_fault_config[id].confirm_ms;
    item->recovery_ms = 0U;
    item->recovery_ready = 0U;

    // 使用饱和计数，防止长期运行后从0xFFFF回绕到0。
    if (item->occurrence_count < 0xFFFFU)
    {
        item->occurrence_count++;
    }

    g_fault_manager.newly_confirmed_mask |= g_fault_config[id].mask;
}

// 清除一个已经确认的故障，并产生一次“新清除”事件。
static void FaultItem_ClearConfirmed(FaultId_t id,
                                     FaultItem_t *item)
{
    g_fault_manager.newly_cleared_mask |= g_fault_config[id].mask;
    FaultItem_SetClear(item);
}


/// 上电调用一次：清空全部故障状态、计时、计数和汇总掩码。
void FaultManager_Init(void)
{
    uint8_t i;

    // 每个故障项独立清零，避免不同故障共享计时状态。
    for (i = 0U; i < (uint8_t)FAULT_ID_COUNT; i++)
    {
        g_fault_manager.items[i].occurrence_count = 0U;
        FaultItem_SetClear(&g_fault_manager.items[i]);
    }

    g_fault_manager.active_mask = FAULT_MASK_NONE;
    g_fault_manager.latched_mask = FAULT_MASK_NONE;
    g_fault_manager.critical_mask = FAULT_MASK_NONE;
    g_fault_manager.recovery_ready_mask = FAULT_MASK_NONE;
    g_fault_manager.newly_confirmed_mask = FAULT_MASK_NONE;
    g_fault_manager.newly_cleared_mask = FAULT_MASK_NONE;
    g_fault_inject_mask = FAULT_MASK_NONE;
}

void FaultManager_BeginEvaluation(void)
{
    /*
     * 这两个掩码只表示“本轮新发生的边沿事件”，
     * 不能像active_mask一样跨周期保留。
     */
    g_fault_manager.newly_confirmed_mask = FAULT_MASK_NONE;
    g_fault_manager.newly_cleared_mask = FAULT_MASK_NONE;
}


/*
 * 故障管理器的核心。它解决的问题是：
 * 给它“某种故障当前是否触发、是否达到恢复条件、当前时间”，
 * 它负责让该故障在确认、保持和恢复状态之间切换。
 */
void FaultManager_UpdateCondition(FaultId_t id,
                                  uint8_t trip_condition,   //表示当前是否满足触发条件：1：当前异常；0：当前不满足触发条件
                                  uint8_t recovery_condition, //表示当前是否满足恢复条件
                                  uint32_t now_ms)
{
    FaultItem_t *item;
    const FaultConfig_t *config;
    uint32_t elapsed_ms;

    // 防止非法ID造成items[]和配置表越界。
    if ((uint32_t)id >= (uint32_t)FAULT_ID_COUNT)
    {
        return;
    }

    item = &g_fault_manager.items[id];
    config = &g_fault_config[id];

    /*
     * 注入命中当前故障时，强制触发条件为1、恢复条件为0。
     * 后面的生命周期仍照常执行确认时间、锁存和恢复流程。
     */
    if ((g_fault_inject_mask & config->mask) != 0U)
    {
        trip_condition = 1U;
        recovery_condition = 0U;
    }

    // 将调用者传入的任意非零值统一规范成1，后面只处理明确的0/1。
    trip_condition = (trip_condition != 0U) ? 1U : 0U;
    recovery_condition = (recovery_condition != 0U) ? 1U : 0U;

    switch (item->state)
    {
        case FAULT_STATE_CLEAR:
            item->pending_ms = 0U;
            item->recovery_ms = 0U;
            item->recovery_ready = 0U;

            // 第一次发现异常：需要确认时间则进入PENDING，否则立即确认。
            if (trip_condition != 0U)
            {
                if (config->confirm_ms == 0U)
                {
                    FaultItem_Confirm(id, item, now_ms);
                }
                else
                {
                    item->state = FAULT_STATE_PENDING;
                    item->state_since_ms = now_ms;
                }
            }
            break;

        case FAULT_STATE_PENDING:
            if (trip_condition == 0U)
            {
                /*
                 * 确认时间内条件消失，说明只是瞬态，
                 * 不计为一次已确认故障。
                 */
                FaultItem_SetClear(item);
            }
            else
            {
                /*
                 * 使用无符号减法计算持续时间。
                 * 即使32位Tick发生回绕，短时间间隔仍能得到正确差值。
                 */
                elapsed_ms = (uint32_t)(now_ms - item->state_since_ms);
                item->pending_ms = elapsed_ms;

                if (elapsed_ms >= config->confirm_ms)
                {
                    FaultItem_Confirm(id, item, now_ms);
                }
            }
            break;

        case FAULT_STATE_CONFIRMED:
            item->pending_ms = config->confirm_ms;
            item->recovery_ms = 0U;
            item->recovery_ready = 0U;

            // 只有达到独立恢复阈值，才开始累计恢复稳定时间。
            if (recovery_condition != 0U)
            {
                item->state = FAULT_STATE_RECOVERY_PENDING;
                item->state_since_ms = now_ms;

                if (config->recovery_confirm_ms == 0U)
                {
                    item->recovery_ready = 1U;
                }
            }
            break;

        case FAULT_STATE_RECOVERY_PENDING:
            /*
             * 原故障重新出现，或者恢复条件失去，
             * 都必须回到CONFIRMED重新累计恢复时间。
             */
            if ((trip_condition != 0U) ||
                (recovery_condition == 0U))
            {
                item->state = FAULT_STATE_CONFIRMED;
                item->state_since_ms = now_ms;
                item->recovery_ms = 0U;
                item->recovery_ready = 0U;
            }
            else
            {
                elapsed_ms = (uint32_t)(now_ms - item->state_since_ms);
                item->recovery_ms = elapsed_ms;

                // 恢复时间达到要求后，人工故障只标记ready，不会自行清除。
                if (elapsed_ms >= config->recovery_confirm_ms)
                {
                    item->recovery_ready = 1U;

                    if (config->manual_clear == 0U)
                    {
                        FaultItem_ClearConfirmed(id, item);
                    }
                }
            }
            break;

        default:
            /*
             * 状态值被破坏时回到CLEAR。
             * 后续完整系统会同时报告FAULT_INTERNAL。
             */
            FaultItem_SetClear(item);
            break;
    }
}

/*
 * 汇总每种故障的独立状态，生成active、latched、critical和recovery_ready掩码。
 * 先把旧的汇总结果清零
 *        ↓
 * 依次检查8种故障
 *       ↓
 * 根据每种故障的状态和配置设置对应掩码
 */

void FaultManager_EndEvaluation(void)
{
    uint8_t i;  //当前正在检查第几个故障
    uint16_t mask;  //当前故障对应的掩码
    FaultItem_t *item; //当前故障的运行状态
    const FaultConfig_t *config; //当前故障的固定配置

    /*
     * 汇总掩码每轮从items[]重新生成，而不是只置位不清位。
     * 这样故障清除后不会在总故障码中残留旧位。
     */
    g_fault_manager.active_mask = FAULT_MASK_NONE;
    g_fault_manager.latched_mask = FAULT_MASK_NONE;
    g_fault_manager.critical_mask = FAULT_MASK_NONE;
    g_fault_manager.recovery_ready_mask = FAULT_MASK_NONE;

    for (i = 0U; i < (uint8_t)FAULT_ID_COUNT; i++)
    {
        item = &g_fault_manager.items[i];
        config = &g_fault_config[i];
        mask = config->mask;

        // PENDING尚未确认，不对外报告为活动故障。
        if ((item->state == FAULT_STATE_CONFIRMED) ||
            (item->state == FAULT_STATE_RECOVERY_PENDING))
        {
            g_fault_manager.active_mask |= mask;

            if (config->manual_clear != 0U)
            {
                g_fault_manager.latched_mask |= mask;
            }

            if (config->level == FAULT_LEVEL_CRITICAL)
            {
                g_fault_manager.critical_mask |= mask;
            }

            if (item->recovery_ready != 0U)
            {
                g_fault_manager.recovery_ready_mask |= mask;
            }
        }
    }
}

/*
 * 检查快照字段之间是否自洽。
 * NTC开路或短路时temperature_valid为0是预期结果，
 * 不能再重复报告为SENSOR_DATA故障。
 */
static uint8_t FaultManager_IsSampleConsistent(const SensorSample_t *sample)
{
    // 没有有效序号、ADC超出12位范围或电压换算无效，都属于不可信快照。
    if ((sample == 0) ||
        (sample->sequence == 0U) ||
        (sample->adc_pa0 > 4095U) ||
        (sample->adc_pa1 > 4095U) ||
        (sample->voltage_valid == 0U))
    {
        return 0U;
    }

    // NTC分类为正常时，必须同时存在有效温度结果。
    if (sample->ntc_input == NTC_INPUT_NORMAL)
    {
        return (sample->temperature_valid != 0U) ? 1U : 0U;
    }

    // NTC已分类为开路/短路时，温度无效是合理结果，不重复报告SENSOR_DATA。
    if ((sample->ntc_input == NTC_INPUT_OPEN_SUSPECT) ||
        (sample->ntc_input == NTC_INPUT_SHORT_SUSPECT))
    {
        return (sample->temperature_valid == 0U) ? 1U : 0U;
    }

    return 0U;
}

/*
 * 把一帧Sensor Service快照转换为完整的传感器故障矩阵。
 * 物理量无效时，OV/UV/OT既不继续确认，也不允许进入恢复。
 * 
 * FaultManager_UpdateSensor()负责把一帧 SensorSample_t 转换成各种故障的布尔条件：
 * 真实测量数据
    ↓
 *这帧数据存在吗？
    ↓
 *这帧数据过期了吗？
    ↓
 *字段之间是否自洽？
    ↓
 *电压和温度能不能用于故障判断？
    ↓
 *生成OV、UV、OT、NTC、数据异常、ADC超时条件
    ↓
 *交给FaultManager_UpdateCondition()处理生命周期
 */
void FaultManager_UpdateSensor(const SensorSample_t *sample,    //传感器快照地址
                               uint8_t sample_available,    //是否成功接收到一帧快照
                               uint32_t now_ms)
{
    /*
     *不是有数据就能判读故障，还需经过：
     *存在
        ↓
     *新鲜
        ↓
     *对应物理量有效
        ↓
     *才允许比较阈值
     */
    uint8_t present;    //有没有一帧已发布的数据？
    uint8_t fresh;  //这帧数据是否新鲜？(年龄<100ms)
    uint8_t consistent; //快照中的字段是否互相符合逻辑？
    uint8_t voltage_usable; //电压是否可用于OV/UV判断？(新鲜且已换算)
    uint8_t temperature_usable; //温度是否可用于OT判断？(新鲜且已换算且NTC分类正常)

    // present只证明存在一帧带有效序号的快照。
    present = ((sample_available != 0U) &&
               (sample != 0) &&
               (sample->sequence != 0U)) ? 1U : 0U;

    // 快照年龄达到100ms即视为超时；时间差写法兼容Tick回绕。
    fresh = ((present != 0U) &&
             ((uint32_t)(now_ms - sample->timestamp_ms) < 100U)) ? 1U : 0U;

    // 只有新鲜快照才进行字段一致性判断，过期数据统一交给ADC_TIMEOUT处理。
    consistent = (fresh != 0U) ?
                 FaultManager_IsSampleConsistent(sample) : 0U;

    // OV和UV必须使用新鲜、已成功换算的电压。
    voltage_usable = ((fresh != 0U) &&
                      (sample->voltage_valid != 0U)) ? 1U : 0U;

    // OT必须使用新鲜、分类正常且已成功换算的温度。
    temperature_usable = ((fresh != 0U) &&
                          (sample->temperature_valid != 0U) &&
                          (sample->ntc_input == NTC_INPUT_NORMAL)) ? 1U : 0U;

    // 一帧快照对应一次完整评估周期。
    FaultManager_BeginEvaluation();

    FaultManager_UpdateCondition(
        FAULT_ID_OV,
        // 触发阈值4.20V，恢复阈值4.10V，中间0.10V形成迟滞区。
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v > 4.20f)),
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v < 4.10f)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_UV,
        // 触发阈值3.00V，恢复阈值3.10V，中间0.10V形成迟滞区。
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v < 3.00f)),
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v > 3.10f)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_OT,
        // 触发阈值60℃，恢复阈值55℃，中间5℃形成迟滞区。
        (uint8_t)((temperature_usable != 0U) &&
                  (sample->temperature_c > 60.0f)),
        (uint8_t)((temperature_usable != 0U) &&
                  (sample->temperature_c < 55.0f)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_NTC_OPEN,
        // NTC开路由Sensor分类结果触发；恢复要求NTC正常且温度可用。
        (uint8_t)((fresh != 0U) &&
                  (sample->ntc_input == NTC_INPUT_OPEN_SUSPECT)),
        (uint8_t)(temperature_usable != 0U),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_NTC_SHORT,
        // NTC短路由Sensor分类结果触发；恢复要求NTC正常且温度可用。
        (uint8_t)((fresh != 0U) &&
                  (sample->ntc_input == NTC_INPUT_SHORT_SUSPECT)),
        (uint8_t)(temperature_usable != 0U),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_SENSOR_DATA,
        // 仅在快照仍新鲜时判断字段矛盾，避免与ADC超时重复报告。
        (uint8_t)((fresh != 0U) && (consistent == 0U)),
        (uint8_t)((fresh != 0U) && (consistent != 0U)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_ADC_TIMEOUT,
        // 没有快照或快照年龄达到100ms时立即确认ADC超时。
        (uint8_t)(fresh == 0U),
        (uint8_t)(fresh != 0U),
        now_ms);

    // 所有传感器故障更新完成后，统一重建对外掩码。
    FaultManager_EndEvaluation();
}



uint16_t FaultManager_RequestManualClear(uint16_t requested_mask)
{
    uint8_t i;
    uint16_t eligible_mask;
    uint16_t cleared_mask = FAULT_MASK_NONE;

    /*
     * 三个条件必须同时满足：用户请求、故障已锁存、恢复已经稳定。
     * 因此按键不能清除仍处于异常条件下的故障。
     */
    eligible_mask = requested_mask &
                    g_fault_manager.latched_mask &
                    g_fault_manager.recovery_ready_mask;

    for (i = 0U; i < (uint8_t)FAULT_ID_COUNT; i++)
    {
        if ((eligible_mask & g_fault_config[i].mask) != 0U)
        {
            FaultItem_ClearConfirmed((FaultId_t)i,
                                     &g_fault_manager.items[i]);
            cleared_mask |= g_fault_config[i].mask;
        }
    }

    FaultManager_EndEvaluation();
    return cleared_mask;
}


/*
 * FaultManager_GetSnapshot — 取得一份完整、一致的故障管理器快照。
 *
 * BmsTask是故障管理器的唯一写入者；
 * CLI、CAN和诊断功能只能通过本接口读取副本。
 *
 * 复制期间使用短临界区，防止BmsTask在结构体复制到一半时抢占，
 * 从而避免调用者读到由新旧两轮数据拼成的快照。
 */
uint8_t FaultManager_GetSnapshot(
    FaultManagerSnapshot_t *snapshot)
{
    /*
     * 空指针保护：
     * 调用者没有提供有效的接收地址时，
     * 不能向该地址写入结构体。
     */
    if (snapshot == 0)
    {
        return 0U;
    }

    /*
     * 临界区只包含一次RAM结构体复制。
     *
     * 临界区内不能加入printf、Flash、CAN发送、
     * vTaskDelay或者其他可能长时间执行的操作。
     */
    taskENTER_CRITICAL();

    *snapshot = g_fault_manager;

    taskEXIT_CRITICAL();

    return 1U;
}


void FaultManager_SetInjectionMask(uint16_t mask)
{
    /* 过滤不允许注入的位，避免伪造SELF_CHECK或真实CAN硬件状态。 */
    g_fault_inject_mask = mask & FAULT_INJECT_ALLOWED_MASK;
}


uint16_t FaultManager_GetInjectionMask(void)
{
    return g_fault_inject_mask;
}

