/*
 * 模块：统一故障管理器公共接口。
 * 作用：定义16位故障编码、单故障生命周期、汇总状态和人工清除接口。
 * 边界：本模块不读取ADC寄存器、不操作蜂鸣器、不发送CAN、不写Flash，
 *       也不直接切换BMS状态；具体故障来源只向它提交布尔条件和系统时间。
 * 并发：
 * 1. BmsTask是故障生命周期和汇总状态的唯一修改者；
 * 2. 其他任务只能通过FaultManager_GetSnapshot()读取副本；
 * 3. 快照复制使用短临界区，保证结构体内部数据来自同一时刻；
 * 4. CLI只能修改独立的测试注入掩码，不能直接修改故障状态。
 */
#ifndef APP_FAULT_MANAGER_H
#define APP_FAULT_MANAGER_H

#include <stdint.h>

// 引入SensorSample_t，故障管理器通过传感器快照取得可信数据。
#include "app_sensor.h"

/*
 * 故障位掩码。--用来表示故障位
 * 每种故障占一个二进制位，所以多个故障可以按位或后同时保存。
 * 例如：UV和OT同时存在时，故障掩码为0x0002 | 0x0004 = 0x0006。
* 当前启用bit0～bit9；其余高位留给任务健康、Watchdog和Flash等故障。
 * 每种故障占一个二进制位
 */

#define FAULT_MASK_NONE          ((uint16_t)0x0000U)
#define FAULT_MASK_OV            ((uint16_t)0x0001U)
#define FAULT_MASK_UV            ((uint16_t)0x0002U)
#define FAULT_MASK_OT            ((uint16_t)0x0004U)
#define FAULT_MASK_NTC_OPEN      ((uint16_t)0x0008U)
#define FAULT_MASK_NTC_SHORT     ((uint16_t)0x0010U)
#define FAULT_MASK_SENSOR_DATA   ((uint16_t)0x0020U)
#define FAULT_MASK_ADC_TIMEOUT   ((uint16_t)0x0040U)
#define FAULT_MASK_SELF_CHECK    ((uint16_t)0x0080U)
// CAN错误计数较高，通信质量已经下降，但控制器尚未离线。
#define FAULT_MASK_CAN_DEGRADED  ((uint16_t)0x0100U)

// CAN发送错误计数超过限制，控制器已经进入Bus-Off。
#define FAULT_MASK_CAN_BUS_OFF   ((uint16_t)0x0200U)

/*
 * CLI允许注入的故障范围。
 * 不包含SELF_CHECK和CAN硬件故障，避免软件测试冒充真实硬件状态。
 */
#define FAULT_INJECT_ALLOWED_MASK  \
    ((uint16_t)(FAULT_MASK_OV | FAULT_MASK_UV | FAULT_MASK_OT | \
                FAULT_MASK_NTC_OPEN | FAULT_MASK_NTC_SHORT | \
                FAULT_MASK_SENSOR_DATA | FAULT_MASK_ADC_TIMEOUT))

/* 当前测试注入掩码，公开用于Keil Watch诊断，业务代码通过接口修改。 */
extern volatile uint16_t g_fault_inject_mask;
/*
 * 故障数组下标。--用来找数组位置
 * FaultId_t只用于访问items[]和配置表；对外传输使用上面的16位故障掩码。
 */
typedef enum
{
    // 电池过压。
    FAULT_ID_OV = 0,
    // 电池欠压。
    FAULT_ID_UV,
    // 电池过温。
    FAULT_ID_OT,
    // NTC或其线路疑似开路。
    FAULT_ID_NTC_OPEN,
    // NTC或其线路疑似短路。
    FAULT_ID_NTC_SHORT,
    // Sensor快照原始值或字段关系不可信（数据异常）。
    FAULT_ID_SENSOR_DATA,
    // 超过允许时间没有得到新鲜ADC快照。
    FAULT_ID_ADC_TIMEOUT,
    // 上电自检失败；当前只预留，后续由自检流程接入。
    FAULT_ID_SELF_CHECK,
    // CAN错误计数较高，通信处于降级状态。
    FAULT_ID_CAN_DEGRADED,
    // CAN控制器进入Bus-Off。
    FAULT_ID_CAN_BUS_OFF,

    // 当前启用的故障项数量，同时作为数组长度。
    FAULT_ID_COUNT
} FaultId_t;

/*
 * 单个故障的生命周期状态。
 * CLEAR -> PENDING -> CONFIRMED -> RECOVERY_PENDING -> 人工/自动清除。
 * CLEAR-当前没有确认故障
 *PENDING-条件已经异常，但持续时间还不足
 *CONFIRMED-持续时间达到要求，故障正式确认
 *RECOVERY_PENDING-故障条件已经消失，正在确认恢复是否稳定
 */
typedef enum
{
    // 当前没有确认故障，也没有正在进行的确认计时。
    FAULT_STATE_CLEAR = 0,
    // 触发条件已经出现，正在累计故障确认时间。
    FAULT_STATE_PENDING,
    // 触发条件已持续到规定时间，故障正式确认并保持有效。
    FAULT_STATE_CONFIRMED,
    // 已满足恢复阈值，正在累计恢复稳定时间。
    FAULT_STATE_RECOVERY_PENDING
} FaultState_t;

// 故障等级决定状态机是否必须进入BMS_FAULT。
typedef enum
{
    // 警告故障：记录和上报，但通常允许继续本地监测。
    FAULT_LEVEL_WARNING = 0,
    // 关键故障：要求状态机进入BMS_FAULT并执行本地报警。
    FAULT_LEVEL_CRITICAL
} FaultLevel_t;

/* 保存一种故障本次上电期间的运行信息。
 *单独保存一种故障当前走到了哪个阶段，以及相关计时和统计信息 
 *保存生命周期状态、进入状态的时间、确认时间、恢复时间、发生次数和恢复就绪标志
 */
typedef struct
{
    // 当前生命周期状态。
    FaultState_t state;
    // 进入当前状态的系统毫秒时间，用于计算无符号时间差。
    uint32_t state_since_ms;
    // 触发条件已经连续存在的时间，主要供Watch和诊断查看。
    uint32_t pending_ms;
    // 恢复条件已经连续成立的时间，主要供Watch和诊断查看。
    uint32_t recovery_ms;
    // 本次上电后该故障被正式确认的次数，达到最大值后保持不变。
    uint16_t occurrence_count;
    // 1表示恢复条件已稳定，但人工锁存故障仍需收到清除请求。
    uint8_t recovery_ready;
} FaultItem_t;//故障项目

/*
 * 故障管理器的完整快照。
 * items[]保存每种故障的详细状态；
 * 各mask提供给状态机、CAN和日志快速使用。
 */
typedef struct
{
    // 每种故障各有一个独立运行项，计时互不覆盖。
    FaultItem_t items[FAULT_ID_COUNT];

    // 已确认且尚未清除的全部故障，不包含仅处于PENDING的条件。(1:当前故障还存在)
    uint16_t active_mask;

    // 已确认且必须经过人工恢复流程清除的故障。(1:故障已经被锁存，必须按键清除)
    uint16_t latched_mask;

    // active_mask中等级为CRITICAL的故障，用于决定是否进入BMS_FAULT。(1:这是关键故障，系统必须保持FAULT)
    uint16_t critical_mask;

    // 已连续满足恢复条件、现在允许清除的故障。(1:运行清除；0：不允许清除)
    uint16_t recovery_ready_mask;

    // 只在故障首次确认的那一轮置位，用于CAN事件和Flash日志去重。
    uint16_t newly_confirmed_mask;

    // 只在故障真正被清除的那一轮置位，用于恢复事件记录。
    uint16_t newly_cleared_mask;
} FaultManagerSnapshot_t;

// 上电调用一次：清空全部故障状态、计时、计数和汇总掩码。
void FaultManager_Init(void);

// 开始一次完整评估：清除仅维持一轮的“新确认/新清除”事件掩码。
void FaultManager_BeginEvaluation(void);

/*
 * 更新一种故障的通用生命周期。
 * trip_condition为1表示当前满足触发条件；recovery_condition为1表示满足恢复条件。
 * 两个条件由具体数据来源生成，本函数只负责时间确认和状态转换。
 */
void FaultManager_UpdateCondition(FaultId_t id,
                                  uint8_t trip_condition,
                                  uint8_t recovery_condition,
                                  uint32_t now_ms);

// 结束一次完整评估：根据所有items[]重新生成活动、锁存、关键和恢复就绪掩码。
void FaultManager_EndEvaluation(void);

/*
 * 评估一帧Sensor快照。
 * sample_available表示调用者是否取得快照；sample可为空；now_ms是当前系统时间。
 * 本函数完成电压、温度、NTC、数据一致性和ADC超时条件的生成。
 */
void FaultManager_UpdateSensor(const SensorSample_t *sample,
                               uint8_t sample_available,
                               uint32_t now_ms);

/*
 * 请求人工清除指定故障位。
 * 只有同时处于latched_mask和recovery_ready_mask中的位才会真正清除；返回实际清除掩码。
 */
uint16_t FaultManager_RequestManualClear(uint16_t requested_mask);

// 将内部管理器复制给调用者；成功返回1，空指针返回0。
uint8_t FaultManager_GetSnapshot(FaultManagerSnapshot_t *snapshot);

/*
 * 设置或读取测试注入掩码。
 * 设置0表示撤销注入；输入中不允许的位会被自动过滤掉。
 */
void FaultManager_SetInjectionMask(uint16_t mask);
uint16_t FaultManager_GetInjectionMask(void);

#endif
