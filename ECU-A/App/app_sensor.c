/*
 * 模块：公共传感器换算与最新快照服务。
 * PA0：原始值 → 引脚电压 → 按1.5倍还原电池电压。
 * PA1：先分类输入，再按上拉10k、NTC为10k/B3950换算温度。
 * Sensor Service独占ADC_ReadSafe，并为每组新数据附加序号和时间戳。
 * 其他模块读取快照，不再直接消费DMA就绪标志。
 * 最新快照的发布和复制使用FreeRTOS短临界区保护，
 * 防止SensorTask与BMS、CAN、CLI任务并发时读到不完整数据。
 * 本模块只提供测量事实，不执行故障确认、锁存、状态切换或保护动作。
 */

// 引入公共数据类型与函数声明。
#include "app_sensor.h"
/*
 * FreeRTOS.h提供内核配置和公共类型；
 * task.h提供任务临界区接口。
 */
#include "FreeRTOS.h"
#include "task.h"
//引入ADC/DMA安全读取接口，只允许本服务使用
#include "bsp_adc_dma.h"

// 引入NTC公式使用的自然对数函数。
#include <math.h>

// 沿用当前参考电压，不补偿损坏的稳压器。
#define SENSOR_ADC_VREF_V       3.27f

// 电池分压上方10k、下方20k，还原比例为1.5。
#define SENSOR_CELL_GAIN        1.5f

// 沿用当前NTC低端测试阈值。
#define SENSOR_NTC_ADC_LOW      16U

// 沿用当前NTC高端测试阈值。
#define SENSOR_NTC_ADC_HIGH     4079U

// 保存最近一次完整快照；只由Sensor_ServiceUpdate写入。
static SensorSample_t g_latest_sample;

// 保存最近一次分配的快照序号；0保留给“尚无快照”。
static uint32_t g_sample_sequence = 0U;

// 标记服务是否至少成功接收过一组数据。
static uint8_t g_has_sample = 0U;

/*
 * 模块：一次采样结果换算。
 * 电压与温度独立处理，一个通道无效不阻止另一个通道更新。
 * 无效数值填0，仅用于初始化字段，不能当作真实测量值使用。
 */
void Sensor_Convert(uint16_t adc0, uint16_t adc1, SensorSample_t *sample)
{
    // 保存NTC电阻，单位欧姆。
    float resistance;

    // 保存绝对温度的倒数。
    float inverse_kelvin;

    // 没有输出对象时直接返回。
    if (sample == 0)
    {
        return;
    }

     // 纯换算结果没有采集上下文，元数据由Sensor Service在成功读取后填写。
    sample->sequence = 0U;
    sample->timestamp_ms = 0U;


    // 保存原始读数，便于后续诊断。
    sample->adc_pa0 = adc0;
    sample->adc_pa1 = adc1;

    // 每次重新初始化结果，防止沿用上一帧的有效标志。
    sample->pa0_voltage_v = 0.0f;
    sample->cell_voltage_v = 0.0f;
    sample->temperature_c = 0.0f;
    sample->voltage_valid = 0U;
    sample->temperature_valid = 0U;
    sample->ntc_input = NTC_INPUT_UNKNOWN;

    // 只要电压原始值在12位范围内，就进行数值换算。
    if (adc0 <= 4095U)
    {
        // 计算PA0引脚电压。
        sample->pa0_voltage_v = (float)adc0 * SENSOR_ADC_VREF_V / 4095.0f;

        // 还原电池电压，不把0V或满量程当作安全状态。
        sample->cell_voltage_v = sample->pa0_voltage_v * SENSOR_CELL_GAIN;

        // 标记电压换算成功，超压和欠压由故障模块判断。
        sample->voltage_valid = 1U;
    }

    // 先排除越界，避免把损坏的数据误分类为开路。
    if (adc1 > 4095U)
    {
        sample->ntc_input = NTC_INPUT_ADC_INVALID;
    }
    // 接近地电位时报告疑似短路，暂不确认故障。
    else if (adc1 <= SENSOR_NTC_ADC_LOW)
    {
        sample->ntc_input = NTC_INPUT_SHORT_SUSPECT;
    }
    // 接近满量程时报告疑似开路，暂不确认故障。
    else if (adc1 >= SENSOR_NTC_ADC_HIGH)
    {
        sample->ntc_input = NTC_INPUT_OPEN_SUSPECT;
    }
    // 只对允许换算的输入计算温度。
    else
    {
        // 根据上拉10k、下接NTC的分压关系计算电阻。
        resistance = 10000.0f * (float)adc1 / (4095.0f - (float)adc1);

        // 使用R25=10k、B=3950计算绝对温度倒数。
        inverse_kelvin = 1.0f / 298.15f
                      + logf(resistance / 10000.0f) / 3950.0f;

        // 转换为摄氏度，保留负温度。
        sample->temperature_c = 1.0f / inverse_kelvin - 273.15f;

        // 标记允许使用温度结果，是否过温另行判断。
        sample->temperature_valid = 1U;
        sample->ntc_input = NTC_INPUT_NORMAL;
    }
}

/* 初始化传感器快照服务。 */
void Sensor_ServiceInit(void)
{
    SensorSample_t empty_sample = {0};

    g_latest_sample = empty_sample;
    g_sample_sequence = 0U;
    g_has_sample = 0U;
}

/* 尝试接收并发布一组新的双通道快照。 */
uint8_t Sensor_ServiceUpdate(uint32_t now_ms)
{
    uint16_t adc0;
    uint16_t adc1;
    uint32_t next_sequence;// 下一帧快照序号
    SensorSample_t new_sample;// 局部对象，避免对外暴露不完整的快照

    // 没有新的DMA半缓冲或全缓冲时，不改变现有快照。
    if (ADC_ReadSafe(&adc0, &adc1) == 0U)
    {
        return 0U;
    }

    // 先在局部对象中完成换算，避免对外暴露只更新了一半的结果。
    Sensor_Convert(adc0, adc1, &new_sample);

    // 计算下一序号；0永久保留为“从未发布过快照”。
    next_sequence = g_sample_sequence + 1U;
    if (next_sequence == 0U)
    {
        next_sequence = 1U;
    }

    // 元数据只代表这次真正从ADC/DMA取得的新帧。
    new_sample.sequence = next_sequence;
    new_sample.timestamp_ms = now_ms;

    // 完整构造后一次发布，当前裸机协作式调度下不会出现任务并发复制。
    /*
    * new_sample的浮点换算已经在临界区外完成。
    *
    * 临界区中只复制最终结果和更新发布状态，
    * 防止其他任务复制到一半时被SensorTask改写。
    */    
    taskENTER_CRITICAL();    //

    g_latest_sample = new_sample;
    g_sample_sequence = next_sequence;
    g_has_sample = 1U;

    taskEXIT_CRITICAL();    

    return 1U;
}

/*
 * 复制最近一次完整快照。
 *
 * 返回的是一份独立副本。调用者退出本函数后，
 * SensorTask继续更新内部快照也不会改变调用者已经取得的数据。
 */
uint8_t Sensor_GetLatest(SensorSample_t *sample)
{
    uint8_t sample_available;
    //空指针不进入临界区，也不作为复制目标

    if (sample == 0)
    {
        return 0U;
    }

    taskENTER_CRITICAL();    // 保护对全局快照的访问，防止被SensorTask并发修改

    sample_available = g_has_sample;
    if(sample_available != 0U)
    {
        *sample = g_latest_sample;
    }
    taskEXIT_CRITICAL();    

    return sample_available;
}

/* 根据时间戳计算快照是否仍在允许年龄内。 */
uint8_t Sensor_IsFresh(const SensorSample_t *sample,
                       uint32_t now_ms,
                       uint32_t max_age_ms)
{
    if ((sample == 0) || (sample->sequence == 0U))
    {
        return 0U;
    }

    if ((uint32_t)(now_ms - sample->timestamp_ms) > max_age_ms)
    {
        return 0U;
    }

    return 1U;
}

