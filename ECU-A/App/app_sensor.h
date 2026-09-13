/*
 * 模块：公共传感器换算与最新快照服务。
 * 输入：ADC/DMA提供的PA0电压通道和PA1温度通道原始值。
 * 输出：带序号、时间戳、物理量、有效标志和NTC分类的完整快照。
 * Sensor Service是ADC_ReadSafe的唯一消费者；其他模块只读取快照。
 * 本模块不确认故障、不改变BMS状态、不发送CAN、不写Flash。
 */
#ifndef APP_SENSOR_H
#define APP_SENSOR_H

// 引入固定宽度整数类型，避免依赖具体单片机寄存器。
#include <stdint.h>

// 定义NTC输入分类，保留原测试程序使用的名称和编号。
typedef enum
{
    // 尚未分类。
    NTC_INPUT_UNKNOWN = 0,
    // 原始值允许进行温度换算，不代表温度未超限。
    NTC_INPUT_NORMAL = 1,
    // 输入接近满量程，疑似NTC或线路开路。
    NTC_INPUT_OPEN_SUSPECT = 2,
    // 输入接近地电位，疑似NTC或线路短路。
    NTC_INPUT_SHORT_SUSPECT = 3,
    // 原始值超出12位ADC范围。
    NTC_INPUT_ADC_INVALID = 4
} NTC_InputState_t;

// 定义一次换算的完整结果，由调用者保存。
typedef struct
{
    // 每接收一组新的双通道数据增加一次；0表示尚无有效快照。
    uint32_t sequence;
    // Sensor Service接收这组数据时的系统毫秒时间。
    uint32_t timestamp_ms;
    // 保存PA0原始读数。
    uint16_t adc_pa0;
    // 保存PA1原始读数。
    uint16_t adc_pa1;
    // 保存PA0引脚电压，单位V。
    float pa0_voltage_v;
    // 保存还原后的电池电压，单位V。
    float cell_voltage_v;
    // 保存NTC温度，单位摄氏度。
    float temperature_c;
    // 1表示电压已换算，0表示本次电压不可用。
    uint8_t voltage_valid;
    // 1表示温度已换算，0表示本次温度不可用。
    uint8_t temperature_valid;
    // 保存本次NTC输入分类，不等于已确认的故障码。
    NTC_InputState_t ntc_input;
} SensorSample_t;


/*
 * 换算一组原始值；每次覆盖结果，各通道独立判断。
 * 无效通道的数值字段填0，使用者必须先检查对应valid标志。
 * sample为空时直接返回，不访问任何输出。
 * 本接口不记录采样时刻，不证明数据新鲜或两个通道同步。
 */
void Sensor_Convert(uint16_t adc0, uint16_t adc1, SensorSample_t *sample);

/* 初始化最新快照；
 *应在ADC开始产生数据前调用一次。 */
void Sensor_ServiceInit(void);

/*
 * 尝试从ADC/DMA取得一组新数据。
 * 成功生成新快照返回1；当前没有新数据返回0。
 */
uint8_t Sensor_ServiceUpdate(uint32_t now_ms);

/* 有快照时复制到调用者并返回1；尚无快照或参数为空时返回0。 */
uint8_t Sensor_GetLatest(SensorSample_t *sample);

/* 判断快照年龄是否未超过允许值；没有快照或参数为空时返回0。 */
uint8_t Sensor_IsFresh(const SensorSample_t *sample,
                       uint32_t now_ms,
                       uint32_t max_age_ms);

#endif
