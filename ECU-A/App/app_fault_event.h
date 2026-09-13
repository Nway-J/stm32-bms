/*
 * 文件名称：app_fault_event.h
 *
 * 模块名称：故障边沿事件队列公共接口
 *
 * 模块职责：
 * 1. 定义故障“新确认”“新清除”和“系统启动”三类事件的数据格式。
 *
 * 2. 保存故障状态发生变化瞬间的固定宽度上下文，
 *    包括：
 *    - 事件发生时间
 *    - 本次发生变化的故障位
 *    - 变化后仍然存在的故障位
 *    - 电池电压快照
 *    - 温度快照
 *    - BMS状态机状态
 *    - 传感器数据有效性
 *
 * 3. 为日志、USART诊断、CAN上传等后续消费者
 *    提供统一的故障事件数据来源。
 *
 * 补充说明：
 * FaultEvent_t也被启动日志复用，但SYSTEM_BOOT不会进入本模块的RAM队列；
 * 启动记录由FlashLog在初始化阶段直接构造并写入Flash。
 *
 * 模块边界：
 * 本模块只负责“保存和传递已经产生的故障事件”，不负责：
 *
 * - 判断某个电压是否属于过压；
 * - 判断某个温度是否属于过温；
 * - 执行故障确认延时或消抖；
 * - 修改BMS状态机；
 * - 控制继电器、MOS、蜂鸣器等执行器；
 * - 直接访问SPI Flash；
 * - 直接发送CAN报文；
 * - 决定故障恢复策略。
 *
 * 换句话说：
 *
 *     故障检测模块
 *          ↓
 *     发现故障边沿
 *          ↓
 *     FaultEvent_Push()
 *          ↓
 *     RAM事件队列
 *          ↓
 *     日志 / CAN / USART 等消费者
 *
 * 并发模型：
 * 当前裸机版本按单生产者、单消费者场景设计。
 *
 * 并发模型：
 * 内部使用FreeRTOS静态Queue。
 *
 * 当前生产者是BMS故障判断路径，
 * 当前消费者是故障日志处理路径。
 *
 * 入队和出队可以由不同任务调用；
 * 调用者不需要直接操作队列索引，也不需要额外添加临界区。
 */

#ifndef APP_FAULT_EVENT_H 
#define APP_FAULT_EVENT_H 
 
#include <stdint.h> 
#include "FreeRTOS.h"
#include "app_sensor.h" 
 
// 队列固定保存8条事件，内部使用静态内存，不从Heap动态申请 
#define FAULT_EVENT_QUEUE_CAPACITY       8U 
 
// valid_flags的Bit0：事件中的电池电压有效。 
#define FAULT_EVENT_VALID_VOLTAGE        ((uint8_t)0x01U) 
 
// valid_flags的Bit1：事件中的温度有效。 
#define FAULT_EVENT_VALID_TEMPERATURE    ((uint8_t)0x02U) 

// valid_flags的Bit7：本次新确认事件来自CLI测试注入。 
#define FAULT_EVENT_FLAG_INJECTED         ((uint8_t)0x80U) 
 
// 区分故障确认、故障清除和系统启动三种日志事件。 
typedef enum 
{ 
    FAULT_EVENT_CONFIRMED = 1, //故障确认
    FAULT_EVENT_CLEARED = 2, //故障清除
    FAULT_EVENT_SYSTEM_BOOT = 3 //系统启动
} FaultEventType_t; 
 
/* 
 * 一条系统事件的固定宽度快照。
 * 故障事件用changed_mask描述本次变化，用active_mask描述剩余故障；
 * 启动事件没有故障变化，因此两个掩码都为0。
 */ 
typedef struct 
{ 
    uint32_t timestamp_ms; //事件发生时间，单位ms
    uint16_t changed_mask; //本次新发生或新清除的故障
    uint16_t active_mask;  //变化后仍然存在的全部故障
    uint16_t cell_voltage_mv; //电池电压快照，单位mV
    int16_t temperature_dC; //温度快照，单位0.1℃
    /* 使用明确的1字节存储，避免不同编译器改变Flash记录布局。 */ 
    uint8_t event_type;  // 1=新确认，2=新清除，3=系统启动
    uint8_t bms_state;  //BMS状态机状态，取值范围0~255
    uint8_t valid_flags; //Bit0=电压有效，Bit1=温度有效，Bit7=测试注入
} FaultEvent_t; 
 
/* 
 * 供Keil Watch观察的队列诊断信息。 
 * 业务代码不得通过该变量修改队列，只能调用下面的公共接口。 
 */ 
typedef struct 
{ 
    FaultEvent_t last_pushed; //最近一次成功写入队列的事件快照
    FaultEvent_t last_processed; //最近一次成功从队列取出的事件快照
    uint32_t pushed_count;   //自上电以来成功写入队列的事件总数
    uint32_t processed_count; //自上电以来成功从队列取出的事件总数
    uint32_t dropped_count;   //自上电以来因队列满而被丢弃的事件总数
    uint8_t depth;            //当前队列深度
    uint8_t last_pushed_valid; //最近一次写入的事件是否有效
    uint8_t last_processed_valid; //最近一次取出的事件是否有效
} FaultEventDebug_t; 
 
extern volatile FaultEventDebug_t g_fault_event_debug; 
 
// 创建或复位FreeRTOS静态队列，并清除全部Watch诊断信息 
void FaultEvent_Init(void); 
 
/* 
 * 非阻塞写入一条故障事件。 
 * 成功返回1；参数非法或队列已满返回0。队列满不会影响保护动作。 
 */ 
uint8_t FaultEvent_Push(FaultEventType_t event_type, 
                        uint16_t changed_mask, 
                        uint16_t active_mask, 
                        const SensorSample_t *sample, 
                        uint8_t bms_state, 
                        uint32_t timestamp_ms); 
 
/*
 * 从队列取出最早的一条故障事件。
 *
 * wait_ticks = 0：
 * 队列为空时立即返回。
 *
 * wait_ticks = portMAX_DELAY：
 * 队列为空时阻塞当前任务，直到新事件进入队列。
 */
uint8_t FaultEvent_Pop(
    FaultEvent_t *event,
    TickType_t wait_ticks);
 
/*
 * 等待并处理一条故障事件。
 *
 * 等待时间由调用者决定：
 * 旧调度器传0，不阻塞；
 * 后续FaultLogTask传portMAX_DELAY，阻塞等待事件。
 */
void FaultEvent_ProcessOne(
    TickType_t wait_ticks);
 
#endif 
