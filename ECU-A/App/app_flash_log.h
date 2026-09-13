/*
 * 文件名称：app_flash_log.h
 *
 * 模块名称：Flash V2故障黑匣子日志
 *
 * 模块职责：
 * 1. 把故障确认、故障清除和系统启动事件写入外部SPI Flash；
 * 2. 保存故障时间、故障掩码、电压、温度、SOC、BMS状态、
 *    CAN状态和本次上电的复位原因；
 * 3. 使用固定32字节记录和CRC8检查记录完整性；
 * 4. 提供日志初始化、写入、串口打印和清除接口。
 *
 * 模块边界：
 * 本模块不判断故障条件，不修改BMS状态，也不控制蜂鸣器和CAN。
 * 它只负责持久化已经确认的故障事件和本次系统启动信息。
 *
 * 存储区域：
 * V1旧日志保留在外部Flash的0x000000～0x000FFF；
 * V2新日志使用0x001000～0x001FFF。
 */
#ifndef APP_FLASH_LOG_H
#define APP_FLASH_LOG_H

#include <stdint.h>

#include "app_fault_event.h"


/* 一条V2记录固定为32字节。 */
#define FLASH_LOG_RECORD_SIZE       32U

/* 一个4KB扇区可以保存128条32字节记录。 */
#define FLASH_LOG_MAX_RECORDS       128U


/*
 * 写入Flash的CAN状态。
 * 记录的是故障事件发生时系统确认的通信状态。
 */
typedef enum
{
    FLASH_LOG_CAN_OK = 0,
    FLASH_LOG_CAN_DEGRADED = 1,
    FLASH_LOG_CAN_BUS_OFF = 2
} FlashLogCanState_t;


/*
 * 本次程序启动的复位原因。
 * 一次上电期间产生的所有故障记录使用同一个复位原因。
 */
typedef enum
{
    FLASH_LOG_RESET_UNKNOWN = 0,
    FLASH_LOG_RESET_POWER_ON = 1,
    FLASH_LOG_RESET_PIN = 2,
    FLASH_LOG_RESET_SOFTWARE = 3,
    FLASH_LOG_RESET_IWDG = 4,
    FLASH_LOG_RESET_WWDG = 5,
    FLASH_LOG_RESET_LOW_POWER = 6
} FlashLogResetReason_t;


/*
 * 供Keil Watch观察的日志诊断变量。
 * 业务代码只能读取，不能直接修改。
 */
extern volatile uint16_t g_flash_log_next_index;
extern volatile uint32_t g_flash_log_write_ok_count;
extern volatile uint32_t g_flash_log_write_fail_count;
extern volatile uint32_t g_flash_log_crc_error_count;
extern volatile uint32_t g_flash_log_erase_fail_count;
extern volatile FlashLogResetReason_t g_flash_log_reset_reason;


/*
 * 初始化Flash V2日志。
 *
 * 主要工作：
 * 1. 读取并保存本次启动的复位原因；
 * 2. 扫描V2日志扇区；
 * 3. 找到下一条可写记录的位置；
 * 4. 统计扫描过程中发现的CRC错误。
 */
void FlashLog_Init(void);


/*
 * 写入一条系统启动记录。
 *
 * 该记录不经过故障事件队列，因为系统刚启动时还没有故障边沿；
 * 记录中的Reset字段用于说明本次启动由上电、按键、软件或看门狗引起。
 * 写入成功返回1，失败返回0。
 */
uint8_t FlashLog_WriteBootEvent(void);


/*
 * 写入一条完整故障事件。
 *
 * 成功写入并回读验证通过返回1；
 * 参数无效或写入验证失败返回0。
 */
uint8_t FlashLog_WriteEvent(const FaultEvent_t *event);


/*
 * 从Flash逐条读取并通过USART打印。
 *
 * 每次只在栈上保存一条32字节记录，
 * 不再申请旧版的4096字节局部数组。
 */
void FlashLog_PrintAll(void);


/*
 * 擦除V2日志扇区并清零日志诊断信息。
 *
 * 不擦除0号扇区中的V1旧日志。
 */
/* 擦除并回读验证成功返回1；擦除未生效返回0。 */
uint8_t FlashLog_Clear(void);


#endif

