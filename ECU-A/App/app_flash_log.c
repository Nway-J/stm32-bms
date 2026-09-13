/*
 * 文件名称：app_flash_log.c
 *
 * 模块名称：Flash V2故障黑匣子日志
 *
 * 模块职责：
 * 1. 使用外部SPI Flash的0x001000～0x001FFF保存V2系统记录；
 * 2. 每条记录固定32字节，一个4KB扇区保存128条；
 * 3. 上电扫描记录，找到下一条安全的写入位置；
 * 4. 使用记录标记、版本号和CRC8识别无效或损坏记录；
 * 5. 保存每次系统启动事件及其复位原因。
 *
 * 设计原则：
 * Flash擦除后为0xFF，写入只能把1改为0；
 * 已经写过或部分写坏的位置不能直接覆盖；
 * 日志写满后才擦除整个V2扇区重新开始。
 *
 * 当前功能：
 * 已实现启动扫描、启动/故障事件写入、写后回读验证、
 * CRC完整性检查、历史日志打印和日志扇区清除。
 * 
 * 并发规则：
 * 1. 调度器启动前不存在任务并发，启动扫描和启动日志直接执行；
 * 2. 调度器运行后，写入、打印和清空操作共用一个静态Mutex；
 * 3. Mutex只保护Flash资源，不参与故障判断和保护动作；
 * 4. 所有外部调用者只能使用本文件提供的公共接口。
 */

#include "app_flash_log.h"
/*
 * Flash日志运行阶段由FaultLogTask和CLI共同访问，
 * 使用FreeRTOS Mutex保证同一时间只有一个访问者。
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>

#include "bsp_spi_flash.h"
#include "can_protocol.h"
#include "app_fault_manager.h"


/* V2使用外部Flash第二个4KB扇区，保留0号扇区中的V1旧日志。 */
#define FLASH_LOG_V2_SECTOR_ADDR     ((uint32_t)0x001000U)

/* 每条有效V2记录的固定开头。 */
#define FLASH_LOG_V2_MAGIC           ((uint8_t)0xA5U)
#define FLASH_LOG_V2_VERSION         ((uint8_t)0x02U)

/* CRC保存在32字节记录的最后一个字节。 */
#define FLASH_LOG_V2_CRC_OFFSET      31U

/*
 * 32字节V2记录中各字段的固定位置。
 * 使用明确偏移，避免直接把C结构体写入Flash造成对齐差异。
 */
#define FLASH_LOG_OFFSET_MAGIC          0U
#define FLASH_LOG_OFFSET_VERSION        1U
#define FLASH_LOG_OFFSET_EVENT_TYPE     2U
#define FLASH_LOG_OFFSET_VALID_FLAGS    3U
#define FLASH_LOG_OFFSET_SEQUENCE       4U
#define FLASH_LOG_OFFSET_TIMESTAMP      6U
#define FLASH_LOG_OFFSET_CHANGED_MASK  10U
#define FLASH_LOG_OFFSET_ACTIVE_MASK   12U
#define FLASH_LOG_OFFSET_VOLTAGE_MV    14U
#define FLASH_LOG_OFFSET_TEMPERATURE   16U
#define FLASH_LOG_OFFSET_SOC           18U
#define FLASH_LOG_OFFSET_BMS_STATE     19U
#define FLASH_LOG_OFFSET_CAN_STATE     20U
#define FLASH_LOG_OFFSET_RESET_REASON  21U

/*
 * Flash日志诊断数据。
 * 非static是为了让Keil Watch直接观察。
 */
volatile uint16_t g_flash_log_next_index = 0U;
volatile uint32_t g_flash_log_write_ok_count = 0U;
volatile uint32_t g_flash_log_write_fail_count = 0U;
volatile uint32_t g_flash_log_crc_error_count = 0U;
volatile uint32_t g_flash_log_erase_fail_count = 0U;
volatile FlashLogResetReason_t g_flash_log_reset_reason =
    FLASH_LOG_RESET_UNKNOWN;

/*
 * Flash日志互斥锁使用静态内存。
 *
 * control保存Mutex内部管理信息；
 * handle是后续获取和释放Mutex时使用的句柄。
 */
static StaticSemaphore_t g_flash_log_mutex_control;
static SemaphoreHandle_t g_flash_log_mutex_handle = 0;


/*
 * 申请Flash日志互斥锁。
 *
 * 调度器启动前：
 * 系统中还没有任务并发，允许直接访问Flash。
 *
 * 调度器运行后：
 * 必须先取得Mutex，避免自动写日志与CLI操作冲突。
 */
static uint8_t FlashLog_Lock(void)
{
    BaseType_t scheduler_state;

    scheduler_state =
        xTaskGetSchedulerState();

    /*
     * 启动日志在vTaskStartScheduler()之前写入。
     * 此时不存在任务并发，不获取Mutex。
     */
    if (scheduler_state ==
        taskSCHEDULER_NOT_STARTED)
    {
        return 1U;
    }

    /*
     * 正常业务只允许在调度器运行状态访问Flash。
     */
    if ((scheduler_state != taskSCHEDULER_RUNNING) ||
        (g_flash_log_mutex_handle == 0))
    {
        return 0U;
    }

    /*
     * 如果其他任务正在操作Flash，当前任务进入阻塞，
     * 等对方释放Mutex后再继续，不使用空循环抢锁。
     */
    if (xSemaphoreTake(
            g_flash_log_mutex_handle,
            portMAX_DELAY) != pdTRUE)
    {
        return 0U;
    }

    return 1U;
}


/*
 * 释放Flash日志互斥锁。
 */
static void FlashLog_Unlock(void)
{
    /*
     * 调度器启动前，FlashLog_Lock没有真正取得Mutex，
     * 因此这里也不执行释放操作。
     */
    if (xTaskGetSchedulerState() ==
        taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    if (g_flash_log_mutex_handle != 0)
    {
        (void)xSemaphoreGive(
            g_flash_log_mutex_handle);
    }
}

/*
 * 判断一整条32字节位置是否仍处于擦除状态。
 *
 * 不能只检查Byte0：
 * 如果上次写Flash时突然掉电，Byte0可能仍为0xFF，
 * 但后面的某些字节可能已经被写成0。
 * 这种位置不能再次覆盖写入。
 */
static uint8_t FlashLog_RecordIsErased(const uint8_t *record)
{
    uint8_t i;

    if (record == 0)
    {
        return 0U;
    }

    for (i = 0U; i < FLASH_LOG_RECORD_SIZE; i++)
    {
        if (record[i] != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}


/*
 * 回读整个V2扇区，确认每个字节都已经恢复为0xFF。
 * 每次只使用32字节局部数组，避免占用4KB栈空间。
 */
static uint8_t FlashLog_SectorIsErased(void)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint16_t index;
    uint32_t address;

    for (index = 0U; index < FLASH_LOG_MAX_RECORDS; index++)
    {
        address =
            FLASH_LOG_V2_SECTOR_ADDR +
            ((uint32_t)index * FLASH_LOG_RECORD_SIZE);

        XM25QH32_ReadData(address,
                         record,
                         FLASH_LOG_RECORD_SIZE);

        if (FlashLog_RecordIsErased(record) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}


/*
 * 检查一条已经占用的记录是否有效。
 *
 * 检查顺序：
 * 1. 固定标记是否为0xA5；
 * 2. 版本是否为V2；
 * 3. Byte0～Byte30计算出的CRC是否等于Byte31。
 */
static uint8_t FlashLog_RecordIsValid(const uint8_t *record)
{
    uint8_t calculated_crc;

    if (record == 0)
    {
        return 0U;
    }

    if (record[0] != FLASH_LOG_V2_MAGIC)
    {
        return 0U;
    }

    if (record[1] != FLASH_LOG_V2_VERSION)
    {
        return 0U;
    }

    calculated_crc =
        BMS_CalculateCRC8(record, FLASH_LOG_V2_CRC_OFFSET);

    if (calculated_crc != record[FLASH_LOG_V2_CRC_OFFSET])
    {
        return 0U;
    }

    return 1U;
}


/*
 * 读取RCC复位标志，判断本次程序为什么启动。
 *
 * 多个标志可能同时为1，因此按故障严重性确定优先级：
 * 看门狗 > 软件复位 > 低功耗复位 > 上电复位 > 复位引脚。
 */
static FlashLogResetReason_t FlashLog_DetectResetReason(void)
{
    uint32_t csr;
    FlashLogResetReason_t reason;

    csr = RCC->CSR;
    reason = FLASH_LOG_RESET_UNKNOWN;

    if ((csr & RCC_CSR_IWDGRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_IWDG;
    }
    else if ((csr & RCC_CSR_WWDGRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_WWDG;
    }
    else if ((csr & RCC_CSR_SFTRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_SOFTWARE;
    }
    else if ((csr & RCC_CSR_LPWRRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_LOW_POWER;
    }
    else if ((csr & RCC_CSR_PORRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_POWER_ON;
    }
    else if ((csr & RCC_CSR_PINRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_PIN;
    }

    /*
     * 保存原因后清除硬件复位标志。
     * 否则下次软件复位时可能同时看到上一次的旧标志。
     */
    RCC->CSR |= RCC_CSR_RMVF;

    return reason;
}


/*
 * 以高字节在前的顺序写入一个16位数。
 *
 * 例如0x0201：
 * record[offset]     = 0x02
 * record[offset + 1] = 0x01
 */
static void FlashLog_WriteU16(uint8_t *record,
                              uint8_t offset,
                              uint16_t value)
{
    record[offset] = (uint8_t)(value >> 8);
    record[offset + 1U] = (uint8_t)value;
}


/*
 * 以高字节在前的顺序写入一个32位数。
 *
 * 例如时间戳0x0001E816：
 * 依次写入00、01、E8、16。
 */
static void FlashLog_WriteU32(uint8_t *record,
                              uint8_t offset,
                              uint32_t value)
{
    record[offset] = (uint8_t)(value >> 24);
    record[offset + 1U] = (uint8_t)(value >> 16);
    record[offset + 2U] = (uint8_t)(value >> 8);
    record[offset + 3U] = (uint8_t)value;
}


/*
 * 从记录中读取一个高字节在前的16位数。
 *
 * 例如两个字节为0x12、0x34：
 * 返回的16位结果就是0x1234。
 */
static uint16_t FlashLog_ReadU16(const uint8_t *record,
                                 uint8_t offset)
{
    uint16_t value;

    value = (uint16_t)record[offset] << 8;
    value |= (uint16_t)record[offset + 1U];

    return value;
}


/*
 * 从记录中读取一个高字节在前的32位数。
 *
 * 例如四个字节为00、00、4A、94：
 * 返回结果为0x00004A94，也就是十进制19092。
 */
static uint32_t FlashLog_ReadU32(const uint8_t *record,
                                 uint8_t offset)
{
    uint32_t value;

    value = (uint32_t)record[offset] << 24;
    value |= (uint32_t)record[offset + 1U] << 16;
    value |= (uint32_t)record[offset + 2U] << 8;
    value |= (uint32_t)record[offset + 3U];

    return value;
}


/*
 * 使用当前项目的简化线性方法从电池电压估算SOC。
 *
 * 3.0V及以下为0%；
 * 4.2V及以上为100%；
 * 中间按照线性比例计算。
 */
static uint8_t FlashLog_CalculateSoc(uint16_t cell_voltage_mv)
{
    uint32_t soc;

    if (cell_voltage_mv <= 3000U)
    {
        return 0U;
    }

    if (cell_voltage_mv >= 4200U)
    {
        return 100U;
    }

    /*
     * 加600相当于对除以1200的结果进行四舍五入。
     * 全程使用整数，Flash日志不需要再次进行浮点计算。
     */
    soc =
        (((uint32_t)cell_voltage_mv - 3000U) * 100U + 600U) /
        1200U;

    return (uint8_t)soc;
}


/*
 * 根据事件发生后仍然存在的活动故障，记录CAN状态。
 *
 * Bus-Off优先级最高；
 * 没有Bus-Off但存在通信降级时记录DEGRADED；
 * 两者都不存在时记录OK。
 */
static FlashLogCanState_t FlashLog_GetCanState(
    const FaultEvent_t *event)
{
    if ((event->active_mask & FAULT_MASK_CAN_BUS_OFF) != 0U)
    {
        return FLASH_LOG_CAN_BUS_OFF;
    }

    if ((event->active_mask & FAULT_MASK_CAN_DEGRADED) != 0U)
    {
        return FLASH_LOG_CAN_DEGRADED;
    }

    return FLASH_LOG_CAN_OK;
}


/* 比较写入数据和Flash回读数据是否完全相同。 */
static uint8_t FlashLog_RecordEquals(const uint8_t *expected,
                                     const uint8_t *actual)
{
    uint8_t i;

    for (i = 0U; i < FLASH_LOG_RECORD_SIZE; i++)
    {
        if (expected[i] != actual[i])
        {
            return 0U;
        }
    }

    return 1U;
}


/* 诊断计数达到最大值后保持，避免长期运行后回绕为0。 */
static void FlashLog_IncrementSaturated(
    volatile uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}


/* 把记录中的事件编号转换成串口上容易理解的名称。 */
static const char *FlashLog_GetEventName(uint8_t event_type)
{
    if (event_type == FAULT_EVENT_CONFIRMED)
    {
        return "CONFIRMED";
    }

    if (event_type == FAULT_EVENT_CLEARED)
    {
        return "CLEARED";
    }

    if (event_type == FAULT_EVENT_SYSTEM_BOOT)
    {
        return "BOOT";
    }

    return "UNKNOWN";
}


/*
 * 根据事件类型和注入标志说明记录来源：
 * 启动来自系统，清除来自恢复流程，确认事件来自真实检测或CLI注入。
 */
static const char *FlashLog_GetSourceName(uint8_t event_type,
                                          uint8_t valid_flags)
{
    if (event_type == FAULT_EVENT_SYSTEM_BOOT)
    {
        return "SYSTEM";
    }

    if (event_type == FAULT_EVENT_CLEARED)
    {
        return "RECOVERY";
    }

    if ((valid_flags & FAULT_EVENT_FLAG_INJECTED) != 0U)
    {
        return "INJECTED";
    }

    return "REAL";
}

/*
 * Flash V2初始化。
 *
 * 每个位置分为三种情况：
 *
 * 全部为0xFF：
 *     这是第一个真正空白的位置，可以作为下一写入位置。
 *
 * 记录有效：
 *     说明这里已有历史日志，继续扫描下一条。
 *
 * 已占用但无效：
 *     可能是CRC损坏或掉电造成的半条记录。
 *     统计错误并跳过，不能覆盖它。
 */
void FlashLog_Init(void)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint16_t index;
    uint32_t address;

    /*
    * 第一次初始化时创建静态Mutex。
    * 再次调用初始化函数时不重复创建。
    */
    if (g_flash_log_mutex_handle == 0)
    {
        g_flash_log_mutex_handle =
            xSemaphoreCreateMutexStatic(
                &g_flash_log_mutex_control);
    }

    g_flash_log_next_index = FLASH_LOG_MAX_RECORDS;
    g_flash_log_write_ok_count = 0U;
    g_flash_log_write_fail_count = 0U;
    g_flash_log_crc_error_count = 0U;
    g_flash_log_erase_fail_count = 0U;

    g_flash_log_reset_reason = FlashLog_DetectResetReason();

    for (index = 0U; index < FLASH_LOG_MAX_RECORDS; index++)
    {
        address =
            FLASH_LOG_V2_SECTOR_ADDR +
            ((uint32_t)index * FLASH_LOG_RECORD_SIZE);

        XM25QH32_ReadData(
            address,
            record,
            FLASH_LOG_RECORD_SIZE);

        if (FlashLog_RecordIsErased(record) != 0U)
        {
            g_flash_log_next_index = index;
            break;
        }

        if (FlashLog_RecordIsValid(record) == 0U)
        {
            g_flash_log_crc_error_count++;
        }
    }

    printf("[LOG V2] Ready Index=%u/%u Reset=%u Invalid=%lu\r\n",
           (unsigned int)g_flash_log_next_index,
           (unsigned int)FLASH_LOG_MAX_RECORDS,
           (unsigned int)g_flash_log_reset_reason,
           (unsigned long)g_flash_log_crc_error_count);
}

/*
 * 把一条FaultEvent编码为固定32字节并写入外部Flash。
 *
 * 执行过程：
 * 检查事件
 *   ↓
 * 必要时擦除已写满的扇区
 *   ↓
 * 逐字段组装32字节
 *   ↓
 * 计算Byte0～Byte30的CRC8
 *   ↓
 * Page Program写入Flash
 *   ↓
 * 立即回读32字节
 *   ↓
 * 逐字节比较并更新诊断计数
 * 
 * 已持有Flash Mutex时执行的实际写入逻辑。
 * 调用者不能直接调用这个内部函数。
 */
static uint8_t FlashLog_WriteEventUnlocked(
    const FaultEvent_t *event)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint8_t verify_record[FLASH_LOG_RECORD_SIZE];
    uint8_t i;
    uint8_t soc;
    uint16_t record_index;
    uint32_t address;

    /*
     * 空指针或非法事件类型不写入。
     * 故障确认/清除必须包含变化位；系统启动没有故障变化，允许为0。
     */
    if ((event == 0) ||
        ((event->event_type != FAULT_EVENT_CONFIRMED) &&
         (event->event_type != FAULT_EVENT_CLEARED) &&
         (event->event_type != FAULT_EVENT_SYSTEM_BOOT)) ||
        (((event->event_type == FAULT_EVENT_CONFIRMED) ||
          (event->event_type == FAULT_EVENT_CLEARED)) &&
         (event->changed_mask == 0U)))
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_write_fail_count);

        return 0U;
    }

    /*
     * 128条记录全部占用后，擦除整个V2扇区。
     * 旧V1日志所在的0号扇区不会受到影响。
     */
    if (g_flash_log_next_index >= FLASH_LOG_MAX_RECORDS)
    {
        XM25QH32_SectorErase(FLASH_LOG_V2_SECTOR_ADDR);

        /*
         * 擦除未通过回读验证时不能把写指针归零，
         * 否则会在仍有旧数据的位置上继续编程并破坏记录。
         */
        if (FlashLog_SectorIsErased() == 0U)
        {
            FlashLog_IncrementSaturated(
                &g_flash_log_erase_fail_count);
            FlashLog_IncrementSaturated(
                &g_flash_log_write_fail_count);

            printf("[LOG V2] Full-sector erase verify failed\r\n");
            return 0U;
        }

        g_flash_log_next_index = 0U;
        g_flash_log_crc_error_count = 0U;
    }

    record_index = g_flash_log_next_index;

    /*
     * 先把整条记录清零。
     * Byte22～Byte30没有单独赋值，因此自然作为保留字段保存0。
     */
    for (i = 0U; i < FLASH_LOG_RECORD_SIZE; i++)
    {
        record[i] = 0U;
    }

    record[FLASH_LOG_OFFSET_MAGIC] =
        FLASH_LOG_V2_MAGIC;

    record[FLASH_LOG_OFFSET_VERSION] =
        FLASH_LOG_V2_VERSION;

    record[FLASH_LOG_OFFSET_EVENT_TYPE] =
        event->event_type;

    record[FLASH_LOG_OFFSET_VALID_FLAGS] =
        event->valid_flags;

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_SEQUENCE,
        record_index);

    FlashLog_WriteU32(
        record,
        FLASH_LOG_OFFSET_TIMESTAMP,
        event->timestamp_ms);

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_CHANGED_MASK,
        event->changed_mask);

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_ACTIVE_MASK,
        event->active_mask);

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_VOLTAGE_MV,
        event->cell_voltage_mv);

    /*
     * int16_t温度转换为uint16_t后，底层二进制位保持不变。
     * 例如-5.0℃保存为-50，对应0xFFCE。
     */
    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_TEMPERATURE,
        (uint16_t)event->temperature_dC);

    /*
     * 电压有效时计算SOC；
     * 电压无效时写0xFF，明确表示SOC不可用。
     */
    if ((event->valid_flags &
         FAULT_EVENT_VALID_VOLTAGE) != 0U)
    {
        soc = FlashLog_CalculateSoc(
            event->cell_voltage_mv);
    }
    else
    {
        soc = 0xFFU;
    }

    record[FLASH_LOG_OFFSET_SOC] = soc;

    record[FLASH_LOG_OFFSET_BMS_STATE] =
        event->bms_state;

    record[FLASH_LOG_OFFSET_CAN_STATE] =
        (uint8_t)FlashLog_GetCanState(event);

    record[FLASH_LOG_OFFSET_RESET_REASON] =
        (uint8_t)g_flash_log_reset_reason;

    /*
     * 最后计算CRC。
     * CRC覆盖Byte0～Byte30，CRC自身存放在Byte31。
     */
    record[FLASH_LOG_V2_CRC_OFFSET] =
        BMS_CalculateCRC8(
            record,
            FLASH_LOG_V2_CRC_OFFSET);

    address =
        FLASH_LOG_V2_SECTOR_ADDR +
        ((uint32_t)record_index *
         FLASH_LOG_RECORD_SIZE);

    XM25QH32_PageProgram(
        address,
        record,
        FLASH_LOG_RECORD_SIZE);

    /*
     * 写完立即从Flash读回来。
     * API没有提供硬件错误返回值，所以回读比较是实际确认手段。
     */
    XM25QH32_ReadData(
        address,
        verify_record,
        FLASH_LOG_RECORD_SIZE);

    if ((FlashLog_RecordEquals(record, verify_record) == 0U) ||
        (FlashLog_RecordIsValid(verify_record) == 0U))
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_write_fail_count);

        /*
         * 失败位置可能已经有部分位被写成0，不能再次覆盖。
         * 下一次改用后一个32字节位置。
         */
        g_flash_log_next_index++;

        printf("[LOG V2] Write verify failed at #%u\r\n",
               (unsigned int)record_index);

        return 0U;
    }

    g_flash_log_next_index++;

    FlashLog_IncrementSaturated(
        &g_flash_log_write_ok_count);

    printf("[LOG V2] Write #%u Event=%u Changed=0x%04X Active=0x%04X\r\n",
           (unsigned int)record_index,
           (unsigned int)event->event_type,
           (unsigned int)event->changed_mask,
           (unsigned int)event->active_mask);

    return 1U;
}


/*
 * 线程安全的故障日志写入公共接口。
 *
 * 所有运行阶段的调用者都必须经过这里，
 * 不能直接调用Unlocked内部实现。
 * 
 * 公开函数取得Mutex
        ↓
 * 调用内部实际写入函数
        ↓
 * 内部函数无论成功还是失败都返回公开函数
        ↓
 * 公开函数统一释放Mutex
 */
uint8_t FlashLog_WriteEvent(
    const FaultEvent_t *event)
{
    uint8_t result;

    if (FlashLog_Lock() == 0U)
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_write_fail_count);

        return 0U;
    }

    result =
        FlashLog_WriteEventUnlocked(event);

    FlashLog_Unlock();

    return result;
}

/*
 * 在系统初始化阶段生成一条启动记录。
 * 此时TIM3尚未启动，也没有可信的传感器快照，因此时间和有效标志为0；
 * FlashLog_WriteEvent会把FlashLog_Init检测到的复位原因写入记录。
 */
uint8_t FlashLog_WriteBootEvent(void)
{
    FaultEvent_t event;

    event.timestamp_ms = 0U;
    event.changed_mask = 0U;
    event.active_mask = 0U;
    event.cell_voltage_mv = 0U;
    event.temperature_dC = 0;
    event.event_type = (uint8_t)FAULT_EVENT_SYSTEM_BOOT;
    event.bms_state = 0U;
    event.valid_flags = 0U;

    return FlashLog_WriteEvent(&event);
}


/*
 * 逐条读取并打印Flash V2中的历史系统和故障记录。
 *
 * 每次只使用一个32字节数组，避免在STM32的小容量栈中
 * 创建一个完整的4KB扇区缓存。
 */
static void FlashLog_PrintAllUnlocked(void)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint8_t valid_flags;
    uint8_t event_type;
    uint8_t soc;
    uint16_t index;
    uint16_t sequence;
    uint16_t changed_mask;
    uint16_t active_mask;
    uint16_t voltage_mv;
    int16_t temperature_dC;
    int32_t temperature_value;
    uint32_t timestamp_ms;
    uint32_t address;

    printf("\r\n[LOG V2] Dump begin\r\n");

    for (index = 0U; index < FLASH_LOG_MAX_RECORDS; index++)
    {
        address =
            FLASH_LOG_V2_SECTOR_ADDR +
            ((uint32_t)index * FLASH_LOG_RECORD_SIZE);

        XM25QH32_ReadData(address,
                         record,
                         FLASH_LOG_RECORD_SIZE);

        /*
         * 遇到第一条完全空白的记录，说明后面还没有写入，
         * 因此结束本次读取。
         */
        if (FlashLog_RecordIsErased(record) != 0U)
        {
            break;
        }

        /*
         * 记录已经占用，但标记、版本或CRC错误。
         * 打印错误并跳过，继续检查后面的记录。
         */
        if (FlashLog_RecordIsValid(record) == 0U)
        {
            printf("[LOG V2] Slot=%u INVALID\r\n",
                   (unsigned int)index);
            continue;
        }

        event_type =
            record[FLASH_LOG_OFFSET_EVENT_TYPE];

        valid_flags =
            record[FLASH_LOG_OFFSET_VALID_FLAGS];

        sequence =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_SEQUENCE);

        timestamp_ms =
            FlashLog_ReadU32(record,
                             FLASH_LOG_OFFSET_TIMESTAMP);

        changed_mask =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_CHANGED_MASK);

        active_mask =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_ACTIVE_MASK);

        voltage_mv =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_VOLTAGE_MV);

        temperature_dC =
            (int16_t)FlashLog_ReadU16(
                record,
                FLASH_LOG_OFFSET_TEMPERATURE);

        soc = record[FLASH_LOG_OFFSET_SOC];

        printf("[LOG V2] #%u %s Source=%s Time=%lums Changed=0x%04X Active=0x%04X\r\n",
               (unsigned int)sequence,
               FlashLog_GetEventName(event_type),
               FlashLog_GetSourceName(event_type, valid_flags),
               (unsigned long)timestamp_ms,
               (unsigned int)changed_mask,
               (unsigned int)active_mask);

        if ((valid_flags & FAULT_EVENT_VALID_VOLTAGE) != 0U)
        {
            printf("         Voltage=%umV SOC=%u%%\r\n",
                   (unsigned int)voltage_mv,
                   (unsigned int)soc);
        }
        else
        {
            printf("         Voltage=INVALID SOC=INVALID\r\n");
        }

        if ((valid_flags & FAULT_EVENT_VALID_TEMPERATURE) != 0U)
        {
            /*
             * 先提升为32位，再处理负数。
             * 这样即使温度数据接近int16_t下限也不会溢出。
             */
            temperature_value = (int32_t)temperature_dC;

            if (temperature_value < 0)
            {
                temperature_value = -temperature_value;

                printf("         Temperature=-%ld.%ldC",
                       (long)(temperature_value / 10),
                       (long)(temperature_value % 10));
            }
            else
            {
                printf("         Temperature=%ld.%ldC",
                       (long)(temperature_value / 10),
                       (long)(temperature_value % 10));
            }
        }
        else
        {
            printf("         Temperature=INVALID");
        }

        printf(" BMS=%u CAN=%u Reset=%u\r\n",
               (unsigned int)record[FLASH_LOG_OFFSET_BMS_STATE],
               (unsigned int)record[FLASH_LOG_OFFSET_CAN_STATE],
               (unsigned int)record[FLASH_LOG_OFFSET_RESET_REASON]);
    }

    printf("[LOG V2] Dump end. Occupied=%u\r\n\r\n",
           (unsigned int)index);
}


/*
 * 线程安全的Flash历史日志打印接口。
 *
 * 打印期间保持Mutex，使本次输出看到的是一致的日志内容，
 * FaultLogTask产生的新写入会暂时在RAM事件队列中等待。
 */
void FlashLog_PrintAll(void)
{
    if (FlashLog_Lock() == 0U)
    {
        printf("[LOG V2] Flash mutex unavailable\r\n");
        return;
    }

    FlashLog_PrintAllUnlocked();

    FlashLog_Unlock();
}


/*
 * 清除Flash V2故障日志。
 *
 * 只擦除V2使用的第二个4KB扇区，
 * 不影响第一个扇区中保留的V1旧日志。
 *
 * 该函数只能由明确的用户操作调用，
 * 不能放进周期任务或系统启动流程。
 */
static uint8_t FlashLog_ClearUnlocked(void)
{
    /*
     * 擦除0x001000所在的整个4KB扇区。
     * 底层驱动会等待Flash结束忙状态后才返回。
     */
    XM25QH32_SectorErase(FLASH_LOG_V2_SECTOR_ADDR);

    /*
     * 必须回读整个扇区确认擦除结果。
     * 失败时保留原来的next_index，防止从0号位置覆盖旧数据。
     */
    if (FlashLog_SectorIsErased() == 0U)
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_erase_fail_count);

        printf("[LOG V2] Clear verify failed. Index remains %u\r\n",
               (unsigned int)g_flash_log_next_index);
        return 0U;
    }

    /*
     * 扇区擦除后，第0号位置重新成为下一写入位置。
     */
    g_flash_log_next_index = 0U;

    /*
     * 清除本轮日志操作的诊断计数。
     * 复位原因不清零，因为本次启动原因仍然有效。
     */
    g_flash_log_write_ok_count = 0U;
    g_flash_log_write_fail_count = 0U;
    g_flash_log_crc_error_count = 0U;
    g_flash_log_erase_fail_count = 0U;

    printf("[LOG V2] Cleared. Next Index=0\r\n");
    return 1U;
}

/*
 * 线程安全的日志清空公共接口。
 *
 * 擦除和回读验证期间保持Mutex，
 * 防止FaultLogTask同时向正在擦除的扇区写入记录。
 */
uint8_t FlashLog_Clear(void)
{
    uint8_t result;

    if (FlashLog_Lock() == 0U)
    {
        return 0U;
    }

    result =
        FlashLog_ClearUnlocked();

    FlashLog_Unlock();

    return result;
}
