// ═══════════════════════════════════════════════════════════════
// app_flash_log.c — 故障日志系统（SPI Flash存储）
//
// 核心设计（规避Flash写入0→1的限制）：
//   1. 索引不存Flash。FLASH只能1→0，索引反复写同一地址会损坏
//   2. 启动时从LOG_SECTOR_ADDR向后扫描，找到第一条0xFF作为写入位置
//   3. 每条记录8字节，满512条后擦除整个Sector从头开始
//
// 存储布局：
//   Sector0 (0x000000 ~ 0x000FFF) — 故障日志区
//     记录从0x000000开始顺序写入，写满512条后擦除重来
//     不单独划分索引区，索引在内存维护
//
// 编译时检查：确保日志不超过扇区大小
// ═══════════════════════════════════════════════════════════════

#include "app_flash_log.h"
#include "bsp_spi_flash.h"
#include <stdio.h>

// ─── 存储区域定义 ───
#define LOG_SECTOR_ADDR 0x000000 // 日志所在Sector（F103C8T6的Flash扇区为4096字节）
#define LOG_RECORD_SIZE 8        // 每条记录8字节
#define LOG_MAX_RECORDS 512      // 4096 / 8 = 512条

// 编译时断言：检查日志是否超过扇区大小
// 如果超出，编译会报错 "size constraint 'Log area overflow' violated"
// 去掉注释启用（需要C11编译器支持）

// ─── 当前写索引（只在内存中维护，不存Flash）───
static uint16_t g_log_index = 0;

// ─────────────────────────────────────────────
// FlashLog_Init — 故障日志初始化
// 从LOG_SECTOR_ADDR开始向后扫描，找到第一条Byte0=0xFF的位置
// 该位置就是下一条记录的写入地址
//
// 扫描方法：
//   依次读取每条记录的Byte0（故障码）
//   如果Byte0==0xFF（Flash擦除状态），说明该位置未写入
//   当前位置即为g_log_index
//
// 如果扫描完整个扇区都没找到0xFF，说明日志已写满
// 此时g_log_index = LOG_MAX_RECORDS，下次写入会触发擦除
// ─────────────────────────────────────────────
void FlashLog_Init(void)
{
    uint8_t byte;
    uint16_t i;

    g_log_index = 0;

    // 从前往后扫描，找到第一条Byte0=0xFF的记录位置
    for (i = 0; i < LOG_MAX_RECORDS; i++)
    {
        XM25QH32_ReadData(LOG_SECTOR_ADDR + i * LOG_RECORD_SIZE, &byte, 1);
        if (byte == 0xFF) // 未写入的位置
        {
            g_log_index = i;
            break;
        }
    }

    // 如果全满，g_log_index停在LOG_MAX_RECORDS
    // 下次写入会触发擦除
    if (i == LOG_MAX_RECORDS)
        g_log_index = LOG_MAX_RECORDS;

    printf("[LOG] Fault log ready. Index=%d/%d\r\n", g_log_index, LOG_MAX_RECORDS);
}

// ─────────────────────────────────────────────
// FlashLog_Write — 写入一条故障记录
//
// 参数 fault_code：故障码（位掩码：0x01过压/0x02欠压/0x04过温）
// 参数 cell_mv：  故障时刻的Cell电压(mV)
// 参数 temp：     故障时刻的温度(℃)
// 参数 soc：      故障时刻的SOC(%)
//
// 记录格式（8字节）：
//   Byte0:   故障码
//   Byte1~2: Cell电压(mV, 大端)
//   Byte3~4: 保留(填0)
//   Byte5:   温度(℃)
//   Byte6:   SOC(%)
//   Byte7:   记录序号低8位
//
// 写入前判断：如果g_log_index >= LOG_MAX_RECORDS
// 说明日志已写满，先擦除整个Sector再写
// 擦除后g_log_index归零
//
// 索引只在内存维护，不存Flash
// ─────────────────────────────────────────────
void FlashLog_Write(uint8_t fault_code, uint16_t cell_mv, uint8_t temp, uint8_t soc)
{
    uint8_t record[LOG_RECORD_SIZE];
    uint32_t addr;

    // 如果日志已写满，擦除整个Sector重新开始
    if (g_log_index >= LOG_MAX_RECORDS)
    {
        printf("[LOG] Log full (%d records), erasing sector 0x%06X...\r\n", LOG_MAX_RECORDS,
               LOG_SECTOR_ADDR);
        XM25QH32_SectorErase(LOG_SECTOR_ADDR);
        g_log_index = 0;
    }

    // 计算本次写入的Flash地址
    addr = LOG_SECTOR_ADDR + g_log_index * LOG_RECORD_SIZE;

    // 组装记录
    record[0] = fault_code;            // 故障码
    record[1] = (cell_mv >> 8) & 0xFF; // Cell电压高8位
    record[2] = cell_mv & 0xFF;        // Cell电压低8位
    record[3] = 0;                     // 保留
    record[4] = 0;                     // 保留
    record[5] = temp;                  // 温度
    record[6] = soc;                   // SOC
    record[7] = g_log_index & 0xFF;    // 序号低8位

    // 写入记录到Flash
    // 擦除后Flash全为0xFF，PageProgram可以直接写
    // 不需要Write_NoCheck的自动拆页，因为8字节不跨页
    XM25QH32_PageProgram(addr, record, LOG_RECORD_SIZE);

    // 索引递增（只在内存）
    g_log_index++;

    printf("[LOG] Fault #%d logged: Code=0x%02X\r\n", g_log_index - 1, fault_code);
}

// ─────────────────────────────────────────────
// FlashLog_ReadAll — 读取所有故障记录到缓冲区
//
// 参数 buf：     存放记录的缓冲区
// 参数 max_len： 缓冲区大小（字节）
//
// 返回：读取到的记录数
//
// 注意：g_log_index==0时有两种情况
//   1. 刚擦除完，没有记录 → 读第一条如果是0xFF，返回0
//   2. 刚写满一圈归零，有记录 → 读LOG_MAX_RECORDS条
// ─────────────────────────────────────────────
uint16_t FlashLog_ReadAll(uint8_t *buf, uint16_t max_len)
{
    uint16_t count;

    // g_log_index==0时，需要判断是否有记录
    if (g_log_index == 0)
    {
        uint8_t test;
        XM25QH32_ReadData(LOG_SECTOR_ADDR, &test, 1);
        if (test == 0xFF)
            return 0;            // 扇区已擦除，无记录
        count = LOG_MAX_RECORDS; // 索引回绕到0，说明一整圈记录都有效
    }
    else
    {
        count = g_log_index;
    }

    // 计算要读取的字节数
    uint16_t read_bytes = count * LOG_RECORD_SIZE;
    if (read_bytes > max_len)
        read_bytes = max_len;

    // 从Flash读取
    XM25QH32_ReadData(LOG_SECTOR_ADDR, buf, read_bytes);

    return count;
}

// ─────────────────────────────────────────────
// FlashLog_PrintAll — 串口打印所有故障记录
//
// 输出格式：
//   ==== FAULT HISTORY (N records) ====
//   #000: Code=0x01 Cell=394mV T=28C SOC=82%
//   #001: Code=0x02 Cell=312mV T=25C SOC=45%
//   ...
// ─────────────────────────────────────────────
void FlashLog_PrintAll(void)
{
    uint8_t buf[LOG_MAX_RECORDS * LOG_RECORD_SIZE];
    uint16_t count = FlashLog_ReadAll(buf, sizeof(buf));

    if (count == 0)
    {
        printf("[LOG] No records.\r\n");
        return;
    }

    printf("\r\n==== FAULT HISTORY (%d records) ====\r\n", count);

    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t *rec = buf + i * LOG_RECORD_SIZE;
        uint8_t fc = rec[0];                            // 故障码
        uint16_t v1 = ((uint16_t)rec[1] << 8) | rec[2]; // Cell电压
        uint8_t t = rec[5];                             // 温度
        uint8_t s = rec[6];                             // SOC

        printf("#%03d: Code=0x%02X Cell=%dmV T=%dC SOC=%d%%\r\n", i, fc, v1, t, s);
    }
}

// ─────────────────────────────────────────────
// FlashLog_Clear — 清除所有故障记录
//
// 索引归零，擦除整个日志Sector
// 擦除后所有字节变为0xFF，FlashLog_Init扫描时会从0开始
// ─────────────────────────────────────────────
void FlashLog_Clear(void)
{
    g_log_index = 0;

    // 擦除整个日志Sector
    // 日志记录从LOG_SECTOR_ADDR(0x000000)开始
    // 一个扇区4096字节，索引和记录在同一扇区内
    // 擦除后全部为0xFF
    XM25QH32_SectorErase(LOG_SECTOR_ADDR);

    printf("[LOG] All fault records cleared.\r\n");
}
