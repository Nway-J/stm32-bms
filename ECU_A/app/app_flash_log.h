#ifndef __APP_FLASH_LOG_H
#define __APP_FLASH_LOG_H

#include "stm32f10x.h"

// ─────────────────────────────────────────────
// FlashLog_Init — 故障日志初始化
// 扫描Flash日志区，定位最后一条记录的位置
// 在main初始化时调用一次
// ─────────────────────────────────────────────
void FlashLog_Init(void);

// ─────────────────────────────────────────────
// FlashLog_Write — 写入一条故障记录
// 参数 fault_code：故障码(0x01过压/0x02欠压/0x04过温)
// 参数 cell_mv：  故障时Cell电压(mV)
// 参数 temp：     故障时温度(℃)
// 参数 soc：      故障时SOC(%)
// 每条记录8字节，满512条后擦除Sector从头写
// ─────────────────────────────────────────────
void FlashLog_Write(uint8_t fault_code, uint16_t cell_mv, uint8_t temp, uint8_t soc);

// ─────────────────────────────────────────────
// FlashLog_ReadAll — 读取所有故障记录到缓冲区
// 参数 buf：     存放记录的缓冲区
// 参数 max_len： 缓冲区最大字节数
// 返回：实际读取的记录数
// ─────────────────────────────────────────────
uint16_t FlashLog_ReadAll(uint8_t *buf, uint16_t max_len);

// ─────────────────────────────────────────────
// FlashLog_PrintAll — 串口打印所有故障记录
// 通过USART输出故障历史，每条记录一行
// 格式：#001 Code=0x01 Cell=394mV T=28C SOC=82%
// ─────────────────────────────────────────────
void FlashLog_PrintAll(void);

// ─────────────────────────────────────────────
// FlashLog_Clear — 清除所有故障记录
// 擦除整个日志Sector，索引归零
// ─────────────────────────────────────────────
void FlashLog_Clear(void);

#endif
