/*
 * 模块：XM25QH32外部SPI Flash板级接口。
 * 作用：提供设备识别、状态读取、页编程、扇区擦除和数据读取接口。
 * 边界：本模块只处理Flash硬件协议，不解释故障日志的数据格式。
 */
#ifndef __BSP_SPI_FLASH_H
#define __BSP_SPI_FLASH_H

#include "stm32f10x.h"

#define SPI_FLASH_CS_LOW()   GPIOA->BRR = (1 << 4)
#define SPI_FLASH_CS_HIGH()  GPIOA->BSRR = (1 << 4)
#define SPI_FLASH_SECTOR_SIZE     4096

//SPI初始化
void SPI1_Init(void);

//SPI收发数据
uint8_t SPI1_SwapByte(uint8_t tx_data);

//读FLASH寄存器
uint8_t XM25QH32_ReadStatusReg(void);

//FLASH忙标志
void XM25QH32_WaitBusy(void);

//FLASH写使能
void XM25QH32_WriteEnable(void);

//读FLASH--ID
uint16_t XM25QH32_ReadID(void);

//读FLASH数据
void XM25QH32_ReadData(uint32_t addr, uint8_t *buffer, uint16_t len);

//页编程
void XM25QH32_PageProgram(uint32_t addr, uint8_t *buffer, uint16_t len);

//扇区擦除
void XM25QH32_SectorErase(uint32_t addr);

//写数据--已确认Flash区域是擦除状态（全0xFF），直接写
void XM25QH32_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint16_t len);

//FLASH写数据--不确定Flash状态，自动读扇区→判断→擦除→写回
void XM25QH32_Write(uint8_t *buffer,uint32_t addr,uint16_t len);

#endif

