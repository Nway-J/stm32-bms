/*
 * 模块：XM25QH32外部SPI Flash驱动。
 * 连接：SPI1使用PA4-CS、PA5-SCK、PA6-MISO、PA7-MOSI。
 * 时序：当前SPI时钟为72MHz/64=1.125MHz，给杜邦线连接保留信号裕量。
 * 功能：实现状态读取、设备ID、普通读取、页编程和4KB扇区擦除。
 * 边界：本模块不识别FlashLog记录，也不决定日志写入策略。
 */
#include "stm32f10x.h"
#include "bsp_spi_flash.h"

/*
 * 整扇区改写接口使用的4KB缓存只属于本驱动。
 * 放在.c文件可避免每个包含头文件的模块各自生成一份静态数组。
 */
static uint8_t SectorBuffer[SPI_FLASH_SECTOR_SIZE];

void SPI1_Init(void)
{
	//1.开GPIOA/SPI1时钟
	RCC->APB2ENR |= (1U<<12);
	RCC->APB2ENR |= (1U<<2);
	
	//2.配置PA4-CS通用推挽 PA5-SCK复用推挽 
	//PA6-MISO浮空输入 PA7-MOSI复用推挽
	//先把CS输出锁存位置高，避免PA4切换为输出时产生低脉冲。
	GPIOA->BSRR = (1U << 4);
	GPIOA->CRL &= ~(0xFU << 16); 
	GPIOA->CRL |= (0x2U << 16); // PA4：2MHz通用推挽，CS只需低速切换
    GPIOA->CRL &= ~(0xFU << 20); 
	GPIOA->CRL |= (0x9U << 20); // PA5：10MHz复用推挽，降低SCK边沿振铃
    GPIOA->CRL &= ~(0xFU << 24); 
	GPIOA->CRL |= (0x4U << 24);
    GPIOA->CRL &= ~(0xFU << 28); 
	GPIOA->CRL |= (0x9U << 28); // PA7：10MHz复用推挽，降低MOSI边沿振铃
	
	//3.CS片选，主机不选中
	GPIOA->BSRR = (1U<<4);
	
	//4.配置SPI1,模式1
	SPI1->CR1 = 0;
	SPI1->CR1 &= ~(1<<1);//CPOL时钟极性
	SPI1->CR1 &= ~(1<<0);//CPHA时钟相位
	
	SPI1->CR1 |= (1<<2);//MSTR主设备选择
	/*
	 * BR=101：PCLK2六十四分频，SPI时钟为1.125MHz。
	 * 日志吞吐量很小，优先保证杜邦线连接下的读写稳定性。
	 */
	SPI1->CR1 |= (5U<<3);
    SPI1->CR1 |= (1<<8);//SSI内部从设备选择
	SPI1->CR1 |= (1<<9);//SSM软件控制从设备
	SPI1->CR1 |= (1<<6);//SPE开启SPI
	
}

/*
 * SPI1收发一个字节。
 * 先等待发送寄存器空，再写入发 送数据；
 * 随后等待接收寄存器非空，读取并返回本次接收值。
 */
uint8_t SPI1_SwapByte(uint8_t tx_data)
{
    // 等待发送数据寄存器允许写入。
    while ((SPI1->SR & (1U << 1)) == 0U)
    {
    }

    // 采用8位访问，写入本次需要发送的字节。
    *(volatile uint8_t *)&SPI1->DR = tx_data;

    // 等待本次SPI交换接收到一个字节。
    while ((SPI1->SR & (1U << 0)) == 0U)
    {
    }

    // 读取接收数据，同时清除RXNE条件。
    return *(volatile uint8_t *)&SPI1->DR;
}


//读FLASH寄存器
uint8_t XM25QH32_ReadStatusReg(void)
{
  uint8_t status;
  SPI_FLASH_CS_LOW();
  SPI1_SwapByte(0x05);//发送0X05
	status = SPI1_SwapByte(0xFF); //status=接收到的值
	  
	//等BSY=0，忙标志不忙
 	while(SPI1->SR & (1 << 7));
  SPI_FLASH_CS_HIGH();
	
  return status;
}


//FLASH 忙标志
void XM25QH32_WaitBusy(void)
{
    while(XM25QH32_ReadStatusReg() & 0x01);
}


//FLASH写使能
void XM25QH32_WriteEnable(void)
{
    SPI_FLASH_CS_LOW();
    SPI1_SwapByte(0x06);
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
}


//读FLASH ID
uint16_t XM25QH32_ReadID(void)
{
  uint8_t id1, id2;
  SPI_FLASH_CS_LOW();
  SPI1_SwapByte(0x90);
  SPI1_SwapByte(0x00); 
	SPI1_SwapByte(0x00); 
	SPI1_SwapByte(0x00);
  id1 = SPI1_SwapByte(0xFF);
  id2 = SPI1_SwapByte(0xFF);
  while(SPI1->SR & (1 << 7));
  SPI_FLASH_CS_HIGH();
  
	return ((uint16_t)id1 << 8) | id2;
}



//读数据
void XM25QH32_ReadData(uint32_t addr, uint8_t *buffer, uint16_t len)
{
    SPI_FLASH_CS_LOW();
    SPI1_SwapByte(0x03);
    SPI1_SwapByte((addr >> 16) & 0xFF);
    SPI1_SwapByte((addr >> 8) & 0xFF);
    SPI1_SwapByte(addr & 0xFF);
    for(uint16_t i = 0; i < len; i++)
        buffer[i] = SPI1_SwapByte(0xFF);
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
}


//页编程
void XM25QH32_PageProgram(uint32_t addr, uint8_t *buffer, uint16_t len)
{
	if(len > 256)
	{
		len = 256;
	}
    
    XM25QH32_WaitBusy();
    XM25QH32_WriteEnable();
	
    SPI_FLASH_CS_LOW();
	
    SPI1_SwapByte(0x02);
    SPI1_SwapByte((addr >> 16) & 0xFF);
    SPI1_SwapByte((addr >> 8) & 0xFF);
    SPI1_SwapByte(addr & 0xFF);
	
    for(uint16_t i = 0; i < len; i++)
	{
		SPI1_SwapByte(buffer[i]);
	}
  
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
    XM25QH32_WaitBusy();
}


// 扇区擦除
void XM25QH32_SectorErase(uint32_t addr)
{
    XM25QH32_WaitBusy();
    XM25QH32_WriteEnable();
    SPI_FLASH_CS_LOW();
    SPI1_SwapByte(0x20);
    SPI1_SwapByte((addr >> 16) & 0xFF);
    SPI1_SwapByte((addr >> 8) & 0xFF);
    SPI1_SwapByte(addr & 0xFF);
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
    XM25QH32_WaitBusy();
}


//写数据--已确认Flash区域是擦除状态（全0xFF），直接写
void XM25QH32_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint16_t len)
{
    uint16_t page_remain = 256 - (addr % 256);
    if(page_remain == 0) page_remain = 256;

    while(len)
    {
        if(len <= page_remain)
        {
            XM25QH32_PageProgram(addr, buffer, len);
            break;
        }
        else
        {
            XM25QH32_PageProgram(addr, buffer, page_remain);
            addr += page_remain;
            buffer += page_remain;
            len -= page_remain;
            page_remain = 256;
        }
    }
}




//FLASH写数据--不确定Flash状态，自动读扇区→判断→擦除→写回
void XM25QH32_Write(
    uint8_t *buffer,
    uint32_t addr,
    uint16_t len)
{
    uint32_t sector_pos;
    uint16_t sector_offset;
    uint16_t sector_remain;
    uint16_t i;

    sector_pos = addr / SPI_FLASH_SECTOR_SIZE;

    sector_offset = addr % SPI_FLASH_SECTOR_SIZE;

    sector_remain = SPI_FLASH_SECTOR_SIZE - sector_offset;

    if(len <= sector_remain)
    {
        sector_remain = len;
    }

    while(1)
    {
        /* 读取整个扇区 */
        XM25QH32_ReadData(
            sector_pos * SPI_FLASH_SECTOR_SIZE,
            SectorBuffer,
            SPI_FLASH_SECTOR_SIZE);

        /* 判断是否需要擦除 */
        for(i = 0; i < sector_remain; i++)
        {
            if(SectorBuffer[sector_offset + i] != 0xFF)
            {
                break;
            }
        }

        if(i < sector_remain)
        {
            /* 需要擦除 */

            XM25QH32_SectorErase(
                sector_pos * SPI_FLASH_SECTOR_SIZE);

            /* 修改缓存 */

            for(i = 0; i < sector_remain; i++)
            {
                SectorBuffer[sector_offset + i]
                    = buffer[i];
            }

            /* 写回整个扇区 */

            XM25QH32_Write_NoCheck(
                SectorBuffer,
                sector_pos * SPI_FLASH_SECTOR_SIZE,
                SPI_FLASH_SECTOR_SIZE);
        }
        else
        {
            /* 直接写 */

            XM25QH32_Write_NoCheck(
                buffer,
                addr,
                sector_remain);
        }

        if(len == sector_remain)
        {
            break;
        }

        sector_pos++;

        sector_offset = 0;

        buffer += sector_remain;

        addr += sector_remain;

        len -= sector_remain;

        if(len > SPI_FLASH_SECTOR_SIZE)
        {
            sector_remain = SPI_FLASH_SECTOR_SIZE;
        }
        else
        {
            sector_remain = len;
        }
    }
}







