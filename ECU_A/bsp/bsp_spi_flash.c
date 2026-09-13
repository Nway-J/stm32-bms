//SPI Flash驱动（XM25QH32）
#include "stm32f10x.h"
#include "bsp_spi_flash.h"

void SPI1_Init(void)
{
	//1.开GPIOA/SPI1时钟
	RCC->APB2ENR |= (1<<12);
	RCC->APB2ENR |= (1<<2);
	
	//2.配置PA4-CS通用推挽 PA5-SCK复用推挽 
	//PA6-MISO浮空输入 PA7-MOSI复用推挽
	GPIOA->CRL &= ~(0xF << 16); 
	GPIOA->CRL |= (0x3 << 16);
  GPIOA->CRL &= ~(0xF << 20); 
	GPIOA->CRL |= (0xB << 20);
  GPIOA->CRL &= ~(0xF << 24); 
	GPIOA->CRL |= (0x4 << 24);
  GPIOA->CRL &= ~(0xF << 28); 
	GPIOA->CRL |= (0xB << 28);
	
	//3.CS片选，主机不选中
	GPIOA->BSRR |= (1<<4);
	
	//4.配置SPI1,模式1
	SPI1->CR1 = 0;
	SPI1->CR1 &= ~(1<<1);//CPOL时钟极性
	SPI1->CR1 &= ~(1<<0);//CPHA时钟相位
	
	SPI1->CR1 |= (1<<2);//MSTR主设备选择
	SPI1->CR1 |= (2<<3);//BR-波特率 72MHZ/8=9MHZ
  SPI1->CR1 |= (1<<8);//SSI内部从设备选择
	SPI1->CR1 |= (1<<9);//SSM软件控制从设备
	SPI1->CR1 |= (1<<6);//SPE开启SPI
	
}

//收发数据
uint8_t SPI1_SwapByte(uint8_t tx_data)
{
	//等发送寄存器空 TXE=1
	while(!(SPI1->SR & (1<<1)))
	{
		*(volatile uint8_t *) & SPI1->DR = tx_data;
	}
	//等待接收寄存器非空 RXNE=1
	while(!(SPI1->SR & (1<<0)))
	{
		return *(volatile uint8_t *) & SPI1->DR;
	}
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







