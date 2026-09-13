// main.c — ECU-A BMS节点主程序
#include "stm32f10x.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "bsp_adc_dma.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_spi_flash.h"
#include "app_scheduler.h"
#include "app_event.h"
#include "app_bms_state.h"
#include "app_flash_log.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    USART1_Init();
    SPI1_Init();
    LED_Init();
    KEY_Init();
    TIM3_Init();
    ADC_DMA_Init();
    CAN1_Init();

    Scheduler_Init();

    printf("\r\n========== BMS System Start ==========\r\n");

    uint16_t flash_id = XM25QH32_ReadID();

    if (flash_id != 0xFFFF && flash_id != 0x0000)
    {
        printf("[BMS] Flash detected ID=0x%04X\r\n", flash_id);
        FlashLog_Init();
        FlashLog_PrintAll();
    }

    else
        printf("[BMS] No Flash\r\n");

    while (1)
    {
        Scheduler_Run();
    }
}
