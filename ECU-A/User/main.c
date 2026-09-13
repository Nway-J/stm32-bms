/*
 * 文件名称：main.c
 *
 * 模块名称：ECU-A系统启动入口
 *
 * 模块职责：
 * 1. 按依赖顺序初始化板级硬件和应用模块；
 * 2. 初始化Sensor、故障管理、事件队列和日志系统；
 * 3. 创建FreeRTOS应用任务；
 * 4. 启动FreeRTOS调度器；
 * 5. 在任务创建或调度器启动失败时保留错误现场。
 *
 * 当前迁移阶段：
 * FreeRTOS已经接管CPU调度。
 * Sensor、BMS、CAN、FaultLog和Health已经迁移为独立任务；
 * LED状态指示由BmsTask统一处理，裸机Scheduler已停止使用；
 * DiagnosticTask负责CLI和资源诊断功能。
 *
 * 当前时间来源：
 * SysTick为FreeRTOS提供1ms系统节拍；
 * TIM3暂时继续提供ADC硬件触发和原裸机GetTick()时间。
 *
 * 后续方向：
 * 后续完善多任务USART输出的并发保护。
 */

// 引入STM32F103外设和中断优先级接口。
#include "stm32f10x.h"
/*
 * FreeRTOS.h必须放在其他FreeRTOS功能头文件之前。
 * task.h提供任务创建和启动调度器等接口。
 */
#include "FreeRTOS.h"
#include "task.h"
// 引入串口打印声明。
#include <stdio.h>

// 引入板级GPIO、蜂鸣器和通信驱动。
#include "bsp_gpio.h"
#include "bsp_buzzer.h"
#include "bsp_usart.h"
#include "bsp_can.h"

// 引入ADC/DMA、TIM3和SPI Flash驱动。
#include "bsp_adc_dma.h"
#include "bsp_tim.h"
#include "bsp_spi_flash.h"

// 引入应用层传感器、日志和事件接口。
#include "app_sensor.h"
#include "app_flash_log.h"
#include "app_event.h"

#include "app_fault_manager.h"
#include "app_rtos.h"

// 引入故障边沿RAM队列；后续由该接口解耦状态保护和Flash日志。
#include "app_fault_event.h"

// 引入USART命令行诊断接口。
#include "app_cli.h"

// 引入系统健康监控和独立看门狗接口。
#include "app_health_monitor.h"

// 保存SPI Flash设备ID，供Keil Watch验证硬件通信。
volatile uint16_t g_spi_flash_id = 0U;



// 程序入口。
int main(void)
{
    /*
     * FreeRTOS使用BASEPRI管理可调用内核API的中断。
     * STM32F103的4个优先级位全部作为抢占优先级，不再划分子优先级。
     * 当前虽然仍运行裸机调度器，但先完成平台配置，为下一步启动内核做准备。
     */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // 先初始化本地人机接口，保证后续状态和故障可以被观察。
    LED_Init();
    KEY_Init();
    BUZZER_Init();
    USART1_Init();

    /*
    * 先初始化所有软件状态，再启动对应硬件数据源。
    * Sensor Service和Fault Manager必须在ADC产生数据前清零。
    */
    Sensor_ServiceInit();
	
    FaultManager_Init();

    // 必须在第一次故障评估前清空故障事件队列和诊断计数。
    FaultEvent_Init();
	
    ADC_DMA_Init();

    /*
    * 初始化SPI1后才能访问外部Flash。
    * 未初始化SPI就读取ID，会卡在SPI收发标志等待中。
    */
    SPI1_Init();
    g_spi_flash_id = XM25QH32_ReadID();
    FlashLog_Init();

    /*
     * Flash扫描完成后立即保存本次启动及复位原因。
     * 此时TIM3尚未启动，所以启动记录的时间固定为0ms。
     */
    (void)FlashLog_WriteBootEvent();

    // 初始化CAN硬件；当前仍沿用旧版500kbps驱动。
    CAN1_Init();

    // 清空应用事件队列；各FreeRTOS任务在启动后建立自己的周期基准。
    Event_Init();

    /*
     * 最后启动TIM3。
     * 从这一刻开始，系统毫秒计数和ADC的1ms周期触发同时生效。
     */
    TIM3_Init();

   /*
    * 所有硬件、软件状态和周期调度器准备完成后，
    * 才启动Health Monitor和独立看门狗。
    *
    * 启动后约4秒内如果没有正常刷新，STM32会自动复位。
    */
    HealthMonitor_Init();

    /*
    * 创建当前阶段的全部FreeRTOS应用任务：
    * Health、Sensor、BMS、CAN、FaultLog和Diagnostic。
    *
    * 创建成功只表示任务已经加入FreeRTOS任务列表，
    * 此时调度器尚未启动，任务函数还没有真正开始运行。
    */
    if (AppRTOS_CreateTasks() == 0U)
    {
        /*
        * 创建失败通常表示FreeRTOS Heap不足，
        * 或传入的任务参数不合法。
        *
        * 不继续运行裸机调度器，否则会掩盖RTOS启动失败。
        */
        printf("[RTOS] ERROR: task creation failed\r\n");

        for (;;)
        {
            /*
            * 保留错误现场。
            * IWDG已经启动，因此非调试状态下最终会自动复位。
            */
        }
    }

    /*
    * 到这里说明所有应用任务已经创建成功。
    */
    printf("[SYSTEM] ECU-A FreeRTOS scheduler start\r\n");

    /*
    * CLI只初始化一次。
    * 后续CLI_Process()由DiagnosticTask周期调用。
    */
    CLI_Init();

    /*
    * 启动FreeRTOS调度器。
    *
    * 从这里开始：
    * 1. SysTick每1ms产生FreeRTOS节拍；
    * 2. 内核根据任务状态和优先级选择Ready任务运行；
    * 3. 各应用任务开始按照各自周期或事件条件运行；
    * 4. main函数不再参与正常业务循环。
    */
    vTaskStartScheduler();

    /*
    * 正常情况下vTaskStartScheduler()永远不会返回。
    *
    * 如果执行到这里，通常说明Idle任务创建失败，
    * 最常见原因是FreeRTOS Heap不足。
    */
    printf("[RTOS] ERROR: scheduler returned\r\n");

    for (;;)
    {
        /*
        * 保留失败现场并等待IWDG复位。
        */
    }
}
