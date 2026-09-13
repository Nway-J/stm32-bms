#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"

/**
 * @brief FreeRTOS任务栈溢出回调函数
 *
 * @param xTask
 *        发生栈溢出的任务句柄。
 *
 * @param pcTaskName
 *        发生栈溢出的任务名称。
 *
 * @note
 * 当configCHECK_FOR_STACK_OVERFLOW设置为1或2时，
 * 应用程序必须实现本函数。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char *pcTaskName)
{
    /*
     * 避免编译器提示参数未使用。
     * 调试时可以直接观察这两个参数。
     */
    (void)xTask;
    (void)pcTaskName;

    /*
     * 进入该函数说明某个任务的栈已经存在严重风险。
     * 此时继续运行可能造成：
     *
     * 1. TCB被破坏；
     * 2. 其他任务栈被破坏；
     * 3. 返回地址被覆盖；
     * 4. 程序进入HardFault；
     * 5. 系统产生不可预测行为。
     *
     * 所以这里先关闭中断并停机，
     * 保留现场供Keil调试器检查。
     */
    taskDISABLE_INTERRUPTS();

    for (;;)
    {
        /*
         * 在这里打断点。
         *
         * 重点观察：
         * pcTaskName：哪个任务溢出；
         * xTask：对应任务句柄。
         */
    }
}
/**
 * @brief 向FreeRTOS heap_4提供堆指针保护Canary
 *
 * @param pxHeapCanary
 *        用于返回Canary值的指针。
 *
 * @note
 * 当configENABLE_HEAP_PROTECTOR设置为1时，
 * 应用程序必须实现该函数。
 */
void vApplicationGetRandomHeapCanary(
    portPOINTER_SIZE_TYPE *pxHeapCanary)
{
    /*
     * 首先检查FreeRTOS传入的输出指针是否合法。
     */
    configASSERT(pxHeapCanary != NULL);

    /*
     * 当前STM32F103基础工程还没有硬件随机数发生器。
     *
     * STM32F103C8本身没有RNG外设，因此这里先使用：
     * 固定非零值
     * XOR
     * SystemCoreClock
     * XOR
     * 当前输出变量地址
     *
     * 目的主要是避免Canary为0，并让不同固件布局下
     * 得到的值不完全相同。
     *
     * 注意：
     * 这不属于密码学安全随机数。
     * heap protector的目标是提高堆指针损坏的可检测性，
     * 不是用于加密。
     */
    *pxHeapCanary =
        (portPOINTER_SIZE_TYPE)0xA5F03C96UL
        ^ (portPOINTER_SIZE_TYPE)SystemCoreClock
        ^ (portPOINTER_SIZE_TYPE)pxHeapCanary;

    /*
     * 防止极端情况下计算结果正好为0。
     */
    if (*pxHeapCanary == (portPOINTER_SIZE_TYPE)0U)
    {
        *pxHeapCanary =
            (portPOINTER_SIZE_TYPE)0x5A3CC3A5UL;
    }
}
