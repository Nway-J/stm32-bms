# FreeRTOS官方文件与本项目差异

## 先分清两个文件

- `FreeRtos/inc/FreeRTOS.h` 是内核公共头文件。本次与官方V11.1.0比较，**除换行格式外一致，没有业务修改**。
- `FreeRtos/FreeRTOSConfig.h` 是本项目配置。需要适配主频、节拍、中断、内存与功能开关。

官方基准：[FreeRTOS-Kernel V11.1.0](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.1.0)，提交 `dbf70559b27d39c1fdb68dfb9a32140b6a6777a0`。配置基准为该版本 `examples/template_configuration/FreeRTOSConfig.h`，不是凭记忆列出的“常用配置”。

全文件比对结果见 [JSON清单](freertos-file-comparison.json)，配置逐行差异见 [patch](freertos-config.patch)。内核 `inc`、`src`、RVDS/ARM_CM3端口及heap文件均与该版本比对一致（忽略CRLF/LF）。`freertos_hooks.c`是应用适配文件，不是官方模板修改。

## ECU-A配置变化

| 配置 | 官方模板 → 当前 | 原因 |
|---|---|---|
| CPU_CLOCK_HZ | 20MHz → 72MHz | 与板上时钟一致 |
| TICK_RATE_HZ | 100 → 1000 | 1ms调度节拍 |
| USE_TIME_SLICING | 0 → 1 | 同优先级就绪任务可按节拍轮转 |
| NUMBER_OF_CORES | 未显式定义 → 1 | STM32F103单核 |
| MAX_PRIORITIES | 5 → 6 | 允许0~5，Health使用5 |
| TICK_TYPE_WIDTH_IN_BITS | 64 → 32 | 当前32位平台与端口 |
| QUEUE_REGISTRY_SIZE | 0 → 8 | 预留调试注册表；开关不等于已注册所有队列 |
| USE_TIMERS | 1 → 0 | 周期任务自行等待，无软件定时器任务 |
| TIMER_TASK_PRIORITY | MAX_PRIORITIES-1 → 3 | 定时器关闭时此值不生效 |
| USE_STREAM_BUFFERS | 1 → 0 | USART采用自编环形缓冲 |
| TOTAL_HEAP_SIZE | 4096 → 8192字节 | 六个任务栈与TCB等动态分配 |
| ENABLE_HEAP_PROTECTOR | 0 → 1 | 提升堆指针破坏可检测性 |
| KERNEL_INTERRUPT_PRIORITY | 0 → 15<<4 | 内核异常使用最低硬件优先级 |
| MAX_SYSCALL_INTERRUPT_PRIORITY | 0 → 5<<4 | BASEPRI内核API中断边界 |
| USE_TRACE_FACILITY | 0 → 1 | 支持任务诊断相关信息 |
| ENABLE_TRUSTZONE / MPU / FPU / MVE | 1 → 0 | 本项目Cortex-M3端口不用这些扩展 |
| USE_RECURSIVE_MUTEXES | 1 → 0 | 普通Flash Mutex足够 |
| USE_COUNTING_SEMAPHORES | 1 → 0 | 当前通信不需要计数信号量 |
| INCLUDE_uxTaskGetStackHighWaterMark | 0 → 1 | 读取历史最小剩余栈 |
| INCLUDE_xTaskGetIdleTaskHandle / eTaskGetState | 0 → 1 | 保留任务诊断接口 |
| INCLUDE_xTaskAbortDelay / xTaskGetHandle | 0 → 1 | 接口启用，不能因此声称业务已使用 |
| 异常映射 | 新增3个宏 | SVC、PendSV、SysTick对接启动向量 |

表内省略统一的 `config` 前缀。下列设置虽然重要，但**沿用模板**：抢占调度开启、静态/动态分配均开启、普通Mutex和任务通知开启、栈溢出检测为2、Idle栈128 words。

## Cortex-M3移植怎么接起来

```text
启动向量SVC/PendSV/SysTick
     ↓ 配置中的名称映射
RVDS/ARM_CM3端口
     ↓
任务切换与FreeRTOS节拍
```

main设置4位抢占优先级分组。硬件中断数字越小紧急程度越高；调用FromISR内核API的中断必须满足内核阈值要求，不能在硬件优先级0~4中随意调用。串口RX中断当前不调用FreeRTOS API。

TIM3仍提供业务毫秒时间与ADC触发，SysTick负责RTOS调度。不要把两个时钟重复配置为同一个中断处理入口。

## 适配钩子与内存

`freertos_hooks.c`提供栈溢出处理：停止正常调度，便于调试定位，IWDG仍可最终复位。另提供heap canary，基于非零常数、时钟和地址组合；它不是密码学随机数。

工程选择heap_4；其他heap文件虽保留，不代表同时链接。任务动态创建，故障队列和Flash Mutex静态创建。栈参数是word，当前平台1word=4字节；高水位是历史最小剩余量，不是当前剩余量，也不能证明未测试路径不会溢出。

ECU-B目录保留内核与配置文件供后续学习，但主循环没有启动FreeRTOS；其配置不能作为当前运行任务的证据。
