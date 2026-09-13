# STM32双ECU：早期裸机基线

当前分支保存历史裸机代码。推荐使用[main分支的当前验收版](../../tree/main)，其中ECU-A已迁移为FreeRTOS，并完善了故障管理、CLI和通信诊断。

## 来源

作者旧私有仓库 stm32-bms-freertos-portfolio，baseline/baremetal提交 `baae12b7cfc09599bbbcb3eadd16f0ad4a9fd991`。公开整理仅保留业务源码及工程结构，ST依赖改为自行恢复。该版本早于正式RTOS迁移前的最后一轮裸机改造，不声称功能与main等价。

## 模块关系

```mermaid
flowchart LR
    Tick[毫秒时间] --> Scheduler[主循环周期调度]
    ADC[ADC与DMA] --> BMS[状态机与基础故障判断]
    Scheduler --> BMS
    BMS --> Log[SPI日志]
    BMS --> CAN[CAN发送]
    CAN --> Dash[ECU-B接收与OLED]
```

模块详解见[架构](docs/architecture.md)，历史协议及已知显示问题见[协议](docs/can-protocol.md)。该版本有演示性充放电状态与电压比例/显示缺陷；不能用作实际电池安全控制，也不能与main的ECU-B混烧。

## 编译

取得对应ST SPL/CMSIS软件包，使用：

```powershell
powershell -ExecutionPolicy Bypass -File tools/install-dependencies.ps1 -SourceRoot 'D:\SDK'
```

脚本按 [精确依赖清单](docs/dependencies.json) 恢复文件。该历史版本ST文件不同于main，必须匹配哈希。再用Keil ARMCC5打开 `ECU_A/ECU_A.uvprojx` 与 `ECU_B/ECU_B.uvprojx` 分别Rebuild，设置本机SWD下载器后烧录。

历史快照用于学习对照；本次整理的编译结果见[构建记录](docs/build-results.txt)，没有为此旧快照重新做硬件验收。

自编代码采用[MIT](LICENSE)。ST/CMSIS依赖未包含在公开源码中，其权利仍归原作者，按供应方条款取得使用。原始业务源码保持不变。
