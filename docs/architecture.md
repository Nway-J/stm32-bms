# 当前裸机架构（静态源码描述）

本文记录当前仓库中 ECU_A 与 ECU_B 源码呈现的结构和数据路径，供开发沟通使用。它是静态代码阅读结果，不代表本轮已经编译、联调或上板验证，也不应据此推断量产能力。

## ECU_A：BMS 节点

### 启动流程

`ECU_A/user/main.c` 的当前活动入口按以下顺序执行：

1. 配置 NVIC 优先级分组（`NVIC_PriorityGroup_2`）。
2. 初始化 USART1、SPI1、GPIO（LED 与按键）、TIM3、ADC-DMA 和 CAN1。
3. 初始化软件调度器。
4. 输出启动信息，读取 SPI Flash 的 ID；ID 既非 `0xFFFF` 也非 `0x0000` 时初始化并打印故障日志，否则仅报告未检测到 Flash。
5. 进入无限循环，反复调用 `Scheduler_Run()`。

### 周期调度

调度器使用 `GetTick()` 比较每项任务的上次执行时间，在主循环中依次检查、直接调用到期任务。没有任务切换、优先级抢占或独立栈，因此这是**非抢占式裸机调度**；任何任务的执行时间都会影响后续任务的实际响应时刻。

| 周期 | 调用 | 当前作用 |
| --- | --- | --- |
| 10 ms | `BMS_StateMachine()` | 执行当前 BMS 状态对应的任务。放电时读取 ADC-DMA 数据、换算、检测故障并发送业务 CAN 帧。 |
| 20 ms | `Event_Process()` | 取出并处理事件队列，调用状态转换处理。 |
| 500 ms | `LED_Toggle()` | 仅在当前状态不是 `BMS_FAULT` 时翻转 LED。 |

故障状态中的 LED 快闪和 0x303 故障帧发送由 `Task_Fault()` 自行基于 `GetTick()` 控制；它仍只在上述 10 ms 状态机调用中获得执行机会。

### BMS 状态机

状态枚举为：

| 状态 | 当前代码行为摘要 |
| --- | --- |
| `POWER_OFF` | 执行 LED 启动闪烁后切到 `SELF_CHECK`。 |
| `SELF_CHECK` | 等待至少 100 ms，检查 ADC/DMA 就绪标志与 CAN 错误状态；通过进入 `STANDBY`，失败进入 `FAULT`。 |
| `STANDBY` | 当前活动代码仅首次压入 `EVENT_START`。 |
| `DISCHARGE` | 读取电压/温度，计算 SOC、检查故障；正常时发送 0x301 和 0x302。 |
| `FAULT` | 200 ms 节奏翻转 LED，约 500 ms 发送 0x303；按键可请求故障恢复。 |
| `CHARGE` | 使用 ADC 电压/温度，发送充电状态帧；其中 SOC 为演示性递增值，达到阈值后压入充满事件。 |

状态转换由 `BMS_EventHandle()` 集中处理。当前 `EVENT_START` 的活动路径固定为 `STANDBY → DISCHARGE`。`CHARGE` 内确有演示性的 SOC 递增逻辑，但当前活动路径并不通过 `EVENT_START` 进入充电；源码中原有的低 SOC 充电入口处于注释代码内，后续需要统一入口语义后再验证。

### 当前数据流

```text
ADC + DMA 双缓冲采样
        ↓
ECU_A 放电/充电状态任务：电压、温度换算；SOC/故障判断
        ↓
CAN 0x301（电压、SOC、状态）/ 0x302（温度）/ 0x303（故障）
        ↓
ECU_B CAN RX0 中断 → Dash_UpdateData() → g_dash_data
        ↓
同一中断内直接调用 Dash_RefreshDisplay() 刷新 OLED
        ↓
主循环仍每 500 ms 刷新 OLED；每 1000 ms USART 调试输出
```

`SOC_Simple()` 是基于单体电压的简化线性估算（含固定上下边界），未包含电流积分、温度补偿、老化或标定模型。故障判断仅覆盖当前代码中的过压、欠压和过温阈值；日志依赖于启动时的 Flash 探测结果。

## ECU_B：仪表节点

ECU_B 初始化 NVIC、LED、USART、I2C、OLED、CAN 和 TIM3，随后在主循环中按时间戳执行：

| 周期 | 行为 |
| --- | --- |
| 500 ms | 调用 `Dash_RefreshDisplay()`，将 `g_dash_data` 显示到 OLED。 |
| 1000 ms | 通过 USART 输出电压字段、SOC、温度与故障码。 |

`USB_LP_CAN1_RX0_IRQHandler()` 在任何成功接收的标准数据帧后调用 `Dash_UpdateData()`；对 0x301、0x302、0x303 按帧 ID 更新全局 `g_dash_data`，随后在同一接收中断上下文中直接调用 `Dash_RefreshDisplay()` 刷新 OLED。因此 OLED 除主循环的 500 ms 周期刷新外，在收到任何成功接收的标准数据帧时也会即时刷新。显示刷新在数据超时超过 1000 ms 时显示 CAN 丢失提示。

ECU_B 当前接收过滤器配置为全收；`Dash_UpdateData()` 即使走 `default` 分支也会更新 `last_rx_tick`。接收路径未校验目标 ID 和最小 DLC，因此未知帧同样会延长 CAN LOST 判定，且 DLC 过短的目标帧仍可能被读取并解析。

## 已知边界

- 本文只描述当前静态源码，不表示本轮已完成编译、CAN 总线联调或硬件上板。
- 周期是调度目标，非抢占式实现不保证严格的实时期限或抖动上界。
- SOC 为简化线性电压估算；`CHARGE` 中另有演示性的 SOC 递增，二者均不能作为电量计量精度的承诺。
- 当前 `EVENT_START` 活动路径固定进入 `DISCHARGE`，充电进入条件和状态命名/协议语义仍需要后续统一。
- CAN 字段实现与 ECU_B 成员名存在历史命名差异，具体比例和迁移要求见 `docs/protocol/can-protocol.md`。
- ECU_B 当前在 CAN 接收中断中直接执行 OLED 刷新；若迁移到 RTOS，应将显示刷新移出 ISR/接收中断上下文，并重新界定数据同步与刷新触发方式。
- ECU_B 当前未在应用层筛选目标 CAN ID 或校验最小 DLC；未知帧会更新接收时间，短 DLC 的目标帧可能被解析，CAN LOST 不能被视为仅针对 BMS 业务帧的健康状态。
