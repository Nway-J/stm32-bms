/*
 * 文件名称：app_cli.c
 *
 * 模块名称：USART命令行诊断接口
 *
 * 工作流程：
 * USART每收到一个字符
 *        ↓
 * 放入一行命令缓冲区
 *        ↓
 * 收到回车或换行
 *        ↓
 * 比较命令文本并调用对应的只读诊断或明确操作接口
 *
 * 设计边界：
 * 1. 串口接收不等待，没有字符就立即返回；
 * 2. 不在中断中执行printf、Flash读取或扇区擦除；
 * 3. 每轮最多处理16个字符，避免连续串口数据长期占用主循环；
 * 4. 当前不加入故障注入，避免诊断命令意外改变保护条件。
 */

#include "app_cli.h"

#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"
#include "app_bms_state.h"
#include "app_fault_manager.h"
#include "app_flash_log.h"
#include "app_health_monitor.h"
#include "app_sensor.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_usart.h"


/* 包含结尾\0，实际允许输入31个可见字符。 */
#define CLI_LINE_BUFFER_SIZE       32U

/* 单次CLI_Process最多取走的字符数，限制一次调用的执行时间。 */
#define CLI_MAX_CHARS_PER_PROCESS  16U


static char g_cli_line[CLI_LINE_BUFFER_SIZE];
static uint8_t g_cli_line_length = 0U;

volatile uint32_t g_cli_command_count = 0U;
volatile uint32_t g_cli_unknown_count = 0U;


static void CLI_IncrementSaturated(volatile uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}


static const char *CLI_GetBmsStateName(BMS_State_t state)
{
    switch (state)
    {
        case BMS_INIT:
            return "INIT";
        case BMS_SELF_CHECK:
            return "SELF_CHECK";
        case BMS_STANDBY:
            return "STANDBY";
        case BMS_MONITOR:
            return "MONITOR";
        case BMS_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}


static void CLI_PrintPrompt(void)
{
    printf("> ");
}


static void CLI_PrintHelp(void)
{
    printf("\r\nCommands:\r\n");
    printf("  help       - show command list\r\n");
    printf("  status     - show sensor and BMS state\r\n");
    printf("  fault      - show fault manager summary\r\n");
    printf("  fault inject <name> - inject a sensor fault\r\n");
    printf("  fault inject none   - stop fault injection\r\n");
    printf("  log        - print Flash V2 fault history\r\n");
    printf("  log clear  - erase Flash V2 fault history\r\n");
    printf("  can stats  - show CAN diagnostic counters\r\n");
    printf("  watchdog test - stop feeding IWDG and verify reset\r\n");
    printf("  reboot     - software reset ECU-A\r\n\r\n");
}


/* 把16位故障掩码翻译成人能直接阅读的故障名称。 */
static void CLI_PrintFaultNames(uint16_t mask)
{
    if (mask == FAULT_MASK_NONE)
    {
        printf("NONE");
        return;
    }

    if ((mask & FAULT_MASK_OV) != 0U)           { printf("OV "); }
    if ((mask & FAULT_MASK_UV) != 0U)           { printf("UV "); }
    if ((mask & FAULT_MASK_OT) != 0U)           { printf("OT "); }
    if ((mask & FAULT_MASK_NTC_OPEN) != 0U)     { printf("NTC_OPEN "); }
    if ((mask & FAULT_MASK_NTC_SHORT) != 0U)    { printf("NTC_SHORT "); }
    if ((mask & FAULT_MASK_SENSOR_DATA) != 0U)  { printf("SENSOR_DATA "); }
    if ((mask & FAULT_MASK_ADC_TIMEOUT) != 0U)  { printf("ADC_TIMEOUT "); }
    if ((mask & FAULT_MASK_SELF_CHECK) != 0U)   { printf("SELF_CHECK "); }
    if ((mask & FAULT_MASK_CAN_DEGRADED) != 0U) { printf("CAN_DEGRADED "); }
    if ((mask & FAULT_MASK_CAN_BUS_OFF) != 0U)  { printf("CAN_BUS_OFF "); }
}


static void CLI_PrintStatus(void)
{
    SensorSample_t sample;
    uint32_t now_ms;
    uint32_t age_ms;
    uint32_t voltage_mv;
    int32_t temperature_dC;

    now_ms = GetTick();

    printf("\r\n[STATUS] State=%s Fault=0x%04X Tick=%lums\r\n",
           CLI_GetBmsStateName(BMS_GetState()),
           (unsigned int)g_fault_code,
           (unsigned long)now_ms);

    if (Sensor_GetLatest(&sample) == 0U)
    {
        printf("[STATUS] Sensor=NO_DATA\r\n");
        return;
    }

    age_ms = now_ms - sample.timestamp_ms;

    printf("[STATUS] Sequence=%lu Age=%lums ADC0=%u ADC1=%u\r\n",
           (unsigned long)sample.sequence,
           (unsigned long)age_ms,
           (unsigned int)sample.adc_pa0,
           (unsigned int)sample.adc_pa1);

    if (sample.voltage_valid != 0U)
    {
        voltage_mv = (uint32_t)(sample.cell_voltage_v * 1000.0f + 0.5f);
        printf("[STATUS] Cell=%lumV\r\n", (unsigned long)voltage_mv);
    }
    else
    {
        printf("[STATUS] Cell=INVALID\r\n");
    }

    if (sample.temperature_valid != 0U)
    {
        temperature_dC = (sample.temperature_c >= 0.0f) ?
            (int32_t)(sample.temperature_c * 10.0f + 0.5f) :
            (int32_t)(sample.temperature_c * 10.0f - 0.5f);

        if (temperature_dC < 0)
        {
            temperature_dC = -temperature_dC;
            printf("[STATUS] Temp=-%ld.%ldC\r\n",
                   (long)(temperature_dC / 10),
                   (long)(temperature_dC % 10));
        }
        else
        {
            printf("[STATUS] Temp=%ld.%ldC\r\n",
                   (long)(temperature_dC / 10),
                   (long)(temperature_dC % 10));
        }
    }
    else
    {
        printf("[STATUS] Temp=INVALID NTC=%u\r\n",
               (unsigned int)sample.ntc_input);
    }
}


static void CLI_PrintFault(void)
{
    FaultManagerSnapshot_t snapshot;

    if (FaultManager_GetSnapshot(&snapshot) == 0U)
    {
        printf("\r\n[FAULT] Snapshot unavailable\r\n");
        return;
    }

    printf("\r\n[FAULT] Active=0x%04X Latched=0x%04X\r\n",
           (unsigned int)snapshot.active_mask,
           (unsigned int)snapshot.latched_mask);
    printf("[FAULT] Critical=0x%04X RecoveryReady=0x%04X\r\n",
           (unsigned int)snapshot.critical_mask,
           (unsigned int)snapshot.recovery_ready_mask);
    printf("[FAULT] Names=");
    CLI_PrintFaultNames(snapshot.active_mask);
    printf("\r\n");
    printf("[FAULT] Injected=0x%04X ",
           (unsigned int)FaultManager_GetInjectionMask());
    CLI_PrintFaultNames(FaultManager_GetInjectionMask());
    printf("\r\n");
}


/*
 * 把CLI中的故障名称转换成故障管理器使用的位掩码。
 * 返回1表示名称有效，返回0表示不支持该名称。
 */
static uint8_t CLI_ParseInjectionName(const char *name,
                                      uint16_t *mask)
{
    if ((name == 0) || (mask == 0))
    {
        return 0U;
    }

    if (strcmp(name, "none") == 0)             { *mask = FAULT_MASK_NONE; }
    else if (strcmp(name, "ov") == 0)          { *mask = FAULT_MASK_OV; }
    else if (strcmp(name, "uv") == 0)          { *mask = FAULT_MASK_UV; }
    else if (strcmp(name, "ot") == 0)          { *mask = FAULT_MASK_OT; }
    else if (strcmp(name, "ntc_open") == 0)    { *mask = FAULT_MASK_NTC_OPEN; }
    else if (strcmp(name, "ntc_short") == 0)   { *mask = FAULT_MASK_NTC_SHORT; }
    else if (strcmp(name, "sensor_data") == 0) { *mask = FAULT_MASK_SENSOR_DATA; }
    else if (strcmp(name, "adc_timeout") == 0) { *mask = FAULT_MASK_ADC_TIMEOUT; }
    else
    {
        return 0U;
    }

    return 1U;
}


static void CLI_SetFaultInjection(const char *name)
{
    uint16_t mask;

    if (CLI_ParseInjectionName(name, &mask) == 0U)
    {
        CLI_IncrementSaturated(&g_cli_unknown_count);
        printf("\r\n[CLI] Unsupported injection: %s\r\n", name);
        printf("[CLI] Use ov, uv, ot, ntc_open, ntc_short, "
               "sensor_data, adc_timeout or none\r\n");
        return;
    }

    FaultManager_SetInjectionMask(mask);

    printf("\r\n[CLI] Fault injection=0x%04X ",
           (unsigned int)mask);
    CLI_PrintFaultNames(mask);
    printf("\r\n");

    if (mask == FAULT_MASK_NONE)
    {
        printf("[CLI] Injection stopped; normal recovery rules now apply\r\n");
    }
    else
    {
        printf("[CLI] Real sensor values are unchanged\r\n");
    }
}


static void CLI_PrintCanStats(void)
{
    printf("\r\n[CAN] LastResult=%u BusOffActive=%u ESR=0x%08lX\r\n",
           (unsigned int)g_can_last_send_result,
           (unsigned int)g_can_bus_off_active,
           (unsigned long)g_can_last_esr);
    printf("[CAN] TxOK=%lu TxFail=%lu BusOff=%lu Recovery=%lu\r\n",
           (unsigned long)g_can_tx_ok_count,
           (unsigned long)g_can_tx_fail_count,
           (unsigned long)g_can_bus_off_count,
           (unsigned long)g_can_recovery_count);
}


static void CLI_ExecuteCommand(const char *command)
{
    CLI_IncrementSaturated(&g_cli_command_count);

    if (strcmp(command, "help") == 0)
    {
        CLI_PrintHelp();
    }
    else if (strcmp(command, "status") == 0)
    {
        CLI_PrintStatus();
    }
    else if (strcmp(command, "fault") == 0)
    {
        CLI_PrintFault();
    }
    else if (strncmp(command, "fault inject ", 13U) == 0)
    {
        CLI_SetFaultInjection(&command[13]);
    }
    else if (strcmp(command, "log") == 0)
    {
        FlashLog_PrintAll();
    }
    else if (strcmp(command, "log clear") == 0)
    {
        if (FlashLog_Clear() == 0U)
        {
            printf("[CLI] Log clear failed; old write index preserved\r\n");
        }
    }
    else if (strcmp(command, "can stats") == 0)
    {
        CLI_PrintCanStats();
    }
    else if (strcmp(command, "watchdog test") == 0)
    {
        /*
         * 只让Health Monitor停止刷新IWDG，不主动调用软件复位。
         * 这样最终复位是否发生，可以真实证明独立看门狗在工作。
         */
        g_health_test_stop_feed = 1U;
        printf("\r\n[HEALTH] IWDG feed stopped for test\r\n");
        printf("[HEALTH] Hardware reset expected in about 4 seconds\r\n");
    }
    else if (strcmp(command, "reboot") == 0)
    {
        printf("\r\n[CLI] Software reset requested\r\n");
        USART1_WaitSendComplete();
        NVIC_SystemReset();
    }
    else
    {
        CLI_IncrementSaturated(&g_cli_unknown_count);
        printf("\r\n[CLI] Unknown command: %s\r\n", command);
        printf("[CLI] Type help for command list\r\n");
    }
}


static void CLI_HandleByte(uint8_t data)
{
    /* 回车或换行代表一条命令输入完成。 */
    if ((data == '\r') || (data == '\n'))
    {
        /* CR+LF中的第二个结束符不会产生一条空命令。 */
        if (g_cli_line_length == 0U)
        {
            return;
        }

        g_cli_line[g_cli_line_length] = '\0';
        printf("\r\n");
        CLI_ExecuteCommand(g_cli_line);
        g_cli_line_length = 0U;
        g_cli_line[0] = '\0';
        CLI_PrintPrompt();
        return;
    }

    /* 支持退格键和Delete键修改尚未提交的命令。 */
    if ((data == 0x08U) || (data == 0x7FU))
    {
        if (g_cli_line_length > 0U)
        {
            g_cli_line_length--;
            g_cli_line[g_cli_line_length] = '\0';
            USART1_SendString("\b \b");
        }
        return;
    }

    /* 控制字符不进入命令缓冲区。 */
    if ((data < 0x20U) || (data > 0x7EU))
    {
        return;
    }

    if (g_cli_line_length >= (CLI_LINE_BUFFER_SIZE - 1U))
    {
        g_cli_line_length = 0U;
        g_cli_line[0] = '\0';
        printf("\r\n[CLI] Command too long, input cleared\r\n");
        CLI_PrintPrompt();
        return;
    }

    g_cli_line[g_cli_line_length] = (char)data;
    g_cli_line_length++;
    g_cli_line[g_cli_line_length] = '\0';

    /* 本机回显；若串口工具启用了本地回显，应关闭本地回显。 */
    USART1_SendByte(data);
}


void CLI_Init(void)
{
    g_cli_line_length = 0U;
    g_cli_line[0] = '\0';
    g_cli_command_count = 0U;
    g_cli_unknown_count = 0U;

    printf("[CLI] Diagnostic terminal ready. Type help.\r\n");
    CLI_PrintPrompt();
}


void CLI_Process(void)
{
    uint8_t data;
    uint8_t processed_count;

    processed_count = 0U;

    while ((processed_count < CLI_MAX_CHARS_PER_PROCESS) &&
           (USART1_ReadByteNonBlocking(&data) != 0U))
    {
        CLI_HandleByte(data);
        processed_count++;
    }
}
