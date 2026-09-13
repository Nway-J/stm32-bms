/*
 * 文件名称：app_cli.h
 *
 * 模块名称：USART命令行诊断接口
 *
 * 模块职责：
 * 1. 接收用户通过USART输入的一行命令；
 * 2. 查询传感器、BMS、故障、CAN和Flash日志状态；
 * 3. 只在用户输入完整命令后执行对应操作。
 *
 * 当前裸机实现：
 * main循环频繁调用CLI_Process()；没有收到字符时立即返回。
 * 迁移FreeRTOS后，可保留命令解释部分，把字符来源改成队列。
 */
#ifndef APP_CLI_H
#define APP_CLI_H

#include <stdint.h>

/* 供Keil Watch观察CLI是否收到并执行命令。 */
extern volatile uint32_t g_cli_command_count;
extern volatile uint32_t g_cli_unknown_count;

/* 清空命令缓冲并输出帮助提示。 */
void CLI_Init(void);

/* 非阻塞接收并处理USART字符，应在主循环中频繁调用。 */
void CLI_Process(void);

#endif
