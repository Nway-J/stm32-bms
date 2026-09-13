/*
 * 文件名称：app_can_service.h
 *
 * 模块名称：CAN应用服务公共接口
 *
 * 模块职责：
 * 1. 对外提供电压、温度和故障状态报文的发送接口；
 * 2. 公开三类报文各自的Alive Counter，供Keil Watch诊断；
 * 3. 隔离BMS业务模块与CAN协议打包、CAN硬件发送操作。
 *
 * 模块边界：
 * 本模块不采集ADC、不换算NTC、不判断电池故障、
 * 不切换BMS状态，也不直接处理ECU-B显示。
 *
 * 当前迁移阶段：
 * 三种报文已经由独立CanTask周期调用；
 * 其他任务不得直接调用发送接口。
 * 
 * 当前并发约束：
 * 所有周期CAN发送均由CanTask执行；
 * 其他任务不得直接调用AppCAN_Send...()或CAN1_Send()。
 */
#ifndef APP_CAN_SERVICE_H
#define APP_CAN_SERVICE_H

#include <stdint.h>

/*
 * 使用最新有效传感器快照发送电压、SOC和监测状态。
 */
void AppCAN_SendVoltageStatus(void);

/*
 * 使用最新有效传感器快照发送温度状态。
 */
void AppCAN_SendTemperatureStatus(void);

/*
 * 发送当前完整的活动故障状态。
 */
void AppCAN_SendFaultStatus(void);

/*
 * 三类报文拥有各自独立的Alive Counter。
 *
 * 这些变量公开仅为了Keil Watch诊断，
 * 业务模块不得直接修改。
 */
extern volatile uint8_t g_can_301_alive_counter;
extern volatile uint8_t g_can_302_alive_counter;
extern volatile uint8_t g_can_303_alive_counter;

#endif
