/*
 * 文件名称：app_dash.h
 * 模块作用：保存ECU-B从CAN接收的显示数据和各报文通信状态。
 * 
 * 0x301和0x302分别保存接收时间并独立判断通信超时，
 * 其他报文到达不能掩盖关键状态报文或温度报文丢失。
 *
 * 0x301和0x302分别检查CRC8和Alive Counter：
 * CRC判断当前报文内容是否可信；
 * Alive Counter判断报文是否重复、跳号或丢失。
 * 
 * 通信状态与OLED提示状态分开保存，使通信恢复后提示仍能停留一段时间。
 * 同时检查0x301的四位Alive Counter，记录当前跳号状态和累计异常次数。
 * 
 */
#ifndef __APP_DASH_H
#define __APP_DASH_H
#include "stm32f10x.h"
#include "can_protocol.h"


/*
 * 温度执行流程：
 收到温度报文
    ↓
检查长度是否为8字节
    ↓
重新计算前7字节的CRC8
    ↓
CRC是否一致？
 ┌──否：拒绝整帧，不更新温度和时间
 ↓是
取出温度报文自己的Alive Counter
    ↓
与上一帧序号+1比较
    ↓
连续：清除当前Alive异常
跳号：记录异常次数，但仍采用当前温度
    ↓
保存温度、Alive基准和接收时间
*/



/*
 时间组：
last_30x_rx_tick

Alive Counter组：
received_30x
last_30x_alive
alive_30x_error
alive_30x_error_count

CRC组：
crc_30x_error
crc_30x_error_count
last_30x_received_crc
last_30x_calculated_crc
*/
typedef struct
{
    uint16_t cell_mv;          // 电压(mV)

    uint8_t  chg_state;        // 充放电状态

    uint8_t  temp;             // 温度

    uint8_t  soc;              // SOC

    uint16_t fault;            // 0x303携带的16位活动故障掩码

    volatile uint8_t valid;    // 是否收到过数据

    uint32_t last_rx_tick;     // 最后一次收到任意已识别BMS报文的时间

    uint32_t last_301_rx_tick; // 最近一次收到0x301的时间，用于独立超时判断
    uint32_t last_302_rx_tick;  // 最近一次收到CRC正确的温度报文的时间
    uint32_t last_303_rx_tick;  // 最近一次收到CRC正确的故障报文的时间

    uint32_t comm_301_notice_since_ms; // 本次0x301丢失提示开始显示的时间

    uint32_t alive_301_error_count; // 0x301序号不连续的累计次数，上电后只增不清
    uint32_t crc_301_error_count; //CRC校验失败的累计次数。当前恢复正常后也不清零，用于历史诊断。

    uint32_t alive_302_error_count; //温度报文序号不连续的累计次数。
    uint32_t crc_302_error_count;   //温度报文CRC校验失败的累计次数。

    uint32_t alive_303_error_count; // 故障报文序号不连续的累计次数
    uint32_t crc_303_error_count;   // 故障报文CRC校验失败的累计次数

    uint8_t received_301;      // 0=上电后尚未收到0x301，1=至少成功收到过一次
    uint8_t received_302;      // 0=上电后尚未收到0x302，1=至少成功收到过一次
    uint8_t received_303;     // 0=尚未收到有效故障报文，1=至少收到过一次

    uint8_t comm_301_timeout;  // 1=0x301已经连续500ms没有到达
    uint8_t comm_302_timeout;  //  1=连续1500ms没有收到CRC正确的温度报文
    uint8_t comm_303_timeout;  //  1=连续500ms没有收到CRC正确的故障报文

    uint8_t comm_301_notice_active; // 1=OLED仍需显示0x301丢失提示

    uint8_t last_301_alive;     // 最近一帧0x301携带的Alive Counter（0~15）
    uint8_t last_302_alive;     // 最近一帧0x302携带的Alive Counter（0~15）
    uint8_t last_303_alive;    // 最近一帧有效故障报文的Alive Counter

    uint8_t alive_301_error;    // 1=本次0x301序号不连续，下一帧连续时自动清零
    uint8_t alive_302_error;    // 1=本次0x302序号不连续，下一帧连续时自动清零
    uint8_t alive_303_error;    // 1=本次故障报文序号不连续，下一帧连续时清零
   /*
    * 1表示最近收到的关键状态报文CRC错误；
    * 下一帧CRC正确时自动恢复为0。
    */
    uint8_t crc_301_error;  //最近一帧CRC是否错误
    uint8_t crc_302_error;  //最近一帧温度报文CRC是否错误
    uint8_t crc_303_error;  // 1=最近一帧故障报文CRC错误

    /*
    * 最近一帧报文中携带的CRC。
    * 主要用于Keil Watch观察。
    */
    uint8_t last_301_received_crc; //报文自己携带的CRC
    uint8_t last_302_received_crc;//最近一帧温度报文自己携带的CRC。
    uint8_t last_303_received_crc; // 最近一帧故障报文携带的CRC

    /*
    * ECU-B根据前7个字节重新计算出的CRC。
    * 正常情况下应该与received_crc相等。
    */
    uint8_t last_301_calculated_crc; //ECU-B重新计算的电压CRC
    uint8_t last_302_calculated_crc; //ECU-B根据温度报文前7个字节重新计算的CRC。
    uint8_t last_303_calculated_crc; // ECU-B重新计算的故障报文CRC
} BMS_DisplayData_t;

// CAN中断写入、主循环读取，volatile要求每次都从实际内存取值。
extern volatile BMS_DisplayData_t g_dash_data;

void Dash_Init(void);
void Dash_UpdateData(CAN_Frame_t *frame);
void Dash_UpdateCommStatus(uint32_t now_ms);
void Dash_RefreshDisplay(void);

#endif





 
 
