// ============================================================
// app_dash.c
//
// ECU-B 仪表显示模块
//
// 功能：
// 1. 接收CAN数据
// 2. 解析BMS状态
// 3. OLED显示
// 4. CAN超时检测
// 5. 检查0x301 Alive Counter是否连续，并累计序号异常次数
// 6. 校验关键状态报文CRC8，错误报文不更新业务数据
// 7. 校验温度报文CRC8和Alive Counter，错误报文不更新温度
// 8. 单类报文超时时只标记对应数据失效，其他有效数据继续显示
// 8. 温度报文超时时停止显示旧温度，但保留电压、SOC和故障显示
// 9. 校验故障报文CRC8和Alive Counter，错误报文不能修改当前故障状态
// 10. 故障报文超时时保留最后可信故障掩码，并显示通信丢失

// CAN报文：
// 0x301
//   Byte0~1 Cell Voltage(mV)
//   Byte2    SOC(%)
//   Byte3    Charge State
//   Byte4低4位 Alive Counter(0~15循环)
//   Byte5~6 Reserved
//   Byte7    CRC8 of Byte0~Byte6
//
// 0x302
//   Byte0    Temperature
//   Byte1~3  Reserved
//   Byte4低4位 Alive Counter(0~15循环)
//   Byte5~6  Reserved
//   Byte7    CRC8 of Byte0~Byte6
//
// 0x303
//   Byte0~1 Active Fault Mask, high byte first
//   Byte2~3 Reserved
//   Byte4低4位 Alive Counter(0~15循环)
//   Byte5~6 Reserved
//   Byte7    CRC8 of Byte0~Byte6

// ============================================================

#include "app_dash.h"
#include "bsp_oled.h"
#include "bsp_tim.h"

#include <stdio.h>
#include <string.h>

// 0x301每100ms发送；连续500ms未收到即认为电压状态通信超时。
#define CAN_301_TIMEOUT_MS        500U

// 0x302每500ms发送；连续1500ms未收到即认为温度状态通信超时。
#define CAN_302_TIMEOUT_MS        1500U

// 0x303每100ms发送；连续500ms没有收到有效帧则认为故障状态通信超时。
#define CAN_303_TIMEOUT_MS        500U

// 通信恢复后仍让0x301丢失提示至少显示2秒，不阻塞主循环。
#define CAN_301_NOTICE_HOLD_MS   2000U


// ============================================================
// 全局显示数据
// ============================================================

volatile BMS_DisplayData_t g_dash_data = {0};


// ============================================================
// Dash_Init
//
// 仪表初始化
// ============================================================

void Dash_Init(void)
{
    memset((void *)&g_dash_data, 0, sizeof(g_dash_data));

}


// ============================================================
// Dash_UpdateData
//
// CAN接收后更新显示数据
//
// 由CAN接收中断调用
// ============================================================

void Dash_UpdateData(CAN_Frame_t *frame)
{
    uint32_t rx_tick;
    uint8_t recognized = 1U;
    uint8_t alive_counter;
    uint8_t expected_alive;

    if(frame == NULL)
        return;

    // 本次中断只读取一次时间，保证同一帧的各时间字段完全一致。
    rx_tick = GetTick();

    switch(frame->id)
    {
        //------------------------------------------------------
        // 电压 + SOC + 状态
        //------------------------------------------------------
        case CAN_ID_BMS_VOLTAGE:
        {
            /*
            * 当前协议规定关键状态报文长度必须是8字节。
            * 长度不正确时无法读取完整CRC，整帧按无效处理。
            */
            if(frame->dlc != 8U)
            {
                g_dash_data.crc_301_error = 1U;
                g_dash_data.crc_301_error_count++;

                /*
                * recognized清零后，本帧不会刷新
                * “最后一次有效BMS报文”的时间。
                */
                recognized = 0U;
                break;
            }

            /*
            * 保存发送端携带的CRC，方便Watch观察。
            */
            g_dash_data.last_301_received_crc =
                frame->data[7];

            /*
            * ECU-B对Byte0～Byte6重新计算CRC。
            */
            g_dash_data.last_301_calculated_crc =
                BMS_CalculateCRC8(frame->data, 7U);

            /*
            * 两个CRC不相等，说明本帧数据不可信。
            */
            if(g_dash_data.last_301_calculated_crc !=
            g_dash_data.last_301_received_crc)
            {
                g_dash_data.crc_301_error = 1U;
                g_dash_data.crc_301_error_count++;

                /*
                * 直接离开这个case：
                * 不更新电压、SOC、Alive Counter和接收时间。
                */
                recognized = 0U;
                break;
            }

            /*
            * 能运行到这里，说明报文长度和CRC都正确。
            */
            g_dash_data.crc_301_error = 0U;
                        g_dash_data.cell_mv =
                            ((uint16_t)frame->data[0] << 8) |
                            frame->data[1];

                        g_dash_data.soc =
                            frame->data[2];

                        g_dash_data.chg_state =
                            frame->data[3];

                        alive_counter = BMS_Unpack301Alive(frame);

            /*
             * 第一帧没有“上一帧”可比较，只保存基准。
             * 从第二帧开始，期待值永远是“上一帧+1后保留低四位”。
             */
            if (g_dash_data.received_301 != 0U)
            {
                expected_alive =
                    (uint8_t)((g_dash_data.last_301_alive + 1U) &
                              CAN_301_ALIVE_MASK);

                if (alive_counter != expected_alive)
                {
                    g_dash_data.alive_301_error = 1U;
                    g_dash_data.alive_301_error_count++;
                }
                else
                {
                    g_dash_data.alive_301_error = 0U;
                }
            }
            else
            {
                g_dash_data.alive_301_error = 0U;
            }

            // 无论本帧是否跳号，都以本帧作为下一次比较的新基准。
            g_dash_data.last_301_alive = alive_counter;

            g_dash_data.valid = 1U;
            g_dash_data.received_301 = 1U;
            g_dash_data.last_301_rx_tick = rx_tick;
            g_dash_data.comm_301_timeout = 0U;

            break;
        }

        //------------------------------------------------------
        // 温度
        //------------------------------------------------------
       case CAN_ID_BMS_TEMP:
        {
            /*
            * 温度报文必须是完整的8字节。
            * 长度错误时无法取得完整CRC。
            */
            if(frame->dlc != 8U)
            {
                g_dash_data.crc_302_error = 1U;
                g_dash_data.crc_302_error_count++;
                recognized = 0U;
                break;
            }

            /*
            * 保存报文携带的CRC，
            * 并根据Byte0～Byte6重新计算。
            */
            g_dash_data.last_302_received_crc =
                frame->data[7];

            g_dash_data.last_302_calculated_crc =
                BMS_CalculateCRC8(frame->data, 7U);

            /*
            * CRC不相等时整帧拒绝。
            * 不能更新温度、序号和接收时间。
            */
            if(g_dash_data.last_302_calculated_crc !=
            g_dash_data.last_302_received_crc)
            {
                g_dash_data.crc_302_error = 1U;
                g_dash_data.crc_302_error_count++;
                recognized = 0U;
                break;
            }

            /*
            * CRC正确，允许使用本帧数据。
            */
            g_dash_data.crc_302_error = 0U;

            /*
            * 先解析本帧温度报文的Alive Counter。
            */
            alive_counter =
                BMS_Unpack302Alive(frame);

            /*
            * 第一帧只建立基准。
            * 从第二帧开始检查序号是否连续。
            */
            if(g_dash_data.received_302 != 0U)
            {
                expected_alive =
                    (uint8_t)(
                        (g_dash_data.last_302_alive + 1U) &
                        CAN_302_ALIVE_MASK);

                if(alive_counter != expected_alive)
                {
                    g_dash_data.alive_302_error = 1U;
                    g_dash_data.alive_302_error_count++;
                }
                else
                {
                    g_dash_data.alive_302_error = 0U;
                }
            }
            else
            {
                g_dash_data.alive_302_error = 0U;
            }

            /*
            * 使用当前有效帧作为下一次比较基准。
            */
            g_dash_data.last_302_alive =
                alive_counter;

            /*
            * CRC和序号处理完成后才更新温度。
            * 序号跳变不代表温度内容一定错误，
            * 所以CRC正确时仍然采用当前温度。
            */
            g_dash_data.temp =
                frame->data[0];

            g_dash_data.valid = 1U;
            g_dash_data.received_302 = 1U;
            g_dash_data.last_302_rx_tick = rx_tick;

            // 收到CRC正确的温度报文，立即恢复温度通信状态。
            g_dash_data.comm_302_timeout = 0U;

            break;
        }

        //------------------------------------------------------
        // 故障码
        //------------------------------------------------------
            case CAN_ID_BMS_FAULT:
            {
                /*
                * 故障报文必须完整包含8个字节。
                * 长度不正确时，不能读取完整CRC。
                */
                if (frame->dlc != 8U)
                {
                    g_dash_data.crc_303_error = 1U;
                    g_dash_data.crc_303_error_count++;
                    recognized = 0U;
                    break;
                }

                /*
                * 保存发送端携带的CRC，
                * 再根据Byte0～Byte6重新计算。
                */
                g_dash_data.last_303_received_crc =
                    frame->data[7];

                g_dash_data.last_303_calculated_crc =
                    BMS_CalculateCRC8(frame->data, 7U);

                /*
                * CRC不相等，说明当前故障内容不可信。
                * 不允许错误报文设置或清除故障显示。
                */
                if (g_dash_data.last_303_calculated_crc !=
                    g_dash_data.last_303_received_crc)
                {
                    g_dash_data.crc_303_error = 1U;
                    g_dash_data.crc_303_error_count++;
                    recognized = 0U;
                    break;
                }

                // 能运行到这里，说明报文长度和CRC都正确。
                g_dash_data.crc_303_error = 0U;

                // 取出当前故障报文自己的Alive Counter。
                alive_counter =
                    BMS_Unpack303Alive(frame);

                /*
                * 第一帧只建立序号基准；
                * 第二帧开始检查是否等于上一帧加1。
                */
                if (g_dash_data.received_303 != 0U)
                {
                    expected_alive =
                        (uint8_t)(
                            (g_dash_data.last_303_alive + 1U) &
                            CAN_303_ALIVE_MASK);

                    if (alive_counter != expected_alive)
                    {
                        g_dash_data.alive_303_error = 1U;
                        g_dash_data.alive_303_error_count++;
                    }
                    else
                    {
                        g_dash_data.alive_303_error = 0U;
                    }
                }
                else
                {
                    g_dash_data.alive_303_error = 0U;
                }

                /*
                * CRC正确后，当前帧可以作为新的序号基准。
                * 即使发生跳号，当前故障内容仍然可以使用。
                */
                g_dash_data.last_303_alive =
                    alive_counter;

                /*
                * 所有完整性检查结束后，
                * 才允许更新OLED使用的活动故障掩码。
                */
                g_dash_data.fault =
                    BMS_UnpackFault(frame);

                g_dash_data.valid = 1U;
                g_dash_data.received_303 = 1U;
                g_dash_data.last_303_rx_tick = rx_tick;
                // 收到CRC正确的故障报文，立即恢复故障通信状态。
                g_dash_data.comm_303_timeout = 0U;

                break;
            }

        default:
            recognized = 0U;
            break;
    }

    // 未识别报文不能刷新BMS总通信时间，避免无关流量掩盖通信丢失。
    if (recognized != 0U)
    {
        g_dash_data.last_rx_tick = rx_tick;
    }
}


/*
 * Dash_UpdateCommStatus — 更新各周期报文的独立超时状态。
 *
 * 0x301每100ms发送，连续500ms没有有效帧则超时；
 * 0x302每500ms发送，连续1500ms没有有效帧则超时；
 * 0x303每100ms发送，连续500ms没有有效帧则超时。
 *
 * 三类报文分别使用自己的接收时间，
 * 其中一种报文到达不能掩盖另一种报文丢失。
 *
 * 所有时间差都使用无符号减法，
 * 因此能够处理32位毫秒计数器回绕。
 */
void Dash_UpdateCommStatus(uint32_t now_ms)
{
    uint8_t previous_timeout;
    uint8_t current_timeout;

    previous_timeout = g_dash_data.comm_301_timeout;

    if (g_dash_data.received_301 == 0U)
    {
        current_timeout =
            (now_ms >= CAN_301_TIMEOUT_MS) ? 1U : 0U;
    }
    else
    {
        current_timeout =
            ((uint32_t)(now_ms - g_dash_data.last_301_rx_tick) >=
             CAN_301_TIMEOUT_MS) ? 1U : 0U;
    }

    g_dash_data.comm_301_timeout = current_timeout;

    // 只在正常变为超时时启动计时，不能在每次刷新时反复重新计时。
    if ((current_timeout != 0U) && (previous_timeout == 0U))
    {
        g_dash_data.comm_301_notice_active = 1U;
        g_dash_data.comm_301_notice_since_ms = now_ms;
    }

    // 通信仍超时时不撤提示；通信恢复且显示满2秒以后才撤掉。
    if ((current_timeout == 0U) &&
        (g_dash_data.comm_301_notice_active != 0U) &&
        ((uint32_t)(now_ms - g_dash_data.comm_301_notice_since_ms) >=
         CAN_301_NOTICE_HOLD_MS))
    {
        g_dash_data.comm_301_notice_active = 0U;
    }

   /*
    * 独立判断温度报文是否超时。
    * 0x301或0x303继续到达，也不能刷新这个时间。
    */
    if (g_dash_data.received_302 == 0U)
    {
        /*
        * 上电后可能还没收到第一帧温度报文，
        * 因此先等待1500ms。
        */
        g_dash_data.comm_302_timeout =
            (now_ms >= CAN_302_TIMEOUT_MS) ? 1U : 0U;
    }
    else
    {
        /*
        * 无符号减法也能处理32位毫秒计数器回绕。
        */
        g_dash_data.comm_302_timeout =
            ((uint32_t)(now_ms -
                        g_dash_data.last_302_rx_tick) >=
            CAN_302_TIMEOUT_MS) ? 1U : 0U;
    }

        /*
        * 独立判断故障状态报文是否超时。
        * 电压或温度报文继续到达，不能刷新故障报文的时间。
        */
        if (g_dash_data.received_303 == 0U)
        {
            /*
            * 上电后还没有收到第一帧故障报文时，
            * 先留出500ms等待时间。
            */
            g_dash_data.comm_303_timeout =
                (now_ms >= CAN_303_TIMEOUT_MS) ? 1U : 0U;
        }
        else
        {
            /*
            * 收到过有效故障报文后，
            * 使用当前时间减去最后一次有效接收时间。
            */
            g_dash_data.comm_303_timeout =
                ((uint32_t)(now_ms -
                            g_dash_data.last_303_rx_tick) >=
                CAN_303_TIMEOUT_MS) ? 1U : 0U;
        }
            
}


// ============================================================
// Dash_RefreshDisplay
//
// 周期刷新OLED
//
// 由主循环每100ms调用一次；CAN接收中断只更新g_dash_data，不直接刷屏。
// ============================================================

void Dash_RefreshDisplay(void)
{
    char line[32];
    const char *fault_name;
    uint32_t now_ms;
    uint8_t status_301_unavailable;

    now_ms = GetTick();
    Dash_UpdateCommStatus(now_ms);

    /*
     * 0x301同时携带电压、SOC和运行状态。
     * 通信真正超时，或丢失提示仍在2秒保持期内，
     * 这三项数据都不应继续伪装成实时数据。
     */
    status_301_unavailable =
        ((g_dash_data.comm_301_timeout != 0U) ||
         (g_dash_data.comm_301_notice_active != 0U)) ? 1U : 0U;

    // 收到过BMS报文后又连续1000ms全部丢失，报告整条CAN通信中断。
    if ((g_dash_data.valid != 0U) &&
        ((uint32_t)(now_ms - g_dash_data.last_rx_tick) > 1000U))
    {
        OLED_Clear();
        OLED_ShowString(0,0,"BMS DASH");
        OLED_ShowString(0,2,"CAN LOST!");
        OLED_Refresh();
        return;
    }

    //----------------------------------------------------------
    // 从未收到CAN数据
    //----------------------------------------------------------

    if(g_dash_data.valid == 0)
    {
        OLED_Clear();

        OLED_ShowString(0,0,"BMS DASH");
        OLED_ShowString(0,2,"Waiting CAN...");

        OLED_Refresh();
			 g_dash_data.valid = 0;  // 加这行，清标志，允许下次更新

        return;
    }

    //----------------------------------------------------------
    // CAN超时检测
    //----------------------------------------------------------

    //----------------------------------------------------------
    // 正常显示
    //----------------------------------------------------------

    OLED_Clear();

    OLED_ShowString(2,0,"=== BMS DASH ===");

    //----------------------------------------------------------
    // 充放电状态
    //----------------------------------------------------------

    if (status_301_unavailable != 0U)
    {
        OLED_ShowString(0,1,"STATUS:CAN LOST");
    }
    else
    {
        switch(g_dash_data.chg_state)
        {
            case CHG_STATE_DISCHARGE:

                OLED_ShowString(0,1,"DISCHARGE");

                break;

            case CHG_STATE_CHARGE:

                OLED_ShowString(0,1,"CHARGING");

                break;

            default:

                OLED_ShowString(0,1,"STANDBY");

                break;
        }
    }


    //----------------------------------------------------------
    // 电压
    //
    // cell_mv单位：
    // mV
    //
    // 4150
    // ->
    // 4.150V
    //----------------------------------------------------------

    if (status_301_unavailable != 0U)
    {
        OLED_ShowString(0,2,"Cell:CAN LOST");
    }
    else
    {
        snprintf(
            line,
            sizeof(line),
            "Cell:%d.%03dV",
            g_dash_data.cell_mv / 1000U,
            g_dash_data.cell_mv % 1000U);

        OLED_ShowString(0,2,line);
    }

    
    //----------------------------------------------------------
    // 温度
    //
    // 通信正常时显示最近收到的温度；
    // 连续1500ms没有收到有效温度报文时，
    // 不再把旧温度伪装成实时温度。
    //----------------------------------------------------------

    if (g_dash_data.comm_302_timeout != 0U)
    {
        OLED_ShowString(0,3,"Temp:CAN LOST");
    }
    else
    {
        snprintf(
            line,
            sizeof(line),
            "Temp:%dC",
            g_dash_data.temp);

        OLED_ShowString(0,3,line);
    }

    //----------------------------------------------------------
    // SOC
    //----------------------------------------------------------

    if (status_301_unavailable != 0U)
    {
        OLED_ShowString(0,4,"SOC:CAN LOST");
    }
    else
    {
        snprintf(
            line,
            sizeof(line),
            "SOC:%d%%",
            g_dash_data.soc);

        OLED_ShowString(0,4,line);
    }

//----------------------------------------------------------
// 故障显示
//
// 通信正常时显示当前活动故障；
// 故障报文超时时保留最后一次可信故障掩码，
// 同时明确说明该数据已经停止更新。
//----------------------------------------------------------

    if (g_dash_data.comm_303_timeout != 0U)
    {
        if (g_dash_data.received_303 != 0U)
        {
            /*
            * 曾经收到过有效故障报文：
            * 保留最后一次可信掩码，但标记通信已经丢失。
            */
            snprintf(
                line,
                sizeof(line),
                "F:%04X CAN LOST",
                (unsigned int)g_dash_data.fault);

            OLED_ShowString(0,5,line);
        }
        else
        {
            /*
            * 上电后从未收到有效故障报文，
            * 此时没有可信的历史故障掩码。
            */
            OLED_ShowString(0,5,"Fault:CAN LOST");
        }
    }
    else if (g_dash_data.fault == 0U)
    {
        OLED_ShowString(0,5,"Fault:NONE");
    }
    else
    {
        /*
         * 一行OLED无法同时写下所有故障名称，所以显示完整掩码，
         * 再按安全优先级显示一个主要故障简称；掩码仍保留全部故障信息。
         */
        if((g_dash_data.fault & CAN_FAULT_SELF_CHECK) != 0U)
            fault_name = "SELF";
        else if((g_dash_data.fault & CAN_FAULT_ADC_TIMEOUT) != 0U)
            fault_name = "ADC";
        else if((g_dash_data.fault & CAN_FAULT_SENSOR_DATA) != 0U)
            fault_name = "DATA";
        else if((g_dash_data.fault & CAN_FAULT_NTC_OPEN) != 0U)
            fault_name = "NTCO";
        else if((g_dash_data.fault & CAN_FAULT_NTC_SHORT) != 0U)
            fault_name = "NTCS";
        else if((g_dash_data.fault & CAN_FAULT_OT) != 0U)
            fault_name = "OT";
        else if((g_dash_data.fault & CAN_FAULT_OV) != 0U)
            fault_name = "OV";
        else if((g_dash_data.fault & CAN_FAULT_UV) != 0U)
            fault_name = "UV";
        else
            fault_name = "OTHER";

        snprintf(line,
                 sizeof(line),
                 "F:%04X %s",
                 (unsigned int)g_dash_data.fault,
                 fault_name);
        OLED_ShowString(0,5,line);
    }

    //----------------------------------------------------------
    // 刷新OLED
    //----------------------------------------------------------

    OLED_Refresh();
}

