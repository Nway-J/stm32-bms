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
//
// CAN报文：
// 0x301
//   Byte0~1 Cell Voltage(mV)
//   Byte2    SOC(%)
//   Byte3    Charge State
//
// 0x302
//   Byte0    Temperature
//
// 0x303
//   Byte0    Fault Code
//
// ============================================================

#include "app_dash.h"
#include "bsp_oled.h"
#include "bsp_tim.h"

#include <stdio.h>
#include <string.h>


// ============================================================
// 全局显示数据
// ============================================================

BMS_DisplayData_t g_dash_data = {0};


// ============================================================
// Dash_Init
//
// 仪表初始化
// ============================================================

void Dash_Init(void)
{
    memset(&g_dash_data, 0, sizeof(g_dash_data));

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
    if(frame == NULL)
        return;

    switch(frame->id)
    {
        //------------------------------------------------------
        // 电压 + SOC + 状态
        //------------------------------------------------------
        case 0x301:
        {
            g_dash_data.cell_mv =
                ((uint16_t)frame->data[0] << 8) |
                 frame->data[1];

            g_dash_data.soc =
                frame->data[2];

            g_dash_data.chg_state =
                frame->data[3];

            g_dash_data.valid = 1;

            break;
        }

        //------------------------------------------------------
        // 温度
        //------------------------------------------------------
        case 0x302:
        {
            g_dash_data.temp =
                frame->data[0];

            g_dash_data.valid = 1;

            break;
        }

        //------------------------------------------------------
        // 故障码
        //------------------------------------------------------
        case 0x303:
        {
            g_dash_data.fault =
                frame->data[0];

            g_dash_data.valid = 1;

            break;
        }

        default:
            break;
    }

    //----------------------------------------------------------
    // 记录最后一次收到CAN时间
    //----------------------------------------------------------

    g_dash_data.last_rx_tick = GetTick();
}


// ============================================================
// Dash_RefreshDisplay
//
// 周期刷新OLED
//
// 建议：
// Scheduler 500ms调用一次
// ============================================================

void Dash_RefreshDisplay(void)
{
    char line[32];

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

    if(GetTick() - g_dash_data.last_rx_tick > 1000)
    {
        OLED_Clear();

        OLED_ShowString(0,0,"BMS DASH");
        OLED_ShowString(0,2,"CAN LOST!");

        OLED_Refresh();

        return;
    }

    //----------------------------------------------------------
    // 正常显示
    //----------------------------------------------------------

    OLED_Clear();

    OLED_ShowString(0,0,"=== BMS DASH ===");

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

    snprintf(
        line,
        sizeof(line),
        "Cell:%d.%03dV",
        g_dash_data.cell_mv / 100,
        g_dash_data.cell_mv % 100);

    OLED_ShowString(0,1,line);

    //----------------------------------------------------------
    // 充放电状态
    //----------------------------------------------------------

    switch(g_dash_data.chg_state)
    {
        case CHG_STATE_DISCHARGE:

            OLED_ShowString(0,2,"DISCHARGE");

            break;

        case CHG_STATE_CHARGE:

            OLED_ShowString(0,2,"CHARGING");

            break;

        default:

            OLED_ShowString(0,2,"STANDBY");

            break;
    }

    //----------------------------------------------------------
    // 温度
    //----------------------------------------------------------

    snprintf(
        line,
        sizeof(line),
        "Temp:%dC",
        g_dash_data.temp);

    OLED_ShowString(0,3,line);

    //----------------------------------------------------------
    // SOC
    //----------------------------------------------------------

    snprintf(
        line,
        sizeof(line),
        "SOC:%d%%",
        g_dash_data.soc);

    OLED_ShowString(0,4,line);

    //----------------------------------------------------------
    // 故障显示
    //----------------------------------------------------------

    if(g_dash_data.fault == 0)
    {
        OLED_ShowString(0,5,"Fault:NONE");
    }
    else
    {
        char fault_str[20] = "";

        if(g_dash_data.fault & 0x01)
        {
            strcat(fault_str,"OVR ");
        }

        if(g_dash_data.fault & 0x02)
        {
            strcat(fault_str,"UND ");
        }

        if(g_dash_data.fault & 0x04)
        {
            strcat(fault_str,"OVT ");
        }

        OLED_ShowString(0,5,"FAULT:");

        OLED_ShowString(48,5,fault_str);
    }

    //----------------------------------------------------------
    // 刷新OLED
    //----------------------------------------------------------

    OLED_Refresh();
}

