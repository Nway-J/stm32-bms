/*
 * 文件名称：can_protocol.h
 * 模块作用：定义ECU-A与ECU-B共同遵守的CAN报文格式和编解码方法。
 *
 * 0x301测量状态报文布局：
 * data[0]~data[1] = 电池电压，单位mV，高字节在前；
 * data[2] = SOC；data[3] = 监测状态；
 * data[4]低四位 = Alive Counter（0~15循环）；高四位保留；
 * data[5]~data[6] = 保留，发送时固定清零；
 * data[7] = Byte0~Byte6的CRC8校验值。
 *
 * 0x302温度状态报文布局：
 * data[0] = 温度；
 * data[1]~data[3] = 保留，发送时固定清零；
 * data[4]低四位 = Alive Counter（0~15循环）；
 * data[5]~data[6] = 保留，发送时固定清零；
 * data[7] = Byte0~Byte6的CRC8校验值。
 * 
 * 
 * 0x303故障状态报文布局：
 * data[0] = 16位活动故障掩码高8位；
 * data[1] = 16位活动故障掩码低8位；
 * data[2]~data[3] = 保留，发送时固定清零；
 * data[4]低四位 = Alive Counter（0~15循环）；
 * data[5]~data[6] = 保留，发送时固定清零；
 * data[7] = Byte0~Byte6的CRC8校验值。
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "stm32f10x.h"

#define CAN_ID_BMS_VOLTAGE    0x301
#define CAN_ID_BMS_TEMP       0x302
#define CAN_ID_BMS_FAULT      0x303
#define CAN_ID_CTRL_CMD       0x100

// 0x303活动故障掩码定义，ECU-B使用这些位解释故障名称。
#define CAN_FAULT_OV            ((uint16_t)0x0001U)
#define CAN_FAULT_UV            ((uint16_t)0x0002U)
#define CAN_FAULT_OT            ((uint16_t)0x0004U)
#define CAN_FAULT_NTC_OPEN      ((uint16_t)0x0008U)
#define CAN_FAULT_NTC_SHORT     ((uint16_t)0x0010U)
#define CAN_FAULT_SENSOR_DATA   ((uint16_t)0x0020U)
#define CAN_FAULT_ADC_TIMEOUT   ((uint16_t)0x0040U)
#define CAN_FAULT_SELF_CHECK    ((uint16_t)0x0080U)

typedef struct
{
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;

#define CHG_STATE_IDLE       0
#define CHG_STATE_DISCHARGE  1
#define CHG_STATE_CHARGE     2

#define CAN_301_ALIVE_MASK   0x0FU

//温度报文Alive Counter同样使用低四位，取值范围为0～15。
#define CAN_302_ALIVE_MASK 0x0FU

// 故障报文拥有独立的四位Alive Counter，取值范围为0～15。
#define CAN_303_ALIVE_MASK   0x0FU

/*
 * BMS_CalculateCRC8 — 计算应用层CRC8。
 *
 * 参数data：
 *   指向需要计算CRC的数据首地址。
 *
 * 参数length：
 *   参与计算的数据字节数。
 *
 * 返回值：
 *   计算完成的8位CRC。
 *
 * 当前采用CRC-8/SAE-J1850参数：
 *   Polynomial = 0x1D
 *   Init       = 0xFF
 *   XorOut     = 0xFF
 */
static inline uint8_t BMS_CalculateCRC8(
    const uint8_t *data,
    uint8_t length)
{
    uint8_t crc = 0xFFU;
    uint8_t byte_index;
    uint8_t bit_index;

    for(byte_index = 0U;
        byte_index < length;
        byte_index++)
    {
        /*
         * 先把当前数据字节与CRC异或，
         * 让这个字节的每一位参与后续计算。
         */
        crc ^= data[byte_index];

        /*
         * 一个字节有8位，所以每个数据字节处理8次。
         */
        for(bit_index = 0U;
            bit_index < 8U;
            bit_index++)
        {
            /*
             * 如果CRC最高位为1：
             * 左移后再与多项式0x1D异或。
             */
            if((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)(
                    (crc << 1U) ^ 0x1DU);
            }
            else
            {
                /*
                 * 最高位为0时只需要左移。
                 */
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    /*
     * SAE-J1850最后还要与0xFF异或。
     */
    return (uint8_t)(crc ^ 0xFFU);
}

// 0x301电压状态报文打包
static inline void BMS_PackVoltage(CAN_Frame_t *f,
                                   uint16_t mv,
                                   uint8_t soc,
                                   uint8_t chg_state,
                                   uint8_t alive_counter)
{
    f->id = CAN_ID_BMS_VOLTAGE;
    f->dlc = 8;

    f->data[0] = (mv >> 8) & 0xFF; 
    f->data[1] = mv & 0xFF;

    f->data[2] = soc; 
    f->data[3] = chg_state;

    f->data[4] = alive_counter & CAN_301_ALIVE_MASK;
    
   /*
    * Byte5和Byte6暂时没有业务数据，
    * 参与CRC计算前必须固定清零。
    */
    f->data[5] = 0U;
    f->data[6] = 0U;

   /*
    * CRC覆盖Byte0～Byte6，共7个字节。
    * 计算结果放在最后一个字节Byte7。
    */
    f->data[7] =
        BMS_CalculateCRC8(f->data, 7U);
}

// 0x302温度状态报文打包
/*
 * BMS_PackTemp — 组装温度状态报文。
 *
 * Byte0：温度；
 * Byte1～3：保留；
 * Byte4低四位：温度报文Alive Counter；
 * Byte5～6：保留；
 * Byte7：Byte0～Byte6的CRC8。
 */
static inline void BMS_PackTemp(
    CAN_Frame_t *f,
    uint8_t temp,
    uint8_t alive_counter)
{
    f->id = CAN_ID_BMS_TEMP;
    f->dlc = 8U;

    f->data[0] = temp;

    /*
     * 当前没有其他温度业务数据，
     * 所以Byte1～Byte3固定清零。
     */
    f->data[1] = 0U;
    f->data[2] = 0U;
    f->data[3] = 0U;

    /*
     * Byte4只保留计数器低四位。
     */
    f->data[4] =
        (uint8_t)(alive_counter &
                  CAN_302_ALIVE_MASK);

    f->data[5] = 0U;
    f->data[6] = 0U;

    /*
     * 所有前7个字节填写完成后，
     * 才能计算最后的CRC。
     */
    f->data[7] =
        BMS_CalculateCRC8(f->data, 7U);
}



// 只取协议规定的低四位，避免未来使用高四位时干扰Alive Counter。
static inline uint8_t BMS_Unpack301Alive(const CAN_Frame_t *f)
{
    return (uint8_t)(f->data[4] & CAN_301_ALIVE_MASK);
}

/*
 * 从温度状态报文的Byte4低四位
 * 提取该报文自己的Alive Counter。
 */
static inline uint8_t BMS_Unpack302Alive(
    const CAN_Frame_t *f)
{
    return (uint8_t)(
        f->data[4] &
        CAN_302_ALIVE_MASK);
}

/*
 * 从故障状态报文的Byte4低四位
 * 提取该报文自己的Alive Counter。
 */
static inline uint8_t BMS_Unpack303Alive(
    const CAN_Frame_t *f)
{
    return (uint8_t)(
        f->data[4] &
        CAN_303_ALIVE_MASK);
}


/*
 * BMS_PackFault — 组装故障状态报文。
 *
 * fault_mask保存当前所有活动故障；
 * alive_counter表示本报文的发送顺序；
 * Byte7用于检查前7个字节是否被意外改变。
 */
static inline void BMS_PackFault(
    CAN_Frame_t *f,
    uint16_t fault_mask,
    uint8_t alive_counter)
{
    f->id = CAN_ID_BMS_FAULT;
    f->dlc = 8U;

    // 16位故障掩码采用高字节在前。
    f->data[0] =
        (uint8_t)((fault_mask >> 8) & 0x00FFU);

    f->data[1] =
        (uint8_t)(fault_mask & 0x00FFU);

    // 当前没有其他业务数据，保留字节必须固定清零。
    f->data[2] = 0U;
    f->data[3] = 0U;

    // Byte4只使用低四位保存故障报文自己的序号。
    f->data[4] =
        (uint8_t)(alive_counter &
                  CAN_303_ALIVE_MASK);

    f->data[5] = 0U;
    f->data[6] = 0U;

    // 必须先填写Byte0～Byte6，最后才能计算CRC。
    f->data[7] =
        BMS_CalculateCRC8(f->data, 7U);
}


/* 按相同顺序把0x303中的两个字节还原成16位故障掩码。 */
static inline uint16_t BMS_UnpackFault(const CAN_Frame_t *f)
{
    return (uint16_t)(((uint16_t)f->data[0] << 8) |
                      (uint16_t)f->data[1]);
}

#endif
