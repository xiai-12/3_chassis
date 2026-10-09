//
// Created by zhaol on 2026/9/14.
//
#include "m3508.h"

#include <string.h>

M3508_t m3508_data[4];

// m3508 发送头
FDCAN_TxHeaderTypeDef m3508_tx_header;

/*
* M3508减速电机套装参数
    额定电压：24V
    空载转速：482rpm ==> 482rpm * 19.2032f(减速比) = 9255.9424 rpm 电机原始转速
    持续最大扭矩：3N·m
    3N·m下最大转速：469rpm ==> 469rpm * 19.2032 = 9006。3008 rpm
    使用环境温度：0-50° C
 */

/*
 * 改电机须知：
 *  1.改电机 id号，main.c ---- tx_header.Identifier = 0x1FF;
 *  2.接收回调的 id号，m3508.c ---- (hdr.Identifier == 0x205 && hdr.IdType == FDCAN_STANDARD_ID)
 *  3.输出数组位置号，motor_control.c ---- int16_t amp[4] = {0};  amp[0] =  0.80f * (int16_t)mode1_pid.Out + 0.20f * amp[0];
 */

PID_Improve speed_im = {
    .integral_limit = 1,.integral_limit_val = 50000,
    .variable_integal = 0,.variable_integal_k = 0.003f,
    .deriv_on_meas = 1,
    .deriv_filter = 1, .deriv_filter_alpha = 0.8f,
    .out_dead_zone = 1, .out_dead_zone_val = 30.0f
};

/*
 * 将四个电机的输出电流原始值  映射到  8个uint8_t 字节
 */
static  void C620_RawToByte(const int16_t raw_data[4],uint8_t data[8])
{
    for (int i = 0; i < 4; i++)
    {
        const uint16_t tmp = (uint16_t)raw_data[i];
        data[i * 2] = (uint8_t)(tmp >> 8);
        data[i * 2 + 1] = (uint8_t)(tmp & 0xFF);
    }
}


HAL_StatusTypeDef M3508_Init(void)
{
    // tx_header发送头配置
    m3508_tx_header.IdType = FDCAN_STANDARD_ID;
    m3508_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    m3508_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    m3508_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    m3508_tx_header.Identifier = 0x200;
    m3508_tx_header.MessageMarker = 0x01;
    m3508_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    m3508_tx_header.TxFrameType= FDCAN_DATA_FRAME;

    // can过滤器，过滤 0x201 - 0x208，c620电调id
    FDCAN_FilterTypeDef filter;
    filter.IdType  = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x201;
    filter.FilterID2 = 0x208;

    // 开启接收过滤器
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // 开启CAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // 开启CAN回调中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE ,0) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (int i = 0; i < 4; i++)
    {
        PID_Init(&m3508_data[i].speed_pid,6.0f,0.4f,0.0f,15000,-15000,&speed_im);
    }

    return HAL_OK;
}

// m3508电机速度环
// speed_tar 为外部的速度
int16_t M3508_speed_ctl(M3508_t* motor,float speed_tar)
{
    motor->speed_pid.Tar= speed_tar * Reduce_ration;// 单位 rpm  19.2032-减速比
    motor->speed_pid.Act = 0.80f * (float)motor->speed + 0.20f * motor->speed_pid.Act;
    PID_Controllor(&motor->speed_pid);
    float amp =  (int16_t)(0.95f * motor->speed_pid.Out + 0.05f * motor->speed_pid.Out_last);
    motor->speed_pid.Out_last = amp;
    return (int16_t)amp;
}


// 电机状态清楚
void M3508_Reset(M3508_t* motor)
{
    motor->amp = 0;
    motor->speed = 0;
    motor->angle = 0;
    motor->temp = 0;
    motor->turns = 0;
    motor->speed_pid.Tar = 0.0f;
    motor->speed_pid.Out = 0.0f;
    motor->speed_pid.Act = 0.0f;
}

// m3508电机发送函数， amp[4] 为 要发送的电流值(-16384 - 16384)
HAL_StatusTypeDef M3508_SendData(const FDCAN_TxHeaderTypeDef* can_tx_header,int16_t amp[4])
{
    uint8_t data[8] = {0};
    if ((can_tx_header == NULL) || (amp == NULL)) return HAL_ERROR;
    C620_RawToByte(amp,data);
    return  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,can_tx_header,data);
}


// m3508电机回调函数
void  m3508_motor_callback(FDCAN_RxHeaderTypeDef* hdr,M3508_t* motor,uint8_t* hdr_data)
{
    motor->rx_header = *hdr;
    motor->rx_data_ptr = hdr_data;
    motor->angle = (uint16_t)((hdr_data[0] << 8) | hdr_data[1]);
    motor->speed = (int16_t)((hdr_data[2] << 8) | hdr_data[3]);
    motor->amp   = (int16_t)((hdr_data[4] << 8) | hdr_data[5]);
    motor->temp  = hdr_data[6];
    // 判断电机是否转圈
    int32_t delta_angle = motor->angle - motor->angle_last;
    if (delta_angle >  4096) delta_angle -= 8192;
    if (delta_angle < -4096) delta_angle += 8192;
    motor->turns += (float)delta_angle/8192.0f;
    motor->angle_last = motor->angle;
}