//
// Created by zhaol on 2026/9/14.
//
#include "m3508.h"

#include <string.h>

M3508_t m3508_data[3];

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

/*
 * m3508 pid控制算法
 *
 *  减速比
 *  #define Reduce_ration  19.2032f
 *
 *  // m3508发送设置
    int16_t amp_zero[4] = {0};
    int16_t amp[4] = {0};

    pid改进措施结构体初始化
    // PID_Improve pid_im_none;
    //
    // PID_Improve mode1_pid_im = {
    //     .integral_limit = 1,.integral_limit_val = 50000,
    //     .variable_integal =  1,.variable_integal_k = 0.003,
    //     .deriv_on_meas = 1,
    //     .deriv_filter = 1, .deriv_filter_alpha = 0.8
    // };
    //
    // PID_Improve mode2_pid_im = {
    //     .integral_separation = 1, .integral_separation_val = 8000,
    //     .integral_limit = 1, .integral_limit_val = 80000,
    //     .deriv_on_meas = 1,
    //     .deriv_filter = 1, .deriv_filter_alpha = 0.8f,
    //     .out_dead_zone = 1, .out_dead_zone_val = 1000
    // };


 *  速度环
 *  PID_Init(&mode1_pid,8.0,0.3,6.0,15000,-15000,&mode1_pid_im);
    位置环
    PID_Init(&mode2_pid,0.70,0.02,12.0,5000,-5000,&mode2_pid_im);
    双环内环
    PID_Init(&mode3_pid_inner,8.0,0.3,6.0,15000,-15000,&mode1_pid_im);
    双环外环
    PID_Init(&mode3_pid_outer,0.4,0.00,1.0,6000,-6000,&pid_im_none);
    匀加速匀减速
    PID_Init(&mode7_pid,8.0,0.3,6.0,15000,-15000,&mode1_pid_im);
 *
 *  速度环：
    mode1_pid.Tar = Serial_cmd_structor.speed_tar * Reduce_ration;// 单位 rpm  19.2032-减速比
    mode1_pid.Act = 0.80f * (float)m3508_data.speed + 0.20f * mode1_pid.Act;
    PID_Controllor(&mode1_pid);
    amp[0] =  0.80f * (int16_t)mode1_pid.Out + 0.20f * amp[0];
    M3508_SendData(&m3508_tx_header,amp);


    位置环：
    mode2_pid.Tar = Serial_cmd_structor.location_tar *8191.0f *Reduce_ration;// 单位 圈数
    mode2_pid.Act = m3508_data.turns * 8191.0f;
    PID_Controllor(&mode2_pid);
    amp[0] = (int16_t)mode2_pid.Out;
    M3508_SendData(&m3508_tx_header,amp);

    双环，外环位置环，内环速度环
    mode3_pid_outer.Tar = Serial_cmd_structor.location_tar * 8191.0f *Reduce_ration;
    mode3_pid_outer.Act = m3508_data.turns * 8191.0f ;
    PID_Controllor(&mode3_pid_outer);
    mode3_pid_inner.Tar = mode3_pid_outer.Out;
    mode3_pid_inner.Act = (float)m3508_data.speed;
    PID_Controllor(&mode3_pid_inner);
    amp[0] = (int16_t)mode3_pid_inner.Out;
    M3508_SendData(&m3508_tx_header,amp);

    指定最大速度和加速度的匀加速匀减速运动
    TracePlane.pos_target = Serial_cmd_structor.location_tar *8191.0f *Reduce_ration;
    TracePlane.pos_act = m3508_data.turns * 8191.0f;
    Tace_Update(&TracePlane,0.005f);
    mode7_pid.Tar = TracePlane.vref;
    mode7_pid.Act = (float)m3508_data.speed;
    PID_Controllor(&mode7_pid);
    amp[0] = (int16_t)mode7_pid.Out;
    M3508_SendData(&m3508_tx_header,amp);

 */


/*
 * 将 实际电流 映射至 电调控制范围 ，从 -20A~20A 映射到 -16384 ~ 16384
 */
// static  int16_t C620_AmpToRaw(float current_a)
// {
//     float res = current_a * 16384.00f / 20.00f;
//     res += (res >= 0.0f)? 0.5f: -0.5f;
//     if (res > 16384) res = 16384.0f;
//     if (res < -16384) res = -16384.0f;
//     return (int16_t)res;
// }

PID_Improve speed_im = {
    .integral_limit = 1,.integral_limit_val = 50000,
    .variable_integal =  1,.variable_integal_k = 0.003f,
    .deriv_on_meas = 1,
    .deriv_filter = 1, .deriv_filter_alpha = 0.8f
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
    m3508_tx_header.Identifier = 0x1FF;
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

    for (int i = 0; i < 3; i++)
    {
        PID_Init(&m3508_data[i].speed_pid,8.0f,0.3f,6.0f,15000,-15000,&speed_im);
    }

    return HAL_OK;
}

// m3508电机速度环
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
}

// m3508电机发送函数， amp[4] 为 要发送的电流值(-16384 - 16384)
HAL_StatusTypeDef M3508_SendData(const FDCAN_TxHeaderTypeDef* can_tx_header,int16_t amp[4])
{
    uint8_t data[8] = {0};
    if ((can_tx_header == NULL) || (amp == NULL)) return HAL_ERROR;
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     amp_data[i] = C620_AmpToRaw(amp[i]);
    // }
    C620_RawToByte(amp,data);
    return  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,can_tx_header,data);
}


// CAN接收回调函数
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
//     static uint16_t angle_last = 0;
//     FDCAN_RxHeaderTypeDef hdr;
//     static  uint8_t hdr_data[8] = {0};
//
//     while (HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&hdr,hdr_data) == HAL_OK)
//     {
//         if (hdr.Identifier == 0x205 && hdr.IdType == FDCAN_STANDARD_ID)
//         {
//             m3508_data.rx_header = hdr;
//             m3508_data.rx_data_ptr = hdr_data;
//             angle_last = m3508_data.angle;
//             m3508_data.angle = (uint16_t)((hdr_data[0] << 8) | hdr_data[1]);
//             m3508_data.speed = (int16_t)((hdr_data[2] << 8) | hdr_data[3]);
//             m3508_data.amp   = (int16_t)((hdr_data[4] << 8) | hdr_data[5]);
//             m3508_data.temp  = hdr_data[6];
//             // 判断电机是否转圈
//             int32_t delta_angle = m3508_data.angle - angle_last;
//             if (delta_angle >  4096) delta_angle -= 8192;
//             if (delta_angle < -4096) delta_angle += 8192;
//             m3508_data.turns += (float)delta_angle/8192.0f;
//         }
//
//     }
// }

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