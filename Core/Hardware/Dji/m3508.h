//
// Created by zhaol on 2026/9/14.
//

#ifndef INC_1_CAN_3508_M3508_H
#define INC_1_CAN_3508_M3508_H

#include "main.h"
#include "PID/pid.h"

#define Reduce_ration 19.2032f;

// m3508 电机结构体
typedef struct M3508_Motor
{
    FDCAN_RxHeaderTypeDef rx_header; // 电机can通信返回值接收头
    uint8_t* rx_data_ptr;            // 电机数组接受数组指针
    uint16_t angle_last;
    uint16_t angle;                  // 电机旋转角度
    int16_t speed;                   // 电机旋转速度
    int16_t amp;                     // 电机旋转电流值
    uint8_t temp;                    // 电机旋转温度
    float turns;                     // 电机旋转圈数
    PID_Structor speed_pid;          // 单电机速度环
}M3508_t;

enum M3508_index
{
    Motor1_3508,
    Motor2_3508,
    Motor3_3508,
    Motor4_3508,
};

extern FDCAN_HandleTypeDef hfdcan1;
extern M3508_t m3508_data[3];
extern FDCAN_TxHeaderTypeDef m3508_tx_header;

HAL_StatusTypeDef M3508_Init(void);
void M3508_Reset(M3508_t* motor);
int16_t M3508_speed_ctl(M3508_t* motor,float speed_tar);
HAL_StatusTypeDef M3508_SendData(const FDCAN_TxHeaderTypeDef* tx_header,int16_t amp[4]);
void  m3508_motor_callback(FDCAN_RxHeaderTypeDef* hdr,M3508_t* motor,uint8_t* hdr_data);


#endif //INC_1_CAN_3508_M3508_H
