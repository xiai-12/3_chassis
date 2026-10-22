#ifndef ROBOCON_TEST_MOTOR_CONTROL_H
#define ROBOCON_TEST_MOTOR_CONTROL_H

#include "main.h"
#include "usart.h"
#include "PID/pid.h"
#include "Dji/m3508.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "string.h"
#include "Usart/Serial.h"
#include "Key/key.h"
#include "Chassis/chassis.h"
#include "nrf_cmd.h"
#include "Filter/filter.h"
#include "Chassis/chassis.h"
#include "WS2812/ws2812.h"
#include "TJC/tjc_uart_hmi.h"


// 电机运动模式选择
typedef  enum MOTOR_MODE
{
    MODE_NONE = 0,
    MODE_1 =1, // 定速
    MODE_2 =2, // 定位置
    MODE_3 =3, // 串级定位置
}MOTOR_MODE;

// 是否开启调试模式
typedef  enum MOTOR_DEBUG_MODE
{
    DEBUG_TRUE = 1,
    DEBUG_FALSE = 0,
}MOTOR_DEBUG_MODE;

// 串口命令解析接收结构体
typedef struct Serial_CMD_Struct
{
    volatile MOTOR_MODE motor_mode;      // 模式选择
    volatile MOTOR_DEBUG_MODE debug_mode;// 是否开启调试
    float speed_tar;                     // 目标速度
    float location_tar;                  // 目标位置
    float angle;                         // 角度
    float kt;                            // 前馈系数
    float x;                             // x
    float y;                             // y
    float w;                             // 角速度
    float kp;
    float ki;
    float kd;
}Serial_CMD;


// 视觉命令解析接收结构体
typedef struct Vision_CMD_Structor
{
    float vx;
    float vy;
    float w;

    float pos_x;
    float pos_y;
    float yaw;
}Vision_CMD;

// 码盘+imu数据接收体
typedef struct Encoder_imu_Structor
{
    float x_count;          // x原始值
    float y_count;          // y原始值
    float x_turns;          // x圈数
    float y_turns;          // y圈数
    float x_distance;       // x距离，m
    float y_distance;       // y距离，m
    float yaw;              // 航向角
    float yaw_nor;          // 航向角(归一化)
}Encoder_imu_CMD;

// pid结构体可外部引用
extern Serial_CMD Serial_cmd_structor;
extern Vision_CMD Vision_cmd_structor;
extern Encoder_imu_CMD Encoder_imu_cmd_structor;

// 底盘位置模式位置环
extern  PID_Structor chassis_pos_vx;
extern  PID_Structor chassis_pos_vy;

void Parse_serial_line(char *cmd);
void Motor_Reset(void);
void Parse_vision_line(const uint8_t* cmd, uint16_t len);
void Parse_location_line(const uint8_t* cmd,uint16_t len);
void TJC_Parse(uint8_t *buf, uint16_t len);

#endif //ROBOCON_TEST_MOTOR_CONTROL_H
