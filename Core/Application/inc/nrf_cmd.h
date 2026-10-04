//
// Created by zhaol on 2026/10/2.
//

#ifndef INC_3_CHASSIS_NRF_CMD_H
#define INC_3_CHASSIS_NRF_CMD_H

#include "main.h"
#include "Nrf/nrf24.h"
#include "Nrf/Control_slave.h"
#include "Usart/Serial.h"
#include "Chassis/chassis.h"
#include "math.h"


#define STICK_DEADZONE  0.1f       /* 归一化死区，防回中不准自己跑 */

/* ★ 方向符号：实测不对就单独翻某一个，别全翻 */
#define VX_SIGN   (1.0f)           /* 左摇杆前后 -> vx */
#define VY_SIGN   (1.0f)           /* 左摇杆左右 -> vy */
#define W_SIGN    (1.0f)           /* 右摇杆左右 -> w  */

/* 速度上限（先给保守值，跑顺了再往上加） */
#define VX_MAX    1.0f              /* m/s */
#define VY_MAX    1.0f              /* m/s */
#define W_MAX     2.0f              /* rad/s */

/* 轮子线速度预算：转子 9255rpm ÷19.2032 ÷9.5493 ×0.075m = 3.785 m/s，留 15% 余量 */
#define WHEEL_LIN_BUDGET   (3.785f * 0.85f)

typedef struct
{
    float   vx;     /* m/s，机体坐标系，向前为正 */
    float   vy;     /* m/s，机体坐标系，向左为正 */
    float   w;      /* rad/s，正值为顺时针（俯视） */
    uint16_t keys;  // 滑杆按键
    uint16_t button; // 按键
    uint8_t link;   /* 1 = 遥控器在线（200 ms 内有包） */
} Nrf_ChassisCmd_t;

extern Nrf_ChassisCmd_t nrf_chassis_cmd;

void Nrf_UpdateChassisCmd(void);

#endif //INC_3_CHASSIS_NRF_CMD_H
