/**
 * @file    Chassis.h
 * @author  Icol_Lee (icolboom4@gmail.com)
 * @brief   底盘控制
 * @version 0.1
 * @date    2026/02/24
 */

#ifndef USER_CHASSIS_H
#define USER_CHASSIS_H

#include <stdint.h>
#include <math.h>

#define CHASSIS_S   0.0750f // 轮子半径
#define CHASSIS_R   0.3359f // 轮子距离中心的距离

#define SQRT2   (float)M_SQRT2
#define SQRT2_2 (float)M_SQRT1_2

/* 四舵轮底盘 */
#if defined(SWERVE_CHASSIS)
#define WHEEL_NUM     4
#define STEER_NUM     4
    /* 轮 */
    typedef struct {
        float LF;
        float RF;
        float RB;
        float LB;
    } Chassis_WheelType_t;
    /* 舵 */
    typedef struct {
        float s_LF;
        float s_RF;
        float s_RB;
        float s_LB;
    } Chassis_SteerType_t;
#else
/* 月球车底盘 */
#if defined(LUNAR_CHASSIS)
#define WHEEL_NUM       6
    typedef struct {
        float LF;
        float RF;
        float RB;
        float LB;
        float LM;
        float RM;
    } Chassis_WheelType_t;
#else
/* 四全向轮底盘 */
#define WHEEL_NUM       4
    typedef struct {
        float LF;
        float RF;
        float RB;
        float LB;
    } Chassis_WheelType_t;// rad/s
#endif
#endif

/* 坐标系 */
typedef enum {
    Chassis_Body = 0U,
    Chassis_Global
}Chassis_Frame_t;

/* 底盘状态机 */
typedef enum {
    Chassis_Speed = 0U, // 速度模式
    Chassis_Position,   // 位置模式
    Chassis_Halt        // 静止模式
}Chassis_Mode_t;

/* 姿态结构体 */
typedef struct {
    float x;
    float y;
    float yaw;
}Chassis_Pos_t;

/* 速度矢量结构体 */
typedef struct {
    float v_x;
    float v_y;
    float w;
}Chassis_Velocity_t;

typedef struct {
    Chassis_Mode_t          mode;       // 底盘状态
    Chassis_Pos_t           GlobalPos;  // 全局位置
    float                   pitch;      // 俯仰角
    Chassis_Velocity_t      GlobalVel;  // 全局速度矢量
    Chassis_Velocity_t      BodyVel;    // 机体速度矢量

    Chassis_Pos_t           targetPos;  // 目标全局姿态
    Chassis_Velocity_t      targetVel;  // 目标速度矢量

    Chassis_WheelType_t     targetWheel;// 轮子目标速度
#if defined(SWERVE_CHASSIS)
    Chassis_SteerType_t     steer;      // 轮组角度信息
#endif
}Chassis_t;

extern Chassis_t Chassis;

void Chassis_Init(void);
void Chassis_Control(Chassis_Frame_t frame, Chassis_Mode_t mode, float x, float y, float z, float para);
Chassis_Velocity_t Chassis_FK_Wheel2Body(Chassis_WheelType_t* Wheel);                       // 正运动学解算（机器人坐标系）
Chassis_Velocity_t Chassis_FK_Wheel2Global(Chassis_WheelType_t* Wheel, float Yaw);          // 正运动学解算（世界坐标系）

#endif //USER_CHASSIS_H
