/**
 * @file    Chassis.c
 * @author  Icol_Lee (icolboom4@gmail.com)
 * @brief   底盘控制
 * @version 0.1
 * @date    2026/02/24
 */
#include "main.h"
#include "chassis.h"
#include "PID/pid.h"
#include "motor_control.h"

/*
 *        布局示意图：
 *                       ↑ X (前进方向)
 *                 LF      a=轮距/2      RF
 *               0 ┌─────────────────────┐ 3
 *                 │                     │
 *         Y ←     │          Z          │  b=轴距/2
 *                 │                     │
 *               1 └─────────────────────┘ 2
 *                 LB                   RB
 *
 *  9.26 规定的是 电机逆时针旋转方向为正
 *
 *  实际：
 *
 *     id2     id3
 *
 *
 *     id1     id4
 *                   y
 *                  ^
 *                  |
 *                  |
 *             x <————
 *
 */

Chassis_t Chassis;

Chassis_WheelType_t Chassis_IK_Body2Wheel(Chassis_Velocity_t* BodyVelocity);                // 逆运动学解算（机器人坐标系）
Chassis_WheelType_t Chassis_IK_Global2Wheel(Chassis_Velocity_t* GlobalVelocity, float Yaw); // 逆运动学解算（世界坐标系）
Chassis_WheelType_t Chassis_PositionControl_body(Chassis_Pos_t position);                   // 位置控制

PID_Structor chassis_pos_vx;
PID_Structor chassis_pos_vy;
PID_Improve  chassis_pos_im={
    .out_dead_zone = 1, .out_dead_zone_val = 0.02f,
    .deriv_on_meas = 1, .deriv_filter = 1, .deriv_filter_alpha =  0.80f,
    .integral_limit = 1, .integral_limit_val = 5.0f
};

/**
 * 初始化底盘参数
 */
void Chassis_Init(void) {
    /* 状态 */
    Chassis.mode = Chassis_Halt;

    /* 全局位置 */
    Chassis.GlobalPos.x = 0.0f;
    Chassis.GlobalPos.y = 0.0f;
    Chassis.GlobalPos.yaw = 0.0f;

    /* 全局速度矢量 */
    Chassis.GlobalVel.v_x = 0.0f;
    Chassis.GlobalVel.v_y = 0.0f;
    Chassis.GlobalVel.w = 0.0f;

    /* 机体速度矢量 */
    Chassis.BodyVel.v_x = 0.0f;
    Chassis.BodyVel.v_y = 0.0f;
    Chassis.BodyVel.w = 0.0f;

    /* 目标全局姿态 */
    Chassis.targetPos.x = 0.0f;
    Chassis.targetPos.y = 0.0f;
    Chassis.targetPos.yaw = 0.0f;

    /* 目标速度矢量 */
    Chassis.targetVel.v_x = 0.0f;
    Chassis.targetVel.v_y = 0.0f;
    Chassis.targetVel.w = 0.0f;

    PID_Init(&chassis_pos_vx,0.6f,0.0f,0.0f,1.0f,-1.0f,&chassis_pos_im);
    PID_Init(&chassis_pos_vy,0.6f,0.0f,0.0f,1.0f,-1.0f,&chassis_pos_im);
}

/**
 * 底盘状态机控制
 * @param frame 参考坐标系
 * @param mode  控制模式
 * @param x     x轴控制量（速度模式下为线速度m/s，位置模式下为绝对坐标X）
 * @param y     y轴控制量（速度模式下为线速度m/s，位置模式下为绝对坐标Y）
 * @param z     z轴控制量（若不为0，代表外界强制给定的自转角速度；若为0，代表角度锁死）
 * @param para  当前底盘的真实Yaw角（弧度）
 */
void Chassis_Control(Chassis_Frame_t frame, Chassis_Mode_t mode, float x, float y, float z, float para) {

    switch (mode) {
        /* 速度模式 */
        case Chassis_Speed: {
            Chassis.targetVel.v_x = x;
            Chassis.targetVel.v_y = y;
            break;
        }

        /* 位置模式 */
        case Chassis_Position: {

            chassis_pos_vx.Tar = x;
            chassis_pos_vx.Act = Chassis.GlobalPos.x;
            chassis_pos_vy.Tar = y;
            chassis_pos_vy.Act = Chassis.GlobalPos.y;

            PID_Controllor(&chassis_pos_vx);
            Chassis.targetVel.v_x =  chassis_pos_vx.Out;
            PID_Controllor(&chassis_pos_vy);
            Chassis.targetVel.v_y =  chassis_pos_vy.Out;

            // float err_x = x - Chassis.GlobalPos.x;
            // float err_y = y - Chassis.GlobalPos.y;
            // #define POS_P_FACT    0.2f   // 位置环比例
            // #define MAX_VEL_LIMIT 1.0f  // 限幅
            // Chassis.targetVel.v_x = err_x * POS_P_FACT;
            // Chassis.targetVel.v_y = err_y * POS_P_FACT;
            // if (Chassis.targetVel.v_x > MAX_VEL_LIMIT)  Chassis.targetVel.v_x = MAX_VEL_LIMIT;
            // if (Chassis.targetVel.v_x < -MAX_VEL_LIMIT) Chassis.targetVel.v_x = -MAX_VEL_LIMIT;
            // if (Chassis.targetVel.v_y > MAX_VEL_LIMIT)  Chassis.targetVel.v_y = MAX_VEL_LIMIT;
            // if (Chassis.targetVel.v_y < -MAX_VEL_LIMIT) Chassis.targetVel.v_y = -MAX_VEL_LIMIT;
        }
        break;

        /* 静止模式 */
        case Chassis_Halt: {
            Chassis.targetVel.v_x = 0.0f;
            Chassis.targetVel.v_y = 0.0f;
            Chassis.targetVel.w = 0.0f;

            Chassis.targetWheel.LF = Chassis.targetWheel.LB = 0.0f;
            Chassis.targetWheel.RF = Chassis.targetWheel.RB = 0.0f;
            return;
        }
    }

    if (fabsf(z) > 0.001f) {
        Chassis.targetVel.w = z;
        Chassis.targetPos.yaw = para;
    } else {
        float yaw_err = Chassis.targetPos.yaw - para;
        if (yaw_err >  M_PI)  yaw_err -= 2.0f * (float)M_PI;
        else if (yaw_err < -M_PI) yaw_err += 2.0f * (float)M_PI;

        float CHASSIS_W_P = 0.2f;
        Chassis.targetVel.w = yaw_err * CHASSIS_W_P;

        if (Chassis.targetVel.w > 3.0f)   Chassis.targetVel.w = 3.0f;
        if (Chassis.targetVel.w < -3.0f)  Chassis.targetVel.w = -3.0f;
    }

    if (frame == Chassis_Global || mode == Chassis_Position) {
        Chassis.targetWheel = Chassis_IK_Global2Wheel(&Chassis.targetVel, para);
    } else {
        Chassis.targetWheel = Chassis_IK_Body2Wheel(&Chassis.targetVel);
    }
}

/**
 * 逆运动学解算（机器人坐标系）
 * @param BodyVelocity  机器人坐标系速度矢量
 * @return 返回轮子解算速度
 */
Chassis_WheelType_t Chassis_IK_Body2Wheel(Chassis_Velocity_t* BodyVelocity) {
    Chassis_WheelType_t wheel;
    float v_x = BodyVelocity->v_x;
    float v_y = BodyVelocity->v_y;
    float w = BodyVelocity->w;

    wheel.LF = (-v_x * SQRT2_2 + v_y * SQRT2_2 + w * CHASSIS_R) / CHASSIS_S;
    wheel.LB = (-v_x * SQRT2_2 - v_y * SQRT2_2 + w * CHASSIS_R) / CHASSIS_S;
    wheel.RB = (+v_x * SQRT2_2 - v_y * SQRT2_2 + w * CHASSIS_R) / CHASSIS_S;
    wheel.RF = (+v_x * SQRT2_2 + v_y * SQRT2_2 + w * CHASSIS_R) / CHASSIS_S;

    return wheel;
}

/**
 * 逆运动学解算（世界坐标系）
 * @param GlobalVelocity 全局坐标系速度矢量
 * @param Yaw            底盘航偏角（世界坐标系） 单位：弧度
 * @return 返回轮子解算速度
 */
Chassis_WheelType_t Chassis_IK_Global2Wheel(Chassis_Velocity_t* GlobalVelocity, float Yaw) {
    Chassis_Velocity_t body;

    body.v_x = GlobalVelocity->v_x * cosf(Yaw) + GlobalVelocity->v_y * sinf(Yaw);
    body.v_y = -GlobalVelocity->v_x * sinf(Yaw) + GlobalVelocity->v_y * cosf(Yaw);
    body.w = GlobalVelocity->w;

    return Chassis_IK_Body2Wheel(&body);
}

/**
 * 正运动学解算（机器人坐标系）
 * @param Wheel 底盘各轮子的速度
 * @return  机器人坐标系下的底盘速度矢量
 */
Chassis_Velocity_t Chassis_FK_Wheel2Body(Chassis_WheelType_t* Wheel) {
    Chassis_Velocity_t BodyVelocity;

    float LF = Wheel->LF;
    float LB = Wheel->LB;
    float RF = Wheel->RF;
    float RB = Wheel->RB;

    BodyVelocity.v_x = -(+LF + LB - RB - RF) * CHASSIS_S * SQRT2 / 4.0f;
    BodyVelocity.v_y = -(-LF + LB + RB - RF) * CHASSIS_S * SQRT2 / 4.0f;
    BodyVelocity.w = (LF + LB + RB + RF) * CHASSIS_S / (4.0f * CHASSIS_R);

    return BodyVelocity;
}

/**
 * 正运动学解算（世界坐标系）
 * @param Wheel 底盘各轮子的速度
 * @param Yaw   底盘航偏角（世界坐标系） 单位：弧度
 * @return  世界坐标系下的底盘速度矢量
 */
Chassis_Velocity_t Chassis_FK_Wheel2Global(Chassis_WheelType_t* Wheel, float Yaw) {
    Chassis_Velocity_t GlobalVelocity;

    Chassis_Velocity_t BodyVelocity = Chassis_FK_Wheel2Body(Wheel);
    float cos_yaw = cosf(Yaw);
    float sin_yaw = sinf(Yaw);

    GlobalVelocity.v_x = BodyVelocity.v_x * cos_yaw - BodyVelocity.v_y * sin_yaw;
    GlobalVelocity.v_y = BodyVelocity.v_x * sin_yaw + BodyVelocity.v_y * cos_yaw;
    GlobalVelocity.w = BodyVelocity.w;

    return GlobalVelocity;
}
