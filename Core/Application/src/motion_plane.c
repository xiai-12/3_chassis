//
// Created by zhaol on 2026/9/19.
//
#include "motion_plane.h"


// 梯形速度曲线规划初始化
Trace TracePlane={
    .vmax = 6000.0f,  //  最大速度，单位 rpm / r/min(转每分钟)
    .acc = 6000.0f    //  加速度，单位 rpm/s  即 r/(min*s)  (转/每分每秒)
};


/*
 * 轨迹规划函数
 *  根据目标位置与实际位置的差值 err计算刹车速度v_brake = sqrt(2*a_cc*err)
 *  未达到v_max，继续按 acc线性加速
 */
void Tace_Update(Trace* t,float dt)
{
    float err = (t->pos_target - t->pos_act)/8191.0f;  // err 为 电机圈数误差
    float err_abs = fabsf(err);
    float dir = (err >= 0.0f)?1.0f:-1.0f; // 判断电机运动方向
    float v_brake = sqrtf(2.0f * t->acc * err_abs * 60.0f);// v_brake 刹车速度 单位 rpm

    // 到目标位置附近，直接归零，防止在目标点附近反复振荡
    if (err_abs < 0.025f)
    {
        t->vref = 0.0f;
        return;
    }

    // 当前速度方向和目标方向相反
    if (t->vref * dir < 0.0f)
    {
        t->vref += dir * t->acc * dt; // 超出一个周期，返回一次
        if (t->vref * dir < 0.0f) t->vref = 0.0f;// 超出多个周期，直接清零
        return;
    }

    /*
     *  加速阶段：fabsf(t->vref) < v_brake 通过if判断将最大速度限制在 v_brake (v_brake < v_max,说明规划距离短； v_brake > v_max,说明规划距离长，且v_brake会逐渐减小到v_max)
     *  减速阶段： 将速度钳位在 v_brake ,减速阶段通过 v_brake的减小
     */
    if (fabsf(t->vref) < v_brake)
    {
        // 加速阶段
        t->vref += dir * t->acc * dt;
        if (fabsf(t->vref) > t->vmax) t->vref = dir * t->vmax;
        if (fabsf(t->vref) > v_brake)  t->vref = dir * v_brake;
    }
    else
    {
        t->vref = dir * v_brake;
    }
}