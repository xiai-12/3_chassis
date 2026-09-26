//
// Created by zhaol on 2026/9/19.
//

#ifndef INC_1_CAN_3508_MOTION_PLANE_H
#define INC_1_CAN_3508_MOTION_PLANE_H

#include "main.h"
#include <math.h>


// 轨迹结构体
typedef struct
{
    float pos_target;    // 目标位置
    float pos_act;       // 实际位置
    float vref;          // 规划速度
    float vmax;          // 最大速度
    float acc;           // 最大加速度
} Trace;

extern Trace TracePlane;

void Tace_Update(Trace* t,float dt);

#endif //INC_1_CAN_3508_MOTION_PLANE_H
