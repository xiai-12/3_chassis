/**
  ******************************************************************************
  @file     filter.h
  @brief    滤波算法：
                -
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2026-03-14 (Created) | 2026-03-14 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-03-14 [v1.0] Icol Boom: 创建初始版本
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef FILTER_FILTER_H
#define FILTER_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "main.h"


#define NUM_SAMPLES 8

// 低通滤波
typedef struct {
    float ts;       //采样周期(s)
    float fc;       //截至频率(hz)
    float lastYn;   //上一次滤波值
    float alpha;    //滤波系数
} Filter_LP_t;

void Filter_Init_LPF(Filter_LP_t* filter, float ts, float fc);
float LPF_Filter(Filter_LP_t* filter, float data);


#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
