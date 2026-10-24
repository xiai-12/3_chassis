#ifndef FILTER_FILTER_H
#define FILTER_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "main.h"


#define NUM_SAMPLES 3

// 低通滤波
typedef struct {
    float ts;       //采样周期(s)
    float fc;       //截至频率(hz)
    float lastYn;   //上一次滤波值
    float alpha;    //滤波系数
} Filter_LP_t;

typedef struct {
    float samples[NUM_SAMPLES]; // 存储最近的RPM样本
    float sum;                   // 当前窗口内RPM值的总和
    uint8_t index;                  // 当前写入的位置
    uint8_t count;                  // 当前样本数量
} Filter_MA_t;

void Filter_Init_LPF(Filter_LP_t* filter, float ts, float fc);
float LPF_Filter(Filter_LP_t* filter, float data);
void Filter_Init_MAF(Filter_MA_t *maf);
float MAF_Filter(Filter_MA_t *maf, float new_data);


#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
