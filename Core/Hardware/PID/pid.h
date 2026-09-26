#ifndef ROBOCON_TEST_PID_H
#define ROBOCON_TEST_PID_H

#include "main.h"

// PID改进措施结构体，可手动配置改进措施
typedef struct PID_Improve_Struct
{
    // 积分改进
    uint8_t integral_limit; // 积分限幅
    float integral_limit_val;// 积分限幅幅值
    uint8_t integral_separation;// 积分分离
    float integral_separation_val;// 积分分离阈值
    uint8_t variable_integal;// 变速积分
    float variable_integal_k;// 变速积分系数

    // 微分改进
    uint8_t deriv_filter; // 不完全微分
    float deriv_filter_alpha;// 滤波系数
    uint8_t deriv_on_meas;  // 微分先行

    // 输出改进
    uint8_t out_offset; // 输出偏移
    uint8_t out_dead_zone;// 输出死区
    float out_dead_zone_val; // 输出死区的值
}PID_Improve;

// pid结构体
typedef struct PID_Struct{
    float Tar,Act,Out;//目标值、实际值、输出值
    float kp,ki,kd;
    float Error_now,Error_last,ErrorInt,Act_last,Deriv,Out_last;// 本次误差 上次误差 误差累计 上次实际值 微分值 上次输出值
    float OutMax,OutMin;// 输出最大最小值
    PID_Improve pid_im; // PID改进结构体
}PID_Structor;

void PID_Init(PID_Structor* pid_struct,float kp,float ki,float kd,float outmax,float outmin,PID_Improve* pid_im);
void PID_Controllor(PID_Structor* pid);
void PID_ModifyParams(PID_Structor* pid,float kp,float ki,float kd);
void PID_Reset(PID_Structor* pid);

#endif //ROBOCON_TEST_PID_H
