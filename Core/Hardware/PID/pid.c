#include "pid.h"
#include <math.h>

// pid结构体初始化
void PID_Init(PID_Structor* pid_struct,float kp,float ki,float kd,float outmax,float outmin,PID_Improve* pid_im){
    pid_struct->Tar= pid_struct->Act = pid_struct->Out = pid_struct->Out_last =pid_struct->Act_last = pid_struct->Deriv =0;
    pid_struct->Error_now = pid_struct->Error_last = pid_struct->ErrorInt =0;
    pid_struct->kp = kp;
    pid_struct->ki = ki;
    pid_struct->kd = kd;
    pid_struct->OutMax = outmax;
    pid_struct->OutMin = outmin;
    pid_struct->pid_im = *pid_im;
}

// pid核心运算
void PID_Controllor(PID_Structor* pid)
{
    float ki_c = 1.0f;// 积分系数

    // 计算误差
    pid->Error_last = pid->Error_now;
    pid->Error_now = pid->Tar - pid->Act;
    // pid改进措施
    ///////////////////////////////////////////////////////// 积分部分
    //变速积分
    if (pid->pid_im.variable_integal)
    {
        ki_c = 1 / (pid->pid_im.variable_integal_k * fabsf(pid->Error_now)+ 1);
    }

    //积分分离
    if (pid->pid_im.integral_separation && (fabsf(pid->Error_now) > pid->pid_im.integral_separation_val))
    {
        pid->ErrorInt = 0;
    }
    else
    {
        if (pid->ki != 0)
        {
            pid->ErrorInt += ki_c * pid->Error_now;
        }
        else
        {
            pid->ErrorInt = 0;
        }
    }
    // 积分限幅
    if (pid->pid_im.integral_limit)
    {
        if (pid->ErrorInt > pid->pid_im.integral_limit_val)  pid->ErrorInt = pid->pid_im.integral_limit_val;
        if (pid->ErrorInt < -pid->pid_im.integral_limit_val) pid->ErrorInt = -pid->pid_im.integral_limit_val;
    }

    ///////////////////////////////////////////////////////// 微分部分
    // 微分先行
    if (pid->pid_im.deriv_on_meas)
    {
        // 不完全微分
        if (pid->pid_im.deriv_filter)
        {
            pid->Deriv = -(pid->Act - pid->Act_last) * pid->pid_im.deriv_filter_alpha + pid->Deriv * (1 - pid->pid_im.deriv_filter_alpha);
        }
        else
        {
            pid->Deriv = -(pid->Act - pid->Act_last);
        }
    }
    else
    {
        pid->Deriv = pid->Error_now - pid->Error_last;
    }
    pid->Act_last = pid->Act;
    ///////////////////////////////////////////////////////// 输出部分
    // 输出值计算
    pid->Out = (pid->kp)*(pid->Error_now)+(pid->ki)*(pid->ErrorInt)+(pid->kd)*(pid->Deriv);
    // 输出死区
    if (pid->pid_im.out_dead_zone)
    {
        if (fabsf(pid->Error_now) < pid->pid_im.out_dead_zone_val)
        {
            pid->Out = 0.0f;
        }
    }
    // 输出偏移
    if (pid->pid_im.out_offset)
    {
        if (pid->Out > 0) pid->Out += 432.0f;
        else if (pid->Out < 0) pid->Out -= 432.0f;
        else pid->Out = 0.0f;
    }

    //输出限幅
    if(pid->Out > pid->OutMax){pid->Out = pid->OutMax;}
    if(pid->Out < pid->OutMin){pid->Out = pid->OutMin;}
}

// pid调参函数
void PID_ModifyParams(PID_Structor* pid,float kp,float ki,float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// pid清除增益
void PID_Reset(PID_Structor* pid)
{
    pid->Tar= pid->Act = pid->Out = pid->Act_last = pid->Deriv =0;
    pid->Error_now = pid->Error_last = pid->ErrorInt =0;
}