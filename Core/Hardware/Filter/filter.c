#include "filter.h"

// RC 低通滤波
/**
 * 初始化
 * @param filter    过滤器句柄
 * @param ts        采样时间（s）
 * @param fc        截止频率（hz）
 */
void Filter_Init_LPF(Filter_LP_t* filter, float ts, float fc) {
    float b = 2.0f * (float)M_PI * fc * ts;

    filter->ts = ts;
    filter->fc = fc;
    filter->lastYn = 0;
    filter->alpha = b / (b + 1);
}
/**
 * 输出过滤后结果
 * @param filter    过滤器句柄
 * @param data      采样数据
 * @return
 */
float LPF_Filter(Filter_LP_t* filter, float data) {
    float tem = filter->lastYn + (filter->alpha * (data - filter->lastYn));
    filter->lastYn = tem;
    return tem;
}

