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

// 滑动窗口滤波
/**
 *
 * @param maf
 */
void Filter_Init_MAF(Filter_MA_t *maf)
{
    for (int i = 0; i < NUM_SAMPLES; i++) {
        maf->samples[i] = 0;
    }
    maf->sum = 0;
    maf->index = 0;
    maf->count = 0;
}

/**
 *
 * @param maf
 * @param new_data
 * @return
 */
float MAF_Filter(Filter_MA_t *maf, float new_data)
{
    // 从总和中减去即将被替换的旧数据
    maf->sum -= maf->samples[maf->index];

    // 替换为新数据
    maf->samples[maf->index] = new_data;

    // 加上新数据到总和
    maf->sum += new_data;

    // 更新样本计数，直到达到窗口大小
    if (maf->count < NUM_SAMPLES) {
        maf->count++;
    }

    // 更新索引，并在达到窗口大小时回绕
    maf->index = (maf->index + 1) % NUM_SAMPLES;

    // 计算平均值
    return maf->sum / (float)maf->count;
}

