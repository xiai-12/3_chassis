//
// Created by zhaol on 2026/10/13.
//
#include "ws2812.h"
#define ws_tim htim5

/* TIM5 是 32 位定时器，DMA 配的是 32 位(WORD)，所以缓冲区必须是 uint32_t */
static uint32_t ws2812_buf[WS2812_BUF_LEN];

void WS2812_Init(void)
{
    /* 整个缓冲区清零：灯珠数据全为 0，末尾复位段也必须为 0（低电平） */
    memset(ws2812_buf, 0, sizeof(ws2812_buf));
    WS2812_Show();
}

/* 把一颗灯的 24bit 编码进 CCR 数组 */
static void ws2812_encode_led(uint8_t r, uint8_t g, uint8_t b, uint32_t *ccr)
{
    /* WS2812 每颗灯是 GRB 顺序，而且高位(MSB)先发，所以 G 放最高 8 位 */
    uint32_t d = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    for (int i = 0; i < 24; i++) {
        ccr[i] = (d & 0x800000U) ? WS2812_HIGH_1 : WS2812_HIGH_0;
        d <<= 1;
    }
}

/* 设置第 index 颗灯的颜色 (0~255) */
void WS2812_SetLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_NUM_LEDS) return;
    ws2812_encode_led(r, g, b, &ws2812_buf[index * 24]);
}

/* 把整条灯带的数据 DMA 发出去 + 复位锁存 */
HAL_StatusTypeDef WS2812_Show(void)
{
    DMA_HandleTypeDef *hdma = ws_tim.hdma[TIM_DMA_ID_CC2];

    /* 上一次还没发完就不要再启动：HAL 会返回 HAL_BUSY，本次数据就丢了 */
    if (hdma == NULL || hdma->State != HAL_DMA_STATE_READY) {
        return HAL_BUSY;
    }

    return HAL_TIM_PWM_Start_DMA(&ws_tim, TIM_CHANNEL_2,
                                 ws2812_buf, WS2812_BUF_LEN);
}
