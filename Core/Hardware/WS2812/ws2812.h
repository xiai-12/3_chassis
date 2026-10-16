//
// Created by zhaol on 2026/10/13.
//

#ifndef INC_3_CHASSIS_WS2812_H
#define INC_3_CHASSIS_WS2812_H

#include "main.h"
#include "tim.h"
#include "string.h"

#define WS2812_NUM_LEDS      74      // 灯珠数量
#define WS2812_RESET_BITS    280     // 280*1.25us=350us>280us

/* TIM5: PSC=0, ARR=149, TIM5CLK=120MHz => 150/120MHz = 1.25us/bit, 1 tick = 8.33ns
 * T0H = 48 * 8.33ns = 0.40us
 * T1H = 96 * 8.33ns = 0.80us */
#define WS2812_HIGH_1        96
#define WS2812_HIGH_0        48

/* 灯珠数据段 + 复位(锁存)段，两段必须在同一个 DMA 缓冲区里，长度一致 */
#define WS2812_DATA_LEN      (WS2812_NUM_LEDS * 24)
#define WS2812_BUF_LEN       (WS2812_DATA_LEN + WS2812_RESET_BITS)

void WS2812_Init(void);
void WS2812_SetLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef WS2812_Show(void);

#endif //INC_3_CHASSIS_WS2812_H
