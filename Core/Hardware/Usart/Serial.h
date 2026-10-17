//
// Created by zhaol on 2026/9/10.
//
#ifndef INC_3_CAN_LOOPBACK_SERIAL_H
#define INC_3_CAN_LOOPBACK_SERIAL_H

#include  "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stdbool.h"
#include <stdlib.h>

typedef struct  Uart_rx_struct{
    UART_HandleTypeDef* uart_handle;  // 串口句柄
    uint8_t line_max;                 // dma接收数组大小
    volatile bool line_rx_flag;       // dma接收标志
    uint8_t* line;                    // 接收数组
    uint8_t* cmd_line;                // 解析数组
    volatile uint16_t line_rx_len;     // 接收数据个数
}uart_rx_struct;

extern uart_rx_struct serial_uart,vision_uart,locator_uart;

void SERIAL_printf (char *fmt, ...);
void Vision_printf (char *fmt, ...);

HAL_StatusTypeDef Uart_rx_init(uart_rx_struct* uart_rx,UART_HandleTypeDef* uart_handle, uint8_t line_max);
void Uart_callback_func(uart_rx_struct* uart_rx, uint16_t size);
void Uart_rx_free(const uart_rx_struct* uart_rx);
void Uart_error_callback(const uart_rx_struct* uart_rx);

#endif //INC_3_CAN_LOOPBACK_SERIAL_H
