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
    UART_HandleTypeDef* uart_handle;
    uint8_t line_max;
    volatile bool line_rx_flag;
    uint8_t* line;
    volatile uint8_t line_rx_len;
}uart_rx_struct;

extern uart_rx_struct serial_uart,vision_uart,locator_uart;

void SERIAL_printf (char *fmt, ...);
void Vision_printf (char *fmt, ...);

HAL_StatusTypeDef Uart_rx_init(uart_rx_struct* uart_rx,UART_HandleTypeDef* uart_handle, const uint8_t line_max);
void Uart_callback_func(uart_rx_struct* uart_rx, uint16_t size);
void Uart_rx_free(const uart_rx_struct* uart_rx);
void Uart_error_callback(const uart_rx_struct* uart_rx);

#endif //INC_3_CAN_LOOPBACK_SERIAL_H
