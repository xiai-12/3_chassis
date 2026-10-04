//
// Created by zhaol on 2026/9/10.
//
#include "Serial.h"

#define Cmd_line_max 64  // 串口接收最大数量


// 串口打印函数
void SERIAL_printf (char *fmt, ...){
    char buff[256];
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, sizeof(buff),fmt, arg_ptr);
    va_end(arg_ptr);
    HAL_UART_Transmit(&huart1,(uint8_t  *)buff,strlen(buff),100);
}

void Vision_printf (char *fmt, ...){
    char buff[256];
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, sizeof(buff),fmt, arg_ptr);
    va_end(arg_ptr);
    HAL_UART_Transmit(&huart2,(uint8_t  *)buff,strlen(buff),100);
}