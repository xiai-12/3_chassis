//
// Created by zhaol on 2026/9/10.
//
#include "Serial.h"

uart_rx_struct serial_uart,vision_uart,locator_uart,tjc_uart;

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

HAL_StatusTypeDef Uart_rx_init(uart_rx_struct* uart_rx,UART_HandleTypeDef* uart_handle, const uint8_t line_max)
{
    uart_rx->uart_handle = uart_handle;
    uart_rx->line_max = line_max;
    uart_rx->line_rx_flag = 0;
    uart_rx->line_rx_len = 0;
    uart_rx->line = (uint8_t *)malloc(sizeof(uint8_t) * uart_rx->line_max);
    uart_rx->cmd_line = (uint8_t *)malloc(sizeof(uint8_t) * uart_rx->line_max + 1u);
    memset(uart_rx->line,0,uart_rx->line_max);
    memset(uart_rx->cmd_line,0,uart_rx->line_max+1);
    if ((uart_rx->line == NULL) || (uart_rx->cmd_line == NULL)) return HAL_ERROR;
    // 使用Ex函数，接收不定长数据
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart_rx->uart_handle,uart_rx->line,uart_rx->line_max) == HAL_ERROR)
    {
        SERIAL_printf("uart init failed,retry\r\n");
        return HAL_ERROR;
    }
    // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
    __HAL_DMA_DISABLE_IT(uart_rx->uart_handle->hdmarx, DMA_IT_HT);
    return HAL_OK;
}

void Uart_callback_func(uart_rx_struct* uart_rx, const uint16_t size)
{
    uart_rx->line_rx_len = size;
    uart_rx->line_rx_flag = 1;

    memcpy(uart_rx->cmd_line, uart_rx->line, size);
    // 尾部清零：防止解析函数读到上一帧残留的字节
    memset(uart_rx->cmd_line + size, 0, (uart_rx->line_max + 1U - size));

    HAL_UARTEx_ReceiveToIdle_DMA(uart_rx->uart_handle,uart_rx->line,uart_rx->line_max);
    __HAL_DMA_DISABLE_IT(uart_rx->uart_handle->hdmarx, DMA_IT_HT);
}

void Uart_error_callback(const uart_rx_struct* uart_rx)
{
    __HAL_UART_CLEAR_FLAG(uart_rx->uart_handle, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    uart_rx->uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UARTEx_ReceiveToIdle_DMA(uart_rx->uart_handle, uart_rx->line,uart_rx->line_max);
    __HAL_DMA_DISABLE_IT(uart_rx->uart_handle->hdmarx, DMA_IT_HT);
}

void Uart_rx_free(const uart_rx_struct* uart_rx)
{
    if (uart_rx->cmd_line != NULL)
    {
        free(uart_rx->cmd_line);
    }
    if (uart_rx->line != NULL)
    {
        free(uart_rx->line);
    }
}


