//
// Created by Icol_Lee on 2026/1/24.
//

#ifndef DEVICE_TJC_UART_HMI_H
#define DEVICE_TJC_UART_HMI_H

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "usart.h"
#include "math.h"

#define TJC_UART    huart4

// HMI颜色
#define RED     63488
#define BLUE    31
#define GRAY    33840
#define BLACK   0
#define WHITE   65535
#define GREEN   2016
#define BROWN   48192
#define YELLOW  65504

typedef enum {
    /* 指令执行结果通知（受 bkcmd 影响） */
    TJC_INVALID_CMD     = 0x00,  /* 无效指令 */
    TJC_CMD_SUCCESS     = 0x01,  /* 指令成功执行 */
    TJC_COMP_INVALID    = 0x02,  /* 控件ID无效 */
    TJC_PAGE_INVALID    = 0x03,  /* 页面ID无效 */
    TJC_PIC_INVALID     = 0x04,  /* 图片ID无效 */
    TJC_FONT_INVALID    = 0x05,  /* 字库ID无效 */
    TJC_FILE_FAIL       = 0x06,  /* 文件操作失败 */
    TJC_CRC_FAIL        = 0x09,  /* CRC校验失败 */
    TJC_BAUD_INVALID    = 0x11,  /* 波特率设置无效 */
    TJC_CURVE_INVALID   = 0x12,  /* 曲线控件ID/通道无效 */
    TJC_VARNAME_INVALID = 0x1A,  /* 变量名称无效 */
    TJC_VAROP_INVALID   = 0x1B,  /* 变量运算无效 */
    TJC_ASSIGN_FAIL     = 0x1C,  /* 赋值操作失败 */
    TJC_FLASH_FAIL      = 0x1D,  /* 掉电存储操作失败 */
    TJC_PARAM_INVALID   = 0x1E,  /* 参数数量无效 */
    TJC_IO_FAIL         = 0x1F,  /* IO操作失败 */
    TJC_ESCAPE_ERROR    = 0x20,  /* 转义字符使用错误 */
    TJC_VARNAME_LONG    = 0x23,  /* 变量名称太长 */

    //系统状态
    TJC_BUF_OVERFLOW    = 0x24,  /* 串口缓冲区溢出 */
    TJC_SLEEP_IN        = 0x86,  /* 自动进入睡眠 */
    TJC_SLEEP_OUT       = 0x87,  /* 自动唤醒 */
    TJC_BOOT_OK         = 0x88,  /* 系统启动成功 */
    TJC_SD_UPGRADE      = 0x89,  /* 开始SD卡升级 */
    TJC_PASS_DONE       = 0xFD,  /* 透传数据完成 */
    TJC_PASS_READY      = 0xFE,  /* 数据透传就绪 */

    //交互事件
    TJC_TOUCH_COMP      = 0x65,  /* 控件点击事件 */
    TJC_CUR_PAGE        = 0x66,  /* 当前页面ID */
    TJC_TOUCH_XY        = 0x67,  /* 触摸坐标 */
    TJC_TOUCH_SLEEP     = 0x68,  /* 睡眠模式触摸事件 */
    TJC_STR_VAR         = 0x70,  /* 字符串变量返回 */
    TJC_NUM_VAR         = 0x71,  /* 数值变量返回 */
} tjc_uart_type;

void tjc_printf(const char *format, ...);
void tjc_cls(uint16_t color);
void tjc_fill(int posX, int posY, int posW,  int posH, int color);
void tjc_xstr(int posX, int posY, int posW, int posH, int fontcolor, int backcolor, const char *string);
void tjc_line(int posX1, int posY1, int posX2, int posY2, int w, int color);
void tjc_cirs(int posX, int posY, int r, int color);
void Linkage_hmi(float X, float Y);

#endif //DEVICE_TJC_UART_HMI_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
