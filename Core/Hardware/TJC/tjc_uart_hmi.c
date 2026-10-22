//
// Created by Icol_Lee on 2026/1/24.
//
/* Includes ------------------------------------------------------------------*/
#include "tjc_uart_hmi.h"

/* Private define ------------------------------------------------------------*/
#define TJC_TX_BUF_SIZE 125

/* Private variables ---------------------------------------------------------*/
uint8_t tjc_send_buf[TJC_TX_BUF_SIZE];

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 用于陶晶驰串口屏的指令发送
 * @param format
 * @param ...
 */
void tjc_printf(const char *format, ...) {
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *) tjc_send_buf, TJC_TX_BUF_SIZE, (const char *) format, args);
    va_end(args);

    HAL_UART_Transmit(&TJC_UART, (uint8_t *) tjc_send_buf, length,50);
    uint8_t tail[]={0xff,0xff,0xff};
    HAL_UART_Transmit(&TJC_UART,tail, 3,50);
}

/**
 * 颜色清屏
 * @param color
 */
void tjc_cls(uint16_t color) {
    tjc_printf("cls %d", color);
}
/**
 * 矩形颜色填充
 */
void tjc_fill(int posX, int posY, int posW,  int posH, int color) {
    tjc_printf("fill %d,%d,%d,%d,%d", posX, posY, posW, posH, color);
}
/**
 * 插入字符串
 */
void tjc_xstr(int posX, int posY, int posW, int posH, int fontcolor, int backcolor, const char *string) {
    tjc_printf("xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\"%s\"", posX, posY, posW, posH, 0, fontcolor, backcolor, 1, 1, 1, (const char *) string);
}
/**
 * 画线
 */
void tjc_line(int posX1, int posY1, int posX2, int posY2, int w, int color) {
    tjc_printf("line %d,%d,%d,%d,%d", posX1, posY1, posX2, posY2, color);
    for (int i = 0; i <= w; i++) {
        tjc_printf("line %d,%d,%d,%d,%d", posX1 + i, posY1, posX2 + i, posY2, color);
        tjc_printf("line %d,%d,%d,%d,%d", posX1, posY1 + i, posX2, posY2 + i, color);
    }
}

/**
 * 画圆
 */
void tjc_cirs(int posX, int posY, int r, int color) {
    tjc_printf("cirs %d,%d,%d,%d", posX, posY, r, color);
}

////////////////////////////////////////////////////////////////////////////////////

/* 末尾是否为 FF FF FF */
static inline bool TJC_IsEnd(const uint8_t *p)
{
    return (p[0] == 0xFF && p[1] == 0xFF && p[2] == 0xFF);
}

/* 4 字节小端 -> int32 */
static inline int32_t TJC_GetI32LE(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0]| ((uint32_t)p[1] << 8)| ((uint32_t)p[2] << 16)| ((uint32_t)p[3] << 24));
}


void OnButtonEvent(uint8_t page, uint8_t comp, uint8_t event);  /* 65 */
void OnTouchXY    (uint16_t x, uint16_t y, uint8_t event);      /* 67/68 */
void OnCurPage    (uint8_t page);                              /* 66 */
void OnStringVar  (uint8_t *str, uint16_t len);                /* 70 */
void OnNumberVar  (int32_t val);                               /* 71 */
void OnTjcStatus  (uint8_t type);                              /* 24/86/87/88/89/FD/FE */
void OnTjcError   (uint8_t type);                              /* 00~23 错误码 */



void TJC_ParseFrame(const uint8_t *buf, uint16_t len)
{
    if (len < 4) return;                        /* 最短帧为 4 字节 */
    if (!TJC_IsEnd(&buf[len - 3])) return;      /* 帧尾必须 FF FF FF */

    uint8_t type = buf[0];

    switch (type)
    {
    /* ---------- 交互事件：控件点击 ---------- */
    case TJC_TOUCH_COMP:                        /* 65 page comp event FF FF FF */
        if (len == 7)
            OnButtonEvent(buf[1], buf[2], buf[3]);   /* 1=按下 0=弹起 */
        break;

    /* ---------- 交互事件：触摸坐标 / 睡眠触摸 ---------- */
    case TJC_TOUCH_XY:
    case TJC_TOUCH_SLEEP:                       /* 67/68 XH XL YH YL event FF FF FF */
        if (len == 9)
        {
            uint16_t x = ((uint16_t)buf[1] << 8) | buf[2];
            uint16_t y = ((uint16_t)buf[3] << 8) | buf[4];
            OnTouchXY(x, y, buf[5]);
        }
        break;

    /* ---------- 页面查询 ---------- */
    case TJC_CUR_PAGE:                          /* 66 page FF FF FF */
        if (len == 5)
            OnCurPage(buf[1]);
        break;

    /* ---------- 变量查询返回 ---------- */
    case TJC_STR_VAR:                           /* 70 <ASCII...> FF FF FF */
        OnStringVar((uint8_t *)&buf[1], (uint16_t)(len - 4));
        break;

    case TJC_NUM_VAR:                           /* 71 <int32小端> FF FF FF */
        if (len == 8)
            OnNumberVar(TJC_GetI32LE(&buf[1]));
        break;

    /* ---------- 系统状态（无数据） ---------- */
    case TJC_BUF_OVERFLOW:                      /* 24 */
    case TJC_SLEEP_IN:                          /* 86 */
    case TJC_SLEEP_OUT:                         /* 87 */
    case TJC_BOOT_OK:                           /* 88 */
    case TJC_SD_UPGRADE:                        /* 89 */
    case TJC_PASS_DONE:                         /* FD */
    case TJC_PASS_READY:                        /* FE */
        OnTjcStatus(type);
        break;

    /* ---------- 指令成功（bkcmd=3 时才有） ---------- */
    case TJC_CMD_SUCCESS:                       /* 01 */
        OnTjcStatus(type);
        break;

    /* ---------- 指令错误（受 bkcmd 影响） ---------- */
    case TJC_INVALID_CMD:                       /* 00 */
    case TJC_COMP_INVALID:                      /* 02 */
    case TJC_PAGE_INVALID:                      /* 03 */
    case TJC_PIC_INVALID:                       /* 04 */
    case TJC_FONT_INVALID:                      /* 05 */
    case TJC_FILE_FAIL:                         /* 06 */
    case TJC_CRC_FAIL:                          /* 09 */
    case TJC_BAUD_INVALID:                      /* 11 */
    case TJC_CURVE_INVALID:                     /* 12 */
    case TJC_VARNAME_INVALID:                   /* 1A */
    case TJC_VAROP_INVALID:                     /* 1B */
    case TJC_ASSIGN_FAIL:                       /* 1C */
    case TJC_FLASH_FAIL:                        /* 1D */
    case TJC_PARAM_INVALID:                     /* 1E */
    case TJC_IO_FAIL:                           /* 1F */
    case TJC_ESCAPE_ERROR:                      /* 20 */
    case TJC_VARNAME_LONG:                      /* 23 */
        OnTjcError(type);
        break;

    default:                                    /* 未知类型 */
        OnTjcError(0xFF);
        break;
    }
}