//
// Created by zhaol on 2026/10/4.
//

#ifndef INC_3_CHASSIS_CONNECT_H
#define INC_3_CHASSIS_CONNECT_H

#include "main.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdbool.h"

#define VISUAL_HEADER1        0xAA
#define VISUAL_HEADER2        0x55
#define VISUAL_HEADER_LENGTH    1u
#define VISUAL_TAIL1          0x0F
#define VISUAL_TAIL2          0x0A
#define VISUAL_TAIL_LENGTH      1u
#define VISUAL_DATA_LENGTH      6

typedef struct
{
    uint8_t header1;
    uint8_t header2;
    uint8_t id;
    int16_t x;
    int16_t y;
    int16_t w;
    uint8_t crc;
    uint8_t tail;
    //
    int16_t angle;
    int16_t distance;
}Vision_Lia_raw;


extern Vision_Lia_raw vision_lia_raw;

void Visual_Receive(const uint8_t *data);

#endif //INC_3_CHASSIS_CONNECT_H
