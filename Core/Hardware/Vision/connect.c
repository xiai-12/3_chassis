//
// Created by zhaol on 2026/10/4.
//
#include "connect.h"

Vision_Lia_raw vision_lia_raw={0};

/*
 * 共11个字节
 * data                           explain
 *  0,1   -- 0x5A 0xA5             包头
 *  2   -- 0x01                    ID
 *  4,3 -- vx                      vx速度
 *  6,5 -- vy                      vy速度
 *  8,7 -- w                       w速度
 *  9   -- crc                     校验码
 *  10  -- 0xED                    EOF
 */

void Visual_Receive(const uint8_t *data) {
    vision_lia_raw.header1 = data[0];
    vision_lia_raw.header2 = data[1];
    vision_lia_raw.x = (int16_t)(data[4] << 8 | data[3]);
    vision_lia_raw.y = (int16_t)(data[6] << 8 | data[5]);
    vision_lia_raw.w = (int16_t)(data[8] << 8 | data[7]);
    vision_lia_raw.crc = data[9];
    vision_lia_raw.tail = data[10];
}





