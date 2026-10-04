//
// Created by zhaol on 2026/10/4.
//
#include "connect.h"

void Visual_Receive(uint8_t *data) {
    uint32_t temp;
    float floatValue;

    temp = data[5] << 24 | data[4] << 16 | data[3] << 8 | data[2];
    memcpy(&floatValue, &temp, sizeof(float));

    temp = data[9] << 24 | data[8] << 16 | data[7] << 8 | data[6];
    memcpy(&floatValue, &temp, sizeof(float));

    temp = data[13] << 24 | data[12] << 16 | data[11] << 8 | data[10];
    memcpy(&floatValue, &temp, sizeof(float));

    temp = data[17] << 24 | data[16] << 16 | data[15] << 8 | data[14];
    memcpy(&floatValue, &temp, sizeof(float));

    temp = data[21] << 24 | data[20] << 16 | data[19] << 8 | data[18];
    memcpy(&floatValue, &temp, sizeof(float));

    temp = data[25] << 24 | data[24] << 16 | data[23] << 8 | data[22];
    memcpy(&floatValue, &temp, sizeof(float));


    //    uart_printf("%f, %f\r\n", visualData.data1, visualData.data2);
}