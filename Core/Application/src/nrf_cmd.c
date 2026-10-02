//
// Created by zhaol on 2026/10/2.
//
#include "nrf_cmd.h"

volatile uint8_t  radio_init_ok = 0U;   /* 0 = 模块没应答 */
volatile uint32_t radio_rx_cnt  = 0U;   /* 收到的有效包数 */
volatile uint32_t radio_bad_cnt = 0U;   /* 校验失败的包数 */

Nrf_ChassisCmd_t nrf_chassis_cmd = {0.0f, 0.0f, 0.0f, 0U};


// 归一化，限制在 -1 ~ 1 之间
static float Stick_Norm(uint16_t raw)
{
    float v = ((float)raw - 2048.f) / 2048.0f;
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    if (fabsf(v) < STICK_DEADZONE) v = 0.0f;
    return v;
}

void Nrf_UpdateChassisCmd(void)
{
    ControlCommand c;
    float vx, vy, w, need, k;

    if (ControlSlave_Get(&c) == 0)
    {
        nrf_chassis_cmd.vx   = 0.0f;
        nrf_chassis_cmd.vy   = 0.0f;
        nrf_chassis_cmd.w    = 0.0f;
        nrf_chassis_cmd.link = 0;
        return;
    }

    vx = VX_SIGN * Stick_Norm(c.l_y) * VX_MAX;
    vy = VY_SIGN * Stick_Norm(c.l_x) * VY_MAX;
    w  = W_SIGN  * Stick_Norm(c.r_x) * W_MAX;


    need = 0.70710678f * (fabsf(vx) + fabsf(vy)) + CHASSIS_R * fabsf(w);
    if (need > WHEEL_LIN_BUDGET)
    {
        k = WHEEL_LIN_BUDGET / need;
        vx *= k;
        vy *= k;
        w  *= k;
    }

    nrf_chassis_cmd.vx   = vx;
    nrf_chassis_cmd.vy   = vy;
    nrf_chassis_cmd.w    = w;
    nrf_chassis_cmd.link = 1;
}


void StartNrfCmdTask(void *argument)
{
    uint8_t rx[CONTROL_PKT_LEN];
    uint8_t ack[CONTROL_ACK_LEN];
    ControlCommand cmd;
    //uint8_t new_packet;

    radio_init_ok = Nrf24_InitRx();
    //SERIAL_printf("\r\n[radio] init=%u  (1=OK, 0=SPI/module fail)\r\n",(unsigned)radio_init_ok);

    for (;;)
    {
        //new_packet = 0U;
        if (Nrf24_Poll(rx) != 0U)
        {
            if (ControlSlave_Parse(rx, &cmd) != 0U)
            {
                radio_rx_cnt++;
                ControlSlave_Publish(&cmd);           /* 存最新命令 + 打时间戳 */

                ControlSlave_BuildAck(cmd.seq, CONTROL_ACK_STATUS_LINK, 0U, ack);
                Nrf24_QueueAck(ack);                  /* 随下一包硬件应答发出 */
                //new_packet = 1U;
            }
            else
            {
                radio_bad_cnt++;
            }
        }

        Nrf_UpdateChassisCmd();
        // if (new_packet != 0)
        // {
        //     SERIAL_printf("RX %lu seq=%u l_x=%u l_y=%u r_x=%u | vx=%.2f vy=%.2f w=%.2f link=%u\r\n",
        //                   (unsigned long)radio_rx_cnt, (unsigned)cmd.seq,
        //                   (unsigned)cmd.l_x, (unsigned)cmd.l_y, (unsigned)cmd.r_x,
        //                   nrf_chassis_cmd.vx, nrf_chassis_cmd.vy, nrf_chassis_cmd.w,
        //                   (unsigned)nrf_chassis_cmd.link);
        // }
        osDelay(2);
    }
}