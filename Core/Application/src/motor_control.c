#include "motor_control.h"
#include "motion_plane.h"


// 串口解析接收结构体
Serial_CMD Serial_cmd_structor={
    .motor_mode = MODE_NONE,
    .debug_mode = DEBUG_TRUE,
    .speed_tar = 0,
    .location_tar = 0,
    .acc = 0,
    .v_max = 0
};

int16_t amp_zero[4] = {0};
int16_t amp[4] = {0};

Chassis_WheelType_t m3508_act;
Chassis_Velocity_t chassis_velocity;


void StartMotorCtlTask(void *argument)
{
    /* USER CODE BEGIN StartMotorCtlTask */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5);

    M3508_Init();
    Chassis_Init();
    /* Infinite loop */
    for(;;)
    {
        if (Serial_cmd_structor.debug_mode == DEBUG_FALSE)
        {

            switch (Serial_cmd_structor.motor_mode)
            {
                case MODE_NONE:
                    break;
                case MODE_1:
                    // 电调反馈的速度单位是 rpm,是内部转子的速度 ，要转化成 rad/s，同时要的是轮子的速度，也就是外部的速度
                    m3508_act.LF = (float)m3508_data[Motor1_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    m3508_act.LB = (float)m3508_data[Motor2_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    m3508_act.RF = (float)m3508_data[Motor3_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    m3508_act.RB = (float)m3508_data[Motor4_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    chassis_velocity = Chassis_FK_Wheel2Body(&m3508_act);

                    // vx,vy上位机输入的是 m/s，w 输入的单位是 rad/s,经过底盘结算之后的每个轮子的速度 单位是 rad/s ,要转化成 rpm
                    Chassis_Control(Chassis_Body,Chassis_Speed,Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w,0);
                    amp[0] =  M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM);
                    amp[1] =  M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM);
                    amp[2] =  M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM);
                    amp[3] =  M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM);
                    M3508_SendData(&m3508_tx_header,amp);
                    break;
                case MODE_2:
                    break;
                case MODE_3:
                    break;
                default:
                    break;
            }
        }
        else
        {
            M3508_SendData(&m3508_tx_header,amp_zero);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);

    }
    /* USER CODE END StartMotorCtlTask */
}

// 电机重置，用于模式切换，防止 speed_tar \ location_tar继承
void Motor_Reset(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        M3508_Reset(&m3508_data[Motor1_3508]);
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t d[8];
    if (hfdcan != &hfdcan1) return;
    while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &hdr, d) == HAL_OK)
    {
        if (hdr.IdType == FDCAN_EXTENDED_ID) return;
        switch (hdr.Identifier)
        {
        // dji-m3508
        case 0x205:
            m3508_motor_callback(&hdr,&m3508_data[Motor1_3508],d);
            break;
        case 0x206:
            m3508_motor_callback(&hdr,&m3508_data[Motor2_3508],d);
            break;
        case 0x207:
            m3508_motor_callback(&hdr,&m3508_data[Motor3_3508],d);
            break;
        case 0x208:
            m3508_motor_callback(&hdr,&m3508_data[Motor4_3508],d);
            break;

        // dmiao-j4340-2ec
        case 0x010:
            //dm_motor_callback(&hdr,d);
            break;

        default:
            break;;
        }
    }
}
