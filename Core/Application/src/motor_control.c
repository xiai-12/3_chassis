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




void StartMotorCtlTask(void *argument)
{
    /* USER CODE BEGIN StartMotorCtlTask */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5);

    M3508_Init();

        /* Infinite loop */
    for(;;)
    {
        if (Serial_cmd_structor.debug_mode == DEBUG_FALSE)
        {

            switch (Serial_cmd_structor.motor_mode)
            {
                case MODE_NONE:
                    amp[0] =  M3508_speed_ctl(&m3508_data[Motor1_3508],Serial_cmd_structor.speed_tar);
                    M3508_SendData(&m3508_tx_header,amp);
                    break;
                case MODE_1:

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
    M3508_Reset(&m3508_data[Motor1_3508]);
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
