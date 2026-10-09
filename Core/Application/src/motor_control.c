#include "motor_control.h"
#include "motion_plane.h"

static int16_t limit_constrain_(int32_t amp)
{
    return (int16_t)(amp>15000?15000:(amp<-15000?-15000:amp));
}

// 串口解析接收结构体
Serial_CMD Serial_cmd_structor={
    .motor_mode = MODE_NONE,
    .debug_mode = DEBUG_TRUE,
    .speed_tar = 0,
    .location_tar = 0,
    .angle = 0,
    .kt = 2.0f
};

Vision_CMD Vision_cmd_structor = {0};

int16_t amp_zero[4] = {0};
int16_t amp[4] = {0};
int16_t tt_f[4]; // 前馈反馈值
// 单电机转速实际值
Chassis_WheelType_t m3508_vel_act;
// 单电机位置实际值
Chassis_WheelType_t m3508_pos_act;
// 速度正解算值
Chassis_Velocity_t chassis_velocity;
// 位置正解算值
Chassis_Velocity_t chassis_position;


float chassis_yaw = 0.0f;

void StartMotorCtlTask(void *argument)
{
    /* USER CODE BEGIN StartMotorCtlTask */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5);

    M3508_Init();
    Chassis_Init();

    static float vx,vy,w,yaw;
    static uint8_t stop_flag = 0;
    static uint16_t btn_last = 0;
    static float t=0.0f;
    static float angle = 0.0f;
    /* Infinite loop */
    for(;;)
    {

        // 判断是否按下急停
        if (nrf_chassis_cmd.button == 1u && btn_last == 0u)
        {
            stop_flag = !stop_flag;
            Motor_Reset();
            TracePlane.vref = 0.0f;
            Serial_cmd_structor.location_tar = 0;
        }
        btn_last = nrf_chassis_cmd.button;


        // 按键或者串口开启二选一
        if ((Serial_cmd_structor.debug_mode == DEBUG_FALSE || nrf_chassis_cmd.keys == 4) && !stop_flag)
        {
            switch (Serial_cmd_structor.motor_mode)
            {
                case MODE_NONE:
                    break;
                case MODE_1:
                    // 电调反馈的速度单位是 rpm,是内部转子的速度 ，要转化成 rad/s，同时要的是轮子的速度，也就是外部的速度
                    // m3508_act.RF = (float)m3508_data[Motor1_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    // m3508_act.RB = (float)m3508_data[Motor2_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    // m3508_act.LB = (float)m3508_data[Motor3_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    // m3508_act.LF = (float)m3508_data[Motor4_3508].speed / RAD_S_TO_RPM / Reduce_ration;
                    // chassis_velocity = Chassis_FK_Wheel2Body(&m3508_act);
                    // vx,vy上位机输入的是 m/s，w 输入的单位是 rad/s,经过底盘结算之后的每个轮子的速度 单位是 rad/s ,要转化成 rpm

                    /* 遥控器在线就用遥控器；掉线退回串口，方便台上调试 */
                    //float vx_1 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.vx : Serial_cmd_structor.vx;
                    float vx_1 = Vision_cmd_structor.vx;
                    vx = 0.80f * vx_1 + 0.20f * vx;
                    //float vy_1 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.vy : Serial_cmd_structor.vy;
                    float vy_1 = Vision_cmd_structor.vx;
                    vy = 0.80f * vy_1 + 0.20f * vy;
                    //float w_1 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.w : Serial_cmd_structor.w;
                    float w_1 = Vision_cmd_structor.vx;
                    w = 0.80f * w_1 + 0.20f * w;
                    Chassis_Control(Chassis_Body,Chassis_Speed,vx,vy,w,0);

                    amp[0] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM));
                    amp[1] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM));
                    amp[2] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM));
                    amp[3] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM));
                    M3508_SendData(&m3508_tx_header,amp);
                    break;
                case MODE_2:
                {

                    Chassis.GlobalPos.x   = Vision_cmd_structor.pos_y;
                    Chassis.GlobalPos.y   = Vision_cmd_structor.pos_x;
                    Chassis.GlobalPos.yaw = -1.0f * Vision_cmd_structor.yaw;

                    float tgt_x = Serial_cmd_structor.vy;
                    float tgt_y = Serial_cmd_structor.vx;
                    Chassis.targetPos.yaw = -1.0f * Serial_cmd_structor.w;

                    Chassis_Control(Chassis_Global, Chassis_Position, tgt_x, tgt_y, 0.0f, Vision_cmd_structor.yaw);

                    amp[0] = limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508], Chassis.targetWheel.LB * RAD_S_TO_RPM));
                    amp[1] = limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508], Chassis.targetWheel.LF * RAD_S_TO_RPM));
                    amp[2] = limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508], Chassis.targetWheel.RF * RAD_S_TO_RPM));
                    amp[3] = limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508], Chassis.targetWheel.RB * RAD_S_TO_RPM));
                    M3508_SendData(&m3508_tx_header, amp);
                    break;
                }

                // case MODE_2:
                //     // ° -> rad   * 2*M_PI/360 = 0.01745329252f
                //     // angle = Serial_cmd_structor.angle * 0.01745329252f;
                //     // turns -> rad   *2*M_PI/Reduce_ration = 0.3272f
                //     // m3508_pos_act.RF = m3508_data[Motor1_3508].turns * 0.3272f;
                //     // m3508_pos_act.RB = m3508_data[Motor2_3508].turns * 0.3272f;
                //     // m3508_pos_act.LB = m3508_data[Motor3_3508].turns * 0.3272f;
                //     // m3508_pos_act.LF = m3508_data[Motor4_3508].turns * 0.3272f;
                //     // chassis_position = Chassis_FK_Wheel2Body(&m3508_pos_act);// 正解算得到的值单位是 m
                //
                //
                //     /**
                //      * 视觉只给了绝对位置，目标位置仍由串口设定
                //      */
                //     float pos_x = Serial_cmd_structor.vx;
                //     float pos_y = Serial_cmd_structor.vy;
                //     Chassis.targetPos.yaw = Serial_cmd_structor.w;
                //
                // /*
                //     // float pos_x = Vision_cmd_structor.pos_x;
                //     // float pos_y = Vision_cmd_structor.pos_y;
                //     // float pos_z = Vision_cmd_structor.yaw;
                //     TracePlane.pos_target = sqrtf(powf(pos_x,2)+powf(pos_y,2));
                //     angle = atan2f(pos_y,pos_x);
                //     //TracePlane.pos_act = chassis_position.v_x * cosf(angle) + chassis_position.v_y * sinf(angle);
                //     TracePlane.pos_act = (Vision_cmd_structor.pos_y * cosf(angle) + Vision_cmd_structor.pos_x * sinf(angle));
                //     Tace_Update(&TracePlane,0.005f);
                //     vx = TracePlane.vref * cosf(angle);
                //     vy = TracePlane.vref * sinf(angle);
                //     yaw = Vision_cmd_structor.yaw;
                //     Chassis_Control(Chassis_Body,Chassis_Speed,vx,vy,0,0);
                // */
                //
                //     Chassis_Control(Chassis_Body,Chassis_Position,Vision_cmd_structor.pos_x *5.0f,Vision_cmd_structor.pos_y *5.0f,0,Vision_cmd_structor.yaw);
                //     // 阻力前馈
                //     // t  = Chassis.targetWheel.LB * RAD_S_TO_RPM;
                //     // tt_f[0] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                //     // t  = Chassis.targetWheel.LF * RAD_S_TO_RPM;
                //     // tt_f[1] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));
                //     // t  = Chassis.targetWheel.RF * RAD_S_TO_RPM;
                //     // tt_f[2] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));
                //     // t  = Chassis.targetWheel.RB * RAD_S_TO_RPM;
                //     // tt_f[3] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                //     // 输出值计算
                //     amp[0] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM) + tt_f[0]);
                //     amp[1] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM) + tt_f[1]);
                //     amp[2] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM) + tt_f[2]);
                //     amp[3] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM) + tt_f[3]);
                //     M3508_SendData(&m3508_tx_header,amp);
                //     break;
                case MODE_3:
                    break;
                default:
                    break;
            }
        }
        else
        {
            vx = vy = w = 0;
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
        M3508_Reset(&m3508_data[i]);
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
        case 0x201:
            m3508_motor_callback(&hdr,&m3508_data[Motor1_3508],d);
            break;
        case 0x202:
            m3508_motor_callback(&hdr,&m3508_data[Motor2_3508],d);
            break;
        case 0x203:
            m3508_motor_callback(&hdr,&m3508_data[Motor3_3508],d);
            break;
        case 0x204:
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
