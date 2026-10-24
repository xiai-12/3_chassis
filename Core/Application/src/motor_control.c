#include "motor_control.h"
#include "motion_plane.h"

// 串口解析接收结构体
Serial_CMD Serial_cmd_structor={
    .motor_mode = MODE_NONE,
    .debug_mode = DEBUG_TRUE,
    .kt = 0.5f
};
Vision_CMD Vision_cmd_structor = {0};
Encoder_imu_CMD Encoder_imu_cmd_structor={0};

int16_t amp_zero[4] = {0};
int16_t amp[4] = {0};
int16_t tt_f[4]; // 前馈反馈值


static int16_t limit_constrain_(int32_t amp)
{
    return (int16_t)(amp>15000?15000:(amp<-15000?-15000:amp));
}

static float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void StartMotorCtlTask(void *argument)
{
    /* USER CODE BEGIN StartMotorCtlTask */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5);

    M3508_Init();
    Chassis_Init();

    Chassis_WheelType_t m3508_pos_act;

    static float vx,vy,w;
    static float vx3,vy3,w3;
    static uint8_t stop_flag = 0;
    static uint16_t btn_last = 0;
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
                    // Chassis_WheelType_t m3508_vel_act;
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
                    float vy_1 = Vision_cmd_structor.vy;
                    vy = 0.80f * vy_1 + 0.20f * vy;
                    //float w_1 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.w : Serial_cmd_structor.w;
                    float w_1 = Vision_cmd_structor.w;
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
                    static uint8_t lidia_cal_count = 0;
                    static Chassis_WheelType_t wheel_last,wheel_now,wheel_delta;
                    static float  px = 0.0f, py = 0.0f, pyaw = 0.0f,tgt_x, tgt_y,tgt_w,tgt_yaw;

                    //turns -> rad   *2*M_PI/Reduce_ration = 0.3272f
                    wheel_now.LB = m3508_data[Motor1_3508].turns * 0.3272f;
                    wheel_now.LF = m3508_data[Motor2_3508].turns * 0.3272f;
                    wheel_now.RF = m3508_data[Motor3_3508].turns * 0.3272f;
                    wheel_now.RB = m3508_data[Motor4_3508].turns * 0.3272f;

                    wheel_delta.LB = wheel_now.LB - wheel_last.LB;
                    wheel_delta.LF = wheel_now.LF - wheel_last.LF;
                    wheel_delta.RF = wheel_now.RF - wheel_last.RF;
                    wheel_delta.RB = wheel_now.RB - wheel_last.RB;

                    wheel_last = wheel_now;
                    Chassis_Velocity_t chassis_pos_delta = Chassis_FK_Wheel2Body(&wheel_delta);// 正解算得到的值单位是 m

                    pyaw = wrap_pi(pyaw + chassis_pos_delta.w);
                    float c = cosf(pyaw), s = sinf(pyaw);
                    px += chassis_pos_delta.v_x * c - chassis_pos_delta.v_y * s;
                    py += chassis_pos_delta.v_x * s + chassis_pos_delta.v_y * c;

                    if (++lidia_cal_count >= 6)
                    {
                        lidia_cal_count = 0;
                        px  += 0.3f * (-Vision_cmd_structor.pos_y - px);
                        py  += 0.3f * (Vision_cmd_structor.pos_x - py);
                        pyaw  = wrap_pi(pyaw + 0.3f * wrap_pi(Vision_cmd_structor.yaw - pyaw));
                        //SERIAL_printf("tgt=(%.3f,%.3f) pose=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\r\n",tgt_x, tgt_y,px, py, pyaw,Chassis.targetVel.v_x,Chassis.targetVel.v_y,Chassis.targetVel.w);
                        //SERIAL_printf("%d,%d,%d,%d\r\n",amp[0],amp[1],amp[2],amp[3]);
                    }//

                    Chassis.GlobalPos.x= px;
                    Chassis.GlobalPos.y = py;
                    Chassis.GlobalPos.yaw = pyaw;

                    tgt_x = Serial_cmd_structor.x;
                    tgt_y = Serial_cmd_structor.y;
                    tgt_w = Serial_cmd_structor.w;
                    tgt_yaw = Serial_cmd_structor.yaw;

                    Chassis.targetPos.yaw = tgt_yaw;

                    Chassis_Control(Chassis_Global, Chassis_Position, tgt_x, tgt_y, tgt_w, pyaw);

                    float t  = Chassis.targetWheel.LB * RAD_S_TO_RPM;
                    tt_f[0] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                    t  = Chassis.targetWheel.LF * RAD_S_TO_RPM;
                    tt_f[1] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                    t  = Chassis.targetWheel.RF * RAD_S_TO_RPM;
                    tt_f[2] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));
                    t  = Chassis.targetWheel.RB * RAD_S_TO_RPM;
                    tt_f[3] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));

                    amp[0] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM )+tt_f[0]);
                    amp[1] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM )+tt_f[1]);
                    amp[2] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM )+tt_f[2]);
                    amp[3] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM )+tt_f[3]);

                    // if (sqrtf(powf((tgt_x - Vision_cmd_structor.pos_x),2) + powf((tgt_y - Vision_cmd_structor.pos_y),2)) < 0.04f)
                    // {
                    //     amp[0] = amp[1] = amp[2] = amp[3] =0;
                    // }

                    M3508_SendData(&m3508_tx_header,amp);

                    break;

                        // 单雷达数据
                    // Chassis.GlobalPos.x= -Vision_cmd_structor.pos_y;
                    // Chassis.GlobalPos.y = Vision_cmd_structor.pos_x;
                    // Chassis.GlobalPos.yaw = Vision_cmd_structor.yaw;
                    //
                    // float pos_x   = Serial_cmd_structor.x;
                    // float pos_y   = Serial_cmd_structor.y;
                    // float pos_w   = Serial_cmd_structor.w;
                    //
                    // Chassis_Control(Chassis_Global, Chassis_Position, pos_x, pos_y, pos_w, 0.0f);
                    //
                    // amp[0] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM ));
                    // amp[1] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM ));
                    // amp[2] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM ));
                    // amp[3] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM ));
                    //
                    // if (sqrtf(powf((pos_x - Vision_cmd_structor.pos_x),2) + powf((pos_x - Vision_cmd_structor.pos_y),2)) < 0.04f)
                    // {
                    //     amp[0] = amp[1] = amp[2] = amp[3] =0;
                    // }
                    //
                    // M3508_SendData(&m3508_tx_header,amp);

                        // 编码器正解算
                    // turns -> rad   *2*M_PI/Reduce_ration = 0.3272f
                    // m3508_pos_act.LB = m3508_data[Motor1_3508].turns * 0.3272f;
                    // m3508_pos_act.LF = m3508_data[Motor2_3508].turns * 0.3272f;
                    // m3508_pos_act.RF = m3508_data[Motor3_3508].turns * 0.3272f;
                    // m3508_pos_act.RB = m3508_data[Motor4_3508].turns * 0.3272f;
                    // Chassis_Velocity_t chassis_position = Chassis_FK_Wheel2Body(&m3508_pos_act);// 正解算得到的值单位是 m
                    //
                    // Chassis.GlobalPos.x= chassis_position.v_x;
                    // Chassis.GlobalPos.y = chassis_position.v_y;
                    // Chassis.GlobalPos.yaw = chassis_position.w;
                    //
                    // float pos_x   = Serial_cmd_structor.x;
                    // float pos_y   = Serial_cmd_structor.y;
                    // float pos_w = Serial_cmd_structor.w;
                    //
                    // Chassis_Control(Chassis_Global, Chassis_Position, pos_x, pos_y, pos_w, 0.0f);
                    //
                    // float t  = Chassis.targetWheel.LB * RAD_S_TO_RPM;
                    // tt_f[0] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                    // t  = Chassis.targetWheel.LF * RAD_S_TO_RPM;
                    // tt_f[1] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                    // t  = Chassis.targetWheel.RF * RAD_S_TO_RPM;
                    // tt_f[2] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));
                    // t  = Chassis.targetWheel.RB * RAD_S_TO_RPM;
                    // tt_f[3] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));
                    //
                    // amp[0] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM )+tt_f[0]);
                    // amp[1] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM )+tt_f[1]);
                    // amp[2] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM )+tt_f[2]);
                    // amp[3] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM )+tt_f[3]);
                    // M3508_SendData(&m3508_tx_header,amp);
                    //
                    // break;
                }

                case MODE_3:
                    float vx_3 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.vx : Serial_cmd_structor.x;
                    vx3 = 0.80f * vx_3 + 0.20f * vx3;
                    float vy_3 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.vy : Serial_cmd_structor.y;
                    vy3 = 0.80f * vy_3 + 0.20f * vy3;
                    float w_3 = (nrf_chassis_cmd.link != 0) ? nrf_chassis_cmd.w : Serial_cmd_structor.w;
                    w3 = 0.80f * w_3 + 0.20f * w3;
                    Chassis_Control(Chassis_Body,Chassis_Speed,vx3,vy3,w3,0);
                    //阻力前馈
                    float t  = Chassis.targetWheel.LB * RAD_S_TO_RPM;
                    tt_f[0] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                    t  = Chassis.targetWheel.LF * RAD_S_TO_RPM;
                    tt_f[1] = (int16_t)(Serial_cmd_structor.kt * 600.0f * t / (fabsf(t) + 100.0f));
                    t  = Chassis.targetWheel.RF * RAD_S_TO_RPM;
                    tt_f[2] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));
                    t  = Chassis.targetWheel.RB * RAD_S_TO_RPM;
                    tt_f[3] = (int16_t)(Serial_cmd_structor.kt * 1300.0f * t / (fabsf(t) + 100.0f));

                    amp[0] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor1_3508],Chassis.targetWheel.LB * RAD_S_TO_RPM )+tt_f[0]);
                    amp[1] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor2_3508],Chassis.targetWheel.LF * RAD_S_TO_RPM )+tt_f[1]);
                    amp[2] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor3_3508],Chassis.targetWheel.RF * RAD_S_TO_RPM )+tt_f[2]);
                    amp[3] =  limit_constrain_(M3508_speed_ctl(&m3508_data[Motor4_3508],Chassis.targetWheel.RB * RAD_S_TO_RPM )+tt_f[3]);
                    M3508_SendData(&m3508_tx_header,amp);

                    break;
                default:
                    break;
            }
        }
        else
        {
            vx = vy = w = 0;
            Chassis.targetVel.v_x = Chassis.targetVel.v_y = Chassis.targetVel.w = 0.0f;
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
