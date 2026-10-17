#include "main.h"
#include "motor_control.h"
#include "motion_plane.h"
#include "Vision/connect.h"
#include "string.h"
#include "Filter/filter.h"

// 函数声明
static float Parse_float(char *cmd,uint8_t index); // 串口浮点数解析函数，index为浮点数第一位的索引
static void Radar_Filter(float rx, float ry, float ryaw,float* val);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 雷达数据滤波
Filter_MA_t x_mat,y_mat,yaw_mat;
#define Filter_alpha   0.9f
#define Radar_jump_max 0.04f
static float last_rx, last_ry,last_yaw = 0.0f;
static uint8_t radar_first = 1;              // 首帧标志
static void Radar_Filter(float rx, float ry, float ryaw,float* val)
{
    if (radar_first)
    {
        radar_first = 0;
        last_rx = rx; last_ry = ry; last_yaw = ryaw;
        val[0] = rx; val[1] = ry; val[2] = ryaw;
        return;
    }
    // 跳变剔除（正负方向都要判）
    if (fabsf(rx - last_rx) > Radar_jump_max || fabsf(ry - last_ry) > Radar_jump_max)
    {
        val[0] = last_rx; val[1] = last_ry; val[2] = last_yaw;
        last_rx = rx; last_ry = ry; last_yaw = ryaw;
        return;
    }
    // 低通滤波
    val[0] = Filter_alpha * rx + (1 - Filter_alpha)* last_rx;
    val[1] = Filter_alpha * ry + (1 - Filter_alpha)* last_ry;
    float yaw_err = ryaw - last_yaw;
    if (yaw_err > M_PI)   yaw_err -= 2.0f * (float)M_PI;
    else if (yaw_err < -M_PI) yaw_err += 2.0f *(float) M_PI;
    val[2] = last_yaw + Filter_alpha * yaw_err;
    // 滑动窗口滤波
    // MAF_Filter(&x_mat,rx);
    // MAF_Filter(&y_mat,rx);
    // MAF_Filter(&yaw_mat,ryaw);
    last_rx = rx; last_ry = ry;last_yaw = ryaw;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 串口任务函数
void StartSerialCmdTask(void *argument)
{
    /* USER CODE BEGIN StartVisionCmdTask */
    Uart_rx_init(&serial_uart,&huart1,64);
    /* Infinite loop */
    for(;;)
    {
        if (serial_uart.line_rx_flag)
        {
            Parse_serial_line((char *)serial_uart.cmd_line);
            serial_uart.line_rx_flag = 0;
        }
        osDelay(5);
    }

    /* USER CODE END StartVisionCmdTask */
}

// 视觉串口接收解析函数
void StartVisionCmdTask(void *argument)
{
    /* USER CODE BEGIN StartVisionCmdTask */
    Filter_Init_MAF(&x_mat);
    Filter_Init_MAF(&y_mat);
    Filter_Init_MAF(&yaw_mat);
    Uart_rx_init(&vision_uart,&huart2,64);
    /* Infinite loop */
    for(;;)
    {
        if (vision_uart.line_rx_flag)
        {
            Parse_vision_line(vision_uart.cmd_line,vision_uart.line_rx_len);
            vision_uart.line_rx_flag = 0;
        }
        osDelay(5);
    }

    /* USER CODE END StartVisionCmdTask */
}

// 码盘+imu
void StartLocCmdTask(void *argument)
{
    /* USER CODE BEGIN StartVisionCmdTask */
    Uart_rx_init(&locator_uart,&huart3,64);
    /* Infinite loop */
    for(;;)
    {
        if (locator_uart.line_rx_flag)
        {
            Parse_location_line(locator_uart.cmd_line,locator_uart.line_rx_len);
            locator_uart.line_rx_flag = 0;
        }
        osDelay(5);
    }

    /* USER CODE END StartVisionCmdTask */
}

// 串口接收中断，用于将接收到的数据转移到队列
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        Uart_callback_func(&serial_uart,Size);
    }
    if (huart == &huart2)
    {
       Uart_callback_func(&vision_uart,Size);
    }
    if (huart == &huart3)
    {
        Uart_callback_func(&locator_uart,Size);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 浮点数解析函数，index为浮点数第一个出现的位置，用于取出完成的浮点数据
static  float Parse_float(char *cmd,uint8_t index)
{
    char* pos = cmd + index;
    float val = 0.0f;
    float flag =1.0f;// 正负数判断标志
    // 正负数判断
    if (*pos == '-')
    {
        pos ++;
        flag = -1.0f;
    }
    // 整数部分取值
    while (*pos >= '0' && *pos <= '9')
    {
        val = val * 10.0f + (float)(*pos - '0');
        pos++;
    }
    // 小数部分取值
    if (*pos == '.')
    {
        pos ++;
        float fac = 0.1f;
        while (*pos >= '0' && *pos <= '9')
        {
            val += (float)(*pos - '0')* fac;
            fac *= 0.1f;
            pos ++;
        }
    }
    return flag*val;
}

// 命令解析函数，用于解析上位机上送发来的命令
void Parse_serial_line(char *cmd)
{
    uint8_t mode = 0;

    // mode debug_mode x y  w speed_tar location_tar angle
    switch (cmd[0])
    {
        // mode
        case 'm':
            mode = (uint8_t)(cmd[5] - '0');
            // 模式切换时清除所有
            if (mode != Serial_cmd_structor.motor_mode)
            {
                Motor_Reset();
                Serial_cmd_structor.speed_tar = 0;
                Serial_cmd_structor.location_tar = 0;
                Serial_cmd_structor.x = 0;
                Serial_cmd_structor.y = 0;
                Serial_cmd_structor.w = 0;
            }
            Serial_cmd_structor.motor_mode = mode;
            SERIAL_printf("mode:%d\r\n",Serial_cmd_structor.motor_mode);
        break;
        // debug_mode
        case 'd':
            Serial_cmd_structor.debug_mode = (uint8_t)(cmd[11] - '0');
            SERIAL_printf("debug_mode:%d\r\n",Serial_cmd_structor.debug_mode);
        break;
        // x
        case 'x':
            Serial_cmd_structor.x = Parse_float(cmd,2);
            SERIAL_printf("vx:%f,vy:%f,w:%f\r\n",Serial_cmd_structor.x,Serial_cmd_structor.y,Serial_cmd_structor.w);
        break;
        // y
        case 'y':
            Serial_cmd_structor.y = Parse_float(cmd,2);
            SERIAL_printf("vx:%f,vy:%f,w:%f\r\n",Serial_cmd_structor.x,Serial_cmd_structor.y,Serial_cmd_structor.w);
        break;
        // w
        case 'w':
            Serial_cmd_structor.w = Parse_float(cmd,2) ;
            SERIAL_printf("vx:%f,vy:%f,w:%f\r\n",Serial_cmd_structor.x,Serial_cmd_structor.y,Serial_cmd_structor.w);
        break;
        // speed_tar
        case 's':
            Serial_cmd_structor.speed_tar = Parse_float(cmd,10);
            SERIAL_printf("speed_tar:%f\r\n",Serial_cmd_structor.speed_tar);
        break;
        // location_tar
        case 'l':
            Serial_cmd_structor.location_tar = Parse_float(cmd,13);
            SERIAL_printf("location_tar:%f\r\n",Serial_cmd_structor.location_tar);
        break;
        // angle
        case 'a':
            Serial_cmd_structor.angle = Parse_float(cmd,6);
            SERIAL_printf("angle:%f\r\n",Serial_cmd_structor.angle);
        break;
        default:
        break;
    }
    // pid参数调节，须在调试模式下进行，要对那个pid调参就赋值那个pid
    if (Serial_cmd_structor.debug_mode == DEBUG_TRUE)
    {
        PID_Structor* pid_ptr = NULL;
        switch (Serial_cmd_structor.motor_mode)
        {
            case MODE_NONE:

                break;
            case MODE_1:
                pid_ptr = &(m3508_data[Motor1_3508].speed_pid);
                break;
            case MODE_2:
                //pid_ptr = &(m3508_data[Motor1_3508].speed_pid);
                pid_ptr = &chassis_pos_vx;
                break;
            case MODE_3:
                if (Serial_cmd_structor.angle == 0)
                {
                    pid_ptr = &(m3508_data[Motor1_3508].speed_pid);
                }
                else if (Serial_cmd_structor.angle == 90)
                {
                    pid_ptr = &(m3508_data[Motor2_3508].speed_pid);
                }
                else if (Serial_cmd_structor.angle == 180)
                {
                    pid_ptr = &(m3508_data[Motor3_3508].speed_pid);
                }
                else
                {
                    pid_ptr = &(m3508_data[Motor4_3508].speed_pid);
                }
                break;
            default:
                break;
        }
        // 解析 pid参数
        if (pid_ptr != NULL)
        {
            // kp ki kd kt
            if (cmd[0] == 'k' && cmd[2] == ':')
            {
                switch (cmd[1])
                {
                case 'p':
                    Serial_cmd_structor.kp = Parse_float(cmd,3);
                    SERIAL_printf("kp:%f\r\n",Serial_cmd_structor.kp);
                    pid_ptr->kp = Serial_cmd_structor.kp;
                    break;
                case 'i':
                    Serial_cmd_structor.ki = Parse_float(cmd,3);
                    SERIAL_printf("ki:%f\r\n",Serial_cmd_structor.ki);
                    pid_ptr->ki = Serial_cmd_structor.ki;
                    break;
                case 'd':
                    Serial_cmd_structor.kd = Parse_float(cmd,3);
                    SERIAL_printf("kd:%f\r\n",Serial_cmd_structor.kd);
                    pid_ptr->kd = Serial_cmd_structor.kd;
                    break;
                case 't':
                    Serial_cmd_structor.kt = Parse_float(cmd,3);
                    SERIAL_printf("kt:%f\r\n",Serial_cmd_structor.kt);
                    break;
                default:
                    break;
                }
            }
            // for (uint8_t i = 1; i < 4; i++)
            // {
            //     m3508_data[i].speed_pid.kp = pid_ptr->kp;
            //     m3508_data[i].speed_pid.ki = pid_ptr->ki;
            //     m3508_data[i].speed_pid.kd = pid_ptr->kd;
            // }
            PID_ModifyParams(&chassis_pos_vy,Serial_cmd_structor.kp,Serial_cmd_structor.ki,Serial_cmd_structor.kd);
            SERIAL_printf("kp:%3f ki:%3f kd:%3f kt:%3f\r\n",pid_ptr->kp,pid_ptr->ki,pid_ptr->kd,Serial_cmd_structor.kt);
        }
        return;
    }
}

//视觉串口解析函数
void Parse_vision_line(const uint8_t* cmd, uint16_t len)
{
    // 换算倍率
    static float v_scale_rate = 1000.0f;
    static float pos_scale_rate = 10000.0f;

    // crc校验
    uint8_t crc_check = 0;
    for (uint8_t i = 0; i < 9; i++)
    {
        crc_check ^= cmd[i];
    }

    switch (cmd[0])
    {
    case 0x5A:
        if (len == 11 && cmd[1] == 0xA5 && cmd[2] == 0x01  && cmd[9] == crc_check && cmd[10] == 0xED)
        {
            Visual_Receive(cmd);
            Vision_cmd_structor.vx =  (float)vision_lia_raw.x / v_scale_rate;
            Vision_cmd_structor.vy =  (float)vision_lia_raw.y / v_scale_rate;
            Vision_cmd_structor.w  =  (float)vision_lia_raw.w / v_scale_rate;
            //SERIAL_printf("vision_speed:%f,%f,%f\r\n",Vision_cmd_structor.vx,Vision_cmd_structor.vy,Vision_cmd_structor.w);
        }
        break;
    case 0x7E:
        if (len == 14 && cmd[13] == 0x21)
        {
            Visual_Receice_pos(cmd);
            float rx   = (float)vision_lia_raw_pos.pos_x / pos_scale_rate;
            float ry   = (float)vision_lia_raw_pos.pos_y / pos_scale_rate;
            float ryaw = (float)vision_lia_raw_pos.yaw   / pos_scale_rate;

            float val[3];
            Radar_Filter( rx, ry, ryaw,val);
            Vision_cmd_structor.pos_x = val[0];
            Vision_cmd_structor.pos_y = val[1];
            Vision_cmd_structor.yaw   = val[2];
            //SERIAL_printf("%f,%f,%f,%f,%f\r\n",Vision_cmd_structor.pos_x,Vision_cmd_structor.pos_y,Vision_cmd_structor.yaw,Chassis.targetVel.v_x, Chassis.targetVel.v_y);
        }
        break;
    default:
        break;
    }
}

//码盘+imu解析函数
void Parse_location_line(const uint8_t* cmd,uint16_t len)
{
    uint8_t rx_buf[32];
    float rx_data[8];
    if (cmd[0] == '[' && cmd[1] == '8' && len == 36)
    {
        for (uint8_t i = 0; i < 32; i++)
        {
            rx_buf[i] = cmd[i+2];
        }
        memcpy(rx_data,rx_buf,sizeof(rx_buf));
        Encoder_imu_cmd_structor.x_count = rx_data[0];
        Encoder_imu_cmd_structor.y_count = rx_data[1];
        Encoder_imu_cmd_structor.x_turns = rx_data[2];
        Encoder_imu_cmd_structor.y_turns = rx_data[3];
        Encoder_imu_cmd_structor.x_distance = rx_data[4];
        Encoder_imu_cmd_structor.y_distance = rx_data[5];
        Encoder_imu_cmd_structor.yaw = rx_data[6];
        Encoder_imu_cmd_structor.yaw_nor = rx_data[7];
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//串口发送函数
void StartSerialTxTask(void *argument)
{
    /* USER CODE BEGIN StartSerialTXTask */
    /* Infinite loop */
    for(;;)
    {
        //SERIAL_printf("%f,%f,%f,%f,%f\r\n",Vision_cmd_structor.pos_x,Vision_cmd_structor.pos_y,Vision_cmd_structor.yaw,Chassis.targetVel.v_x, Chassis.targetVel.v_y);
        SERIAL_printf("%f,%f,%f,%f,%f,%f,%f,%f\r\n",Encoder_imu_cmd_structor.x_count,Encoder_imu_cmd_structor.y_count,Encoder_imu_cmd_structor.x_turns,Encoder_imu_cmd_structor.y_turns,
            Encoder_imu_cmd_structor.x_distance,Encoder_imu_cmd_structor.y_distance,Encoder_imu_cmd_structor.yaw,Encoder_imu_cmd_structor.yaw_nor);
        //SERIAL_printf("keys:%d,button:%d\r\n",nrf_chassis_cmd.keys,nrf_chassis_cmd.button);
        if ((Serial_cmd_structor.debug_mode == DEBUG_FALSE || nrf_chassis_cmd.keys == 4))
        {
            switch (Serial_cmd_structor.motor_mode)
            {
            case MODE_1:
                SERIAL_printf("mode1:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",Chassis.targetVel.v_x,Chassis.targetVel.v_y,Chassis.targetVel.w,
                                m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor1_3508].speed_pid.Act,
                                m3508_data[Motor2_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Act,
                                m3508_data[Motor3_3508].speed_pid.Tar,m3508_data[Motor3_3508].speed_pid.Act,
                                m3508_data[Motor4_3508].speed_pid.Tar,m3508_data[Motor4_3508].speed_pid.Act);
                break;
            case MODE_2:
                SERIAL_printf("%f,%f,%f,%f,%f\r\n",Chassis.GlobalPos.x,Chassis.GlobalPos.y,Chassis.GlobalPos.yaw,m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Tar);
                break;

            case MODE_3:
                SERIAL_printf("%f,%f,%f,%f,%f,%f,%f,%f\r\n",m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor1_3508].speed_pid.Act,
                                    m3508_data[Motor2_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Act,
                                    m3508_data[Motor3_3508].speed_pid.Tar,m3508_data[Motor3_3508].speed_pid.Act,
                                    m3508_data[Motor4_3508].speed_pid.Tar,m3508_data[Motor4_3508].speed_pid.Act);
                break;
            default:
                break;
            }
        }
        osDelay(30);
    }
    /* USER CODE END StartSerialTXTask */
}

// 错误处理
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Uart_error_callback(&serial_uart);
    }
    else if (huart->Instance == USART2)
    {
        Uart_error_callback(&vision_uart);
    }
    else if (huart->Instance == USART3)
    {
        Uart_error_callback(&locator_uart);
    }
}