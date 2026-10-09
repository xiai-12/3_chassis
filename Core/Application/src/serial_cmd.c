#include "main.h"
#include "motor_control.h"
#include "motion_plane.h"
#include "Vision/connect.h"
#include "string.h"
#include "Filter/filter.h"

#define serial_line_max 64                   // 串口接收最大数量
static uint8_t serial_line[serial_line_max]; // 串口接收缓存数组
volatile uint8_t serial_flag = 0;            // 串口中断标志位
volatile uint16_t serial_len  = 0;           // 串口数据长度

#define vision_line_max 64
static uint8_t vision_line[vision_line_max];
volatile uint8_t vision_flag = 0;
volatile uint16_t vision_len  = 0;

// 函数声明
static  float Parse_float(char *cmd,uint8_t index); // 串口浮点数解析函数，index为浮点数第一位的索引


/* ===================== 雷达滤波 ===================== */
#define RADAR_TS         0.01f
#define RADAR_JUMP_MAX   0.04f

static Filter_LP_t lpf_x, lpf_y, lpf_yaw;
static float   last_rx, last_ry;
static uint8_t med_n = 0, first = 1;
static float   medx[3] = {0}, medy[3] = {0};

/* 角度低通：先对“误差”做 ±π 归一，避免跨 ±π 时输出乱跳 */
static float LPF_Yaw(Filter_LP_t *f, float yaw)
{
    float d = yaw - f->lastYn;
    while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    f->lastYn += f->alpha * d;
    return f->lastYn;
}

/* 每收到一帧雷达数据调用一次；返回 1 = 采纳，0 = 当跳变丢弃 */
void Radar_Filter(float rx, float ry, float ryaw, float *ox, float *oy, float *oyaw)
{
    if (first) {
        last_rx = rx; last_ry = ry;
        lpf_x.lastYn = rx; lpf_y.lastYn = ry; lpf_yaw.lastYn = ryaw;
        medx[0]=medx[1]=medx[2]=rx; medy[0]=medy[1]=medy[2]=ry; med_n = 3;first = 0;
        *ox = rx; *oy = ry; *oyaw = ryaw;
    }
    // 跳变剔除
    float dx = rx - last_rx, dy = ry - last_ry;
    if (sqrtf(dx*dx + dy*dy) > RADAR_JUMP_MAX ) {
        *ox = lpf_x.lastYn; *oy = lpf_y.lastYn; *oyaw = lpf_yaw.lastYn;
    }
    last_rx = rx; last_ry = ry;
    // 中值滤波
    medx[2]=medx[1]; medx[1]=medx[0]; medx[0]=rx;
    medy[2]=medy[1]; medy[1]=medy[0]; medy[0]=ry;
    float mx = (medx[0]>medx[1]) ? ((medx[1]>medx[2])?medx[1]:((medx[0]>medx[2])?medx[2]:medx[0]))
                                 : ((medx[0]>medx[2])?medx[0]:((medx[1]>medx[2])?medx[2]:medx[1]));
    float my = (medy[0]>medy[1]) ? ((medy[1]>medy[2])?medy[1]:((medy[0]>medy[2])?medy[2]:medy[0]))
                                 : ((medy[0]>medy[2])?medy[0]:((medy[1]>medy[2])?medy[2]:medy[1]));
    // 低通滤波
    *ox   = LPF_Filter(&lpf_x, mx);
    *oy   = LPF_Filter(&lpf_y, my);
    *oyaw = LPF_Yaw(&lpf_yaw, ryaw);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 串口任务函数，用于将队列的数据转移到缓存数组里，同时对串口命令进行解析
void StartSerialCmdTask(void *argument)
{
    /* USER CODE BEGIN StartVisionCmdTask */
    // 使用Ex函数，接收不定长数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, serial_line,sizeof(serial_line));
    // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    /* Infinite loop */
    for(;;)
    {
        if (serial_flag)
        {
            serial_flag = 0;
            Parse_serial_line((char *)serial_line);
        }
        osDelay(5);
    }

    /* USER CODE END StartVisionCmdTask */
}


// 视觉串口接收解析函数
void StartVisionCmdTask(void *argument)
{
    /* USER CODE BEGIN StartVisionCmdTask */
    // 使用Ex函数，接收不定长数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vision_line,sizeof(vision_line));
    // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    Filter_Init_LPF(&lpf_x,   RADAR_TS, 5.0f);
    Filter_Init_LPF(&lpf_y,   RADAR_TS, 5.0f);
    Filter_Init_LPF(&lpf_yaw, RADAR_TS, 3.0f);
    /* Infinite loop */
    for(;;)
    {
        if (vision_flag)
        {
            vision_flag = 0;
            Parse_vision_line(vision_line);
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
        serial_len  = Size;
        serial_flag = 1;
        /* 必须在中断里立刻重新挂起，否则 IDLE 之后 DMA 已经停了 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, serial_line, sizeof(serial_line));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

    if (huart == &huart2)
    {
        vision_len  = Size;
        vision_flag = 1;
        /* 必须在中断里立刻重新挂起，否则 IDLE 之后 DMA 已经停了 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vision_line, sizeof(vision_line));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
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
    // 模式选择
    if (strncmp(cmd,"mode:",5) == 0)
    {
        uint8_t mode = (uint8_t)(cmd[5] - '0');
        // 模式切换时清除所有
        if (mode != Serial_cmd_structor.motor_mode)
        {
            Motor_Reset();
            Serial_cmd_structor.speed_tar = 0;
            Serial_cmd_structor.location_tar = 0;
            Serial_cmd_structor.vx = 0;
            Serial_cmd_structor.vy = 0;
            Serial_cmd_structor.w = 0;
        }
        Serial_cmd_structor.motor_mode = mode;
        SERIAL_printf("mode:%d\r\n",Serial_cmd_structor.motor_mode);
    }

    if (strncmp(cmd,"vx:",3) == 0)
    {
        float vx = Parse_float(cmd,3);
        vx = (vx > 5.0f)?5.0f:((vx<-5.0f)?-5.0f:vx);
        Serial_cmd_structor.vx = vx;
        SERIAL_printf("vx:%f,vy:%f,w:%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w);
    }
    if (strncmp(cmd,"vy:",3) == 0)
    {
        float vy = Parse_float(cmd,3);
        vy = (vy > 5.0f)?5.0f:((vy<-5.0f)?-5.0f:vy);
        Serial_cmd_structor.vy = vy;
        SERIAL_printf("vx:%f,vy:%f,w:%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w);
    }
    if (strncmp(cmd,"w:",2) == 0)
    {
        float w = Parse_float(cmd,2);
        w = (w > 8.0f)?8.0f:((w<-8.0f)?-8.0f:w);
        Serial_cmd_structor.w = w;
        SERIAL_printf("vx:%f,vy:%f,w:%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w);
    }

    // 前馈系数选择
    if (strncmp(cmd,"kt:",3) == 0)
    {
        Serial_cmd_structor.kt = Parse_float(cmd,3);
        SERIAL_printf("kt:%f\r\n",Serial_cmd_structor.kt);
    }

    // 是否开启调试模式
    if (strncmp(cmd,"debug_mode:",11) == 0)
    {
        Serial_cmd_structor.debug_mode = (uint8_t)(cmd[11] - '0');
        SERIAL_printf("debug_mode:%d\r\n",Serial_cmd_structor.debug_mode);
    }
    // 速度期望值选择
    if (strncmp(cmd,"speed_tar:",10) == 0)
    {
        Serial_cmd_structor.speed_tar = Parse_float(cmd,10);
        SERIAL_printf("speed_tar:%f\r\n",Serial_cmd_structor.speed_tar);
    }
    // 位置期望值选择
    if (strncmp(cmd,"location_tar:",13) == 0)
    {
        Serial_cmd_structor.location_tar = Parse_float(cmd,13);
        SERIAL_printf("location_tar:%f\r\n",Serial_cmd_structor.location_tar);
    }

    // 加速度期望值选择
    if (strncmp(cmd,"angle:",6) == 0)
    {
        Serial_cmd_structor.angle = Parse_float(cmd,6);
        SERIAL_printf("angle:%f\r\n",Serial_cmd_structor.angle);
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
                pid_ptr = &(m3508_data[Motor1_3508].speed_pid);
                break;
            case MODE_3:
                break;
            default:
                break;
        }
        // 解析 pid参数
        if (pid_ptr != NULL)
        {
            if (strncmp(cmd,"kp:",3) == 0)
            {
                pid_ptr->kp = Parse_float(cmd,3);
            }
            else if (strncmp(cmd,"ki:",3) == 0)
            {
                pid_ptr->ki = Parse_float(cmd,3);
            }
            else if (strncmp(cmd,"kd:",3) == 0)
            {
                pid_ptr->kd = Parse_float(cmd,3);
            }
            for (uint8_t i = 1; i < 4; i++)
            {
                m3508_data[i].speed_pid.kp = pid_ptr->kp;
                m3508_data[i].speed_pid.ki = pid_ptr->ki;
                m3508_data[i].speed_pid.kd = pid_ptr->kd;
            }
            SERIAL_printf("kp:%3f ki:%3f kd:%3f kt:%3f\r\n",pid_ptr->kp,pid_ptr->ki,pid_ptr->kd,Serial_cmd_structor.kt);
        }
        return;
    }
}


/*
 * 视觉串口解析函数
 */
void Parse_vision_line(uint8_t* cmd)
{
    static float v_scale_rate = 1000.0f;
    static float pos_scale_rate = 10000.0f;
    uint8_t crc_check = 0;
    for (uint8_t i = 0; i < 9; i++)
    {
        crc_check ^= cmd[i];
    }

    static uint8_t count = 0;
    switch (cmd[0])
    {
    case 0x5A:
        if (vision_len == 11 && cmd[1] == 0xA5 && cmd[2] == 0x01  && cmd[9] == crc_check && cmd[10] == 0xED)
        {
            Visual_Receive(cmd);
            Vision_cmd_structor.vx =  (float)vision_lia_raw.x / v_scale_rate;
            Vision_cmd_structor.vy =  (float)vision_lia_raw.y / v_scale_rate;
            Vision_cmd_structor.w  =  (float)vision_lia_raw.w / v_scale_rate;

            //SERIAL_printf("vision_speed:%f,%f,%f\r\n",Vision_cmd_structor.vx,Vision_cmd_structor.vy,Vision_cmd_structor.w);
        }
        break;
    case 0x7E:
        if (vision_len == 14 && cmd[13] == 0x21)
        {
            Visual_Receice_pos(cmd);
           //  Vision_cmd_structor.pos_x = 0.80f * (float)vision_lia_raw_pos.pos_x / pos_scale_rate + 0.20f * Vision_cmd_structor.pos_x;
           //  Vision_cmd_structor.pos_y = 0.80f * (float)vision_lia_raw_pos.pos_y / pos_scale_rate + 0.20f * Vision_cmd_structor.pos_y;
           //  Vision_cmd_structor.yaw   = 0.80f * (float)vision_lia_raw_pos.yaw   / pos_scale_rate  + 0.20f * Vision_cmd_structor.yaw;
            float rx   = (float)vision_lia_raw_pos.pos_x / pos_scale_rate;
            float ry   = (float)vision_lia_raw_pos.pos_y / pos_scale_rate;
            float ryaw = (float)vision_lia_raw_pos.yaw   / pos_scale_rate;

            float fx, fy, fyaw;
            Radar_Filter( rx, ry, ryaw,&fx,&fy,&fyaw);

            Vision_cmd_structor.pos_x = fx;
            Vision_cmd_structor.pos_y = fy;
            Vision_cmd_structor.yaw   = fyaw;
            if (count >= 5)
            {
                SERIAL_printf("%f,%f,%f,%f,%f\r\n",Vision_cmd_structor.pos_x,Vision_cmd_structor.pos_y,Vision_cmd_structor.yaw,
                    m3508_data[Motor2_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Act);
                count =0;
            }
            count ++;
        }
        break;
    default:
        break;
    }


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 串口发送函数，根据模式选择发送不同内容
 */
void StartSerialTxTask(void *argument)
{
    /* USER CODE BEGIN StartSerialTXTask */
    /* Infinite loop */
    for(;;)
    {
        //SERIAL_printf("keys:%d,button:%d\r\n",nrf_chassis_cmd.keys,nrf_chassis_cmd.button);
        if ((Serial_cmd_structor.debug_mode == DEBUG_FALSE || nrf_chassis_cmd.keys == 4))
        {
            switch (Serial_cmd_structor.motor_mode)
            {
            case MODE_1:
                SERIAL_printf("mode1:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
                                Chassis.targetVel.v_x,Chassis.targetVel.v_y,Chassis.targetVel.w,
                                m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor1_3508].speed_pid.Act,
                                m3508_data[Motor2_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Act,
                                m3508_data[Motor3_3508].speed_pid.Tar,m3508_data[Motor3_3508].speed_pid.Act,
                                m3508_data[Motor4_3508].speed_pid.Tar,m3508_data[Motor4_3508].speed_pid.Act);
                break;
            case MODE_2:
                // SERIAL_printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",Chassis.targetVel.v_x,Chassis.targetVel.v_y,Chassis.targetVel.w,
                //                 TracePlane.vref,TracePlane.pos_act,TracePlane.pos_target,
                //                 chassis_velocity.v_x,chassis_velocity.v_y,chassis_position.v_x,chassis_position.v_y,
                //                 m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor1_3508].speed_pid.Act,
                //                 m3508_data[Motor2_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Act,
                //                 m3508_data[Motor3_3508].speed_pid.Tar,m3508_data[Motor3_3508].speed_pid.Act,
                //                 m3508_data[Motor4_3508].speed_pid.Tar,m3508_data[Motor4_3508].speed_pid.Act);

                // SERIAL_printf("%f,%f,%f,%f,%f,%f,%f\r\n",Chassis.targetVel.v_x,Chassis.targetVel.v_y,Chassis.targetVel.w,TracePlane.pos_act,TracePlane.pos_target
                //     ,m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Tar);

                break;
            case MODE_3:
                break;
            default:
                break;
            }
        }
        osDelay(100);
    }
    /* USER CODE END StartSerialTXTask */
}

/* 拔线/悬空/过载后 HAL 会停掉 DMA 接收，这里把它重新挂上 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, serial_line, sizeof(serial_line));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vision_line, sizeof(vision_line));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}