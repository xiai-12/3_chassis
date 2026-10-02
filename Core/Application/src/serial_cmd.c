#include "main.h"
#include "motor_control.h"
#include "motion_plane.h"

#define Cmd_line_max 64  // 串口接收最大数量
uint8_t Serial_rxdata;   // 串口接收单字节
static uint8_t cmd_line[Cmd_line_max]; // 串口接收缓存数组
static uint8_t cmd_line_len = 0;       // 数组索引

// 函数声明
float Parse_float(char *cmd,uint8_t index); // 串口浮点数解析函数，index为浮点数第一位的索引
void Parse_cmd_line(char *cmd);// 串口命令解析函数

// 串口任务函数，用于将队列的数据转移到缓存数组里，同时对串口命令进行解析
void StartSerialCmdTask(void *argument)
{
    /* USER CODE BEGIN StartSerialTask */
    HAL_UART_Receive_IT(&huart1, &Serial_rxdata, 1);
    /* Infinite loop */
    for(;;)
    {
        uint8_t rx_data;
        // 判断是否有新数据
        if (osMessageQueueGet(SerialRxQueueHandle,&rx_data,0,100) != osOK)
        {
            continue;
        }
        // 判断命令是否发送完，命令统一以“\n”结束
        if (rx_data == '\n')
        {
            cmd_line[cmd_line_len] = '\0';
            Parse_cmd_line((char *)cmd_line);
            cmd_line_len = 0;// 解析完后，软件实现数组清空
        }
        else if (cmd_line_len < Cmd_line_max-1)
        {
            cmd_line[cmd_line_len++] = rx_data;
        }
        else
        {
            cmd_line_len = 0;
        }
    }
    /* USER CODE END StartSerialTask */
}


// 串口接收中断，用于将接收到的数据转移到队列
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        osMessageQueuePut(SerialRxQueueHandle, &Serial_rxdata, 0, 0);
        HAL_UART_Receive_IT(&huart1, &Serial_rxdata, 1);
    }
}

// 浮点数解析函数，index为浮点数第一个出现的位置，用于取出完成的浮点数据
float Parse_float(char *cmd,uint8_t index)
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
/*
 * 命令包括：
 * 1.模式选择： mode:[%f]  用于选择电机运动模式
 * 2.调试选择： debug_mode:[%f] 用于选择是否开启调试模式
 * 3.速度目标值设置： speed_tar:[%f] 用于设定目标速度期望值
 * 4.位置目标值设置： location_tar:[%f] 用于设定目标位置期望值
 * 5.参数调节： kp\ki\kd:[%f] 用于调节pid参数，需在调试模式下进行
 */
void Parse_cmd_line(char *cmd)
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
        SERIAL_printf("vx:%f\r\n",Serial_cmd_structor.vx);
    }
    if (strncmp(cmd,"vy:",3) == 0)
    {
        float vy = Parse_float(cmd,3);
        vy = (vy > 5.0f)?5.0f:((vy<-5.0f)?-5.0f:vy);
        Serial_cmd_structor.vy = vy;
        SERIAL_printf("vy:%f\r\n",Serial_cmd_structor.vy);
    }
    if (strncmp(cmd,"w:",2) == 0)
    {
        float w = Parse_float(cmd,2);
        w = (w > 8.0f)?8.0f:((w<-8.0f)?-8.0f:w);
        Serial_cmd_structor.w = w;
        SERIAL_printf("w:%f\r\n",Serial_cmd_structor.w);
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
    if (strncmp(cmd,"acc:",4) == 0)
    {
        Serial_cmd_structor.acc = Parse_float(cmd,4);
        SERIAL_printf("acc:%f\r\n",Serial_cmd_structor.acc);
    }

    // 最大速度选择
    if (strncmp(cmd,"v_max:",6) == 0)
    {
        Serial_cmd_structor.v_max = Parse_float(cmd,6);
        SERIAL_printf("v_max:%f\r\n",Serial_cmd_structor.v_max);
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
            SERIAL_printf("kp:%3f ki:%3f kd:%3f\r\n",pid_ptr->kp,pid_ptr->ki,pid_ptr->kd);
        }
        return;
    }
}

/*
 * 串口发送函数，根据模式选择发送不同内容
 */
void StartSerialTxTask(void *argument)
{
    /* USER CODE BEGIN StartSerialTXTask */
    /* Infinite loop */
    for(;;)
    {

        if (Serial_cmd_structor.debug_mode == DEBUG_FALSE)
        {
            switch (Serial_cmd_structor.motor_mode)
            {
            case MODE_1:
                SERIAL_printf("mode1:%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",nrf_chassis_cmd.vx,nrf_chassis_cmd.vy,nrf_chassis_cmd.w,m3508_data[Motor1_3508].speed_pid.Tar,
                                m3508_data[Motor1_3508].speed_pid.Act,m3508_data[Motor1_3508].speed_pid.Out,chassis_velocity.v_x,chassis_velocity.v_y,chassis_velocity.w);

                // SERIAL_printf("motor1:%f,%f,%f,%f,%f,%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w,
                // m3508_data[Motor1_3508].speed_pid.Tar,m3508_data[Motor1_3508].speed_pid.Out,m3508_data[Motor1_3508].speed_pid.Act);
                //  SERIAL_printf("motor2:%f,%f,%f,%f,%f,%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w,
                //  m3508_data[Motor2_3508].speed_pid.Tar,m3508_data[Motor2_3508].speed_pid.Out,m3508_data[Motor2_3508].speed);
                //  SERIAL_printf("motor3:%f,%f,%f,%f,%f,%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w,
                //  m3508_data[Motor3_3508].speed_pid.Tar,m3508_data[Motor3_3508].speed_pid.Out,m3508_data[Motor3_3508].speed);
                //  SERIAL_printf("motor4:%f,%f,%f,%f,%f,%f\r\n",Serial_cmd_structor.vx,Serial_cmd_structor.vy,Serial_cmd_structor.w,
                //  m3508_data[Motor4_3508].speed_pid.Tar,m3508_data[Motor4_3508].speed_pid.Out,m3508_data[Motor4_3508].speed);

                //SERIAL_printf("chassis:%f,%f,%f,%f\r\n",chassis_velocity.v_x,chassis_velocity.v_y,chassis_velocity.w,chassis_yaw);
                break;
            case MODE_2:
                break;
            case MODE_3:
                break;
            default:
                break;
            }
        }
        osDelay(20);
    }
    /* USER CODE END StartSerialTXTask */
}