#include "main.h"
#include "Motion_control.h"
#include "PID.h"
#include "MPU6050.h" 
#include "MotorDriver.h"
#include "Encoder.h"      // 包含编码器头文件
#include <math.h> // 用于fabsf
#include "Sensor.h"
#include "OLED.h"
#include "Odometry.h" // 包含新的里程计模块
#include "Alert.h" 


// 为角度控制专门创建一个PID控制器实例
static PID_Controller_t pid_angle;
static uint8_t is_angle_pid_inited = 0;

// --- 速度控制PID (新增) ---
static PID_Controller_t left_motor_pid;
static PID_Controller_t right_motor_pid;
static PID_Controller_t pid_track; // 为循迹新增一个PID控制器
// static uint8_t is_speed_pids_inited = 0;

// --- 新增：为直行航向角添加一个一阶低通滤波器 ---
static float filtered_yaw = 0.0f;
static uint8_t is_yaw_filter_inited = 0;

// --- 状态机及目标管理变量 ---
volatile Motion_State_t g_motion_state = MOTION_STATE_IDLE;
volatile int32_t g_target_distance_counts = 0;
volatile int32_t g_base_target_speed = 0;
volatile float g_target_yaw = 0.0f;
static float g_track_entry_yaw = 0.0f;

// --- 自动任务状态机 ---


TaskState_t g_current_task_state = TASK_STATE_IDLE;
static const float TASK3_TURN_ANGLE = 38.66f; // 任务3转向角度, 定义为邻边100,对边80的直角三角形锐角(较小角): atan(80/100)
static uint8_t g_active_task_context = 0; // 记录当前运行的主任务 (2, 3, or 4)
static uint8_t task4_loop_counter = 0;    // 任务4的循环计数器
uint8_t g_current_task_id = 0; // 记录当前激活的任务编号 (1, 2, 3, 4), 0代表无任务



void Motion_Init(void)
{
    Odometry_Init();
    // 初始化速度环PID
    PID_Init(&left_motor_pid, 20.0f, 0.8f, 0.15f, -950, 950, 100.0f);                   
	PID_Init(&right_motor_pid, 20.0f, 0.8f, 0.1f, -950, 950, 100.0f);

    // 初始化角度环PID
    PID_Init(&pid_angle, 20.0f, 0.35f, 300.0f, -400, 400, 90);
    
    // 初始化循迹环PID (!!!这些值需要你根据实际情况调试!!!)
    // 循迹环作为外环，输出的是对基础速度的修正量，单位是: counts/编码器更新周期
    PID_Init(&pid_track, 0.17f, 0.0f, 1.5f, -35, 35, 20);
}

void Motion_Turn_Degrees_Start(float angle_to_turn)
{
    if (g_motion_state == MOTION_STATE_IDLE)
    {
        // 1. 设定目标角度 (使用0-360度体系)
        float initial_angle = g_yaw; // 直接使用全局变量
        if (initial_angle < 0.0f) initial_angle += 360.0f; // 确保初始角度也在0-360范围

        g_target_yaw = initial_angle + angle_to_turn;

        // 将目标角度归一化到 0 ~ 360 度范围
        while (g_target_yaw >= 360.0f) g_target_yaw -= 360.0f;
        while (g_target_yaw < 0.0f)   g_target_yaw += 360.0f;

        // 2. 重置PID控制器并启动状态机
        PID_Reset(&pid_angle);
        PID_Set_Setpoint_Float(&pid_angle, g_target_yaw); // PID目标设为0-360范围内的浮点角度
        g_motion_state = MOTION_STATE_TURN;
    }
}

/**
 * @brief [非阻塞式] 命令小车开始转向到指定的绝对角度
 * @param target_angle 目标角度, 范围 [0.0, 360.0)
 */
void Motion_Turn_To_Angle_Start(float target_angle)
{
    // 只有在空闲状态下才能启动新任务
    if (g_motion_state == MOTION_STATE_IDLE)
    {
        // 1. 范围检查与设定目标
        if(target_angle < 0.0f || target_angle >= 360.0f)
        {
            return; // 忽略无效的目标角度
        }
        g_target_yaw = target_angle;

        // 2. 重置PID控制器并启动状态机
        PID_Reset(&pid_angle);
        PID_Set_Setpoint_Float(&pid_angle, g_target_yaw);
        g_motion_state = MOTION_STATE_TURN_TO_ANGLE;
    }
}

void Motion_Go_Straight_Start(int32_t base_speed, int32_t total_counts)
{
    // 只有在空闲状态下才能启动新任务
    if (g_motion_state == MOTION_STATE_IDLE)
    {
        // 1. 设置目标
        g_base_target_speed = base_speed;
        g_target_distance_counts = total_counts;
        
        // --- 角度环目标设定 ---
        // 直接使用由TIM10中断持续更新的全局变量 g_yaw 作为初始目标航向。
        // 不再在此处调用 MPU6050_Data_Update()，以维持任务分离的良好架构。
        g_target_yaw = g_yaw; 


        // 2. 重置所有状态
        Encoder_Reset(ENCODER_CH1);
        Encoder_Reset(ENCODER_CH2);
        Odometry_Sync_With_Encoder_Reset();
        PID_Reset(&left_motor_pid);
        PID_Reset(&right_motor_pid);

        
        // 3. 启动运动状态机
        g_motion_state = MOTION_STATE_GO_STRAIGHT;
    }
}

Motion_State_t Motion_Get_State(void)
{
    return g_motion_state;
}

/**
 * @brief [非阻塞式] 命令小车开始循迹
 */
void Motion_Track_Start(int32_t base_speed)
{
    if (g_motion_state == MOTION_STATE_IDLE)
    {
        g_base_target_speed = base_speed;
        PID_Reset(&pid_track);
        PID_Reset(&left_motor_pid);
        PID_Reset(&right_motor_pid);
        // 新增：记录进入循迹时的yaw角（0~360）
        g_track_entry_yaw = g_yaw;
        if (g_track_entry_yaw < 0.0f) g_track_entry_yaw += 360.0f;
        g_motion_state = MOTION_STATE_TRACK;
    }
}

/**
 * @brief 停止所有运动
 */
void Motion_Stop(void)
{
    Motor_Brake(2);
    g_motion_state = MOTION_STATE_IDLE;
}

/**
 * @brief 运动控制更新函数，必须在固定的定时器中断中被调用
 */
void Motion_Update_In_Interrupt(void)
{
    static uint32_t stop_time = 0;

    // --- 核心更新：始终更新编码器和里程计 ---
    Encoder_Update(ENCODER_CH1);
    Encoder_Update(ENCODER_CH2);
    Odometry_Update();

    // --- 全局角度预处理 ---
    // 假设 g_yaw 由外部在[-180, 180]更新，我们在此统一转换到 [0, 360] 范围进行计算
    float yaw_360 = g_yaw;
    if (yaw_360 < 0.0f)
    {
        yaw_360 += 360.0f;
    }
    
    if (g_motion_state == MOTION_STATE_GO_STRAIGHT)
    {
        // --- 核心更新：首先更新编码器状态 ---
        // 这一步至关重要，它会计算出最新的速度和路程，并存入编码器模块的内部变量中

        // --- 任务：执行速度闭环+陀螺仪方向修正的直行 ---

        // 外环：方向环P控制器
        //-----------------------------------------------------
        
        // 1. 对陀螺仪的yaw值进行一阶低通滤波，以减少抖动 (处理0/360边界)
        if (!is_yaw_filter_inited) {
            // 第一次进入时，用当前的yaw值初始化滤波器，避免突变
            filtered_yaw = yaw_360;
            is_yaw_filter_inited = 1;
        }
        else
        {
            float filter_error = yaw_360 - filtered_yaw;
            // 归一化滤波误差到最短路径
            if (filter_error > 180.0f)       filter_error -= 360.0f;
            else if (filter_error < -180.0f) filter_error += 360.0f;

            const float YAW_FILTER_ALPHA = 0.2f; // 滤波系数，可调(0.0-1.0)
            filtered_yaw += YAW_FILTER_ALPHA * filter_error;
            
            // 将滤波结果归一化回0-360范围
            if (filtered_yaw >= 360.0f) filtered_yaw -= 360.0f;
            if (filtered_yaw < 0.0f)    filtered_yaw += 360.0f;
        }

        // 2. 计算航向误差
        float yaw_error = g_target_yaw - filtered_yaw;

        // 将误差归一化到最短路径(-180 ~ +180)
        if (yaw_error > 180.0f)       yaw_error -= 360.0f;
        else if (yaw_error < -180.0f) yaw_error += 360.0f;
        
        // 2. P控制器计算修正量
        const float YAW_CORRECTION_KP = 0.05f; // 此P增益是关键调试参数！
        float raw_correction = YAW_CORRECTION_KP * yaw_error;
        int32_t yaw_correction = (int32_t)roundf(raw_correction);

        // 保证只要有误差，最小输出±1
        if (yaw_correction == 0 && fabsf(yaw_error) > 0.0f) {
         yaw_correction = (raw_correction > 0) ? 1 : -1;
        }


        // 内环：速度环PID计算
        //-----------------------------------------------------
        // 1. 根据方向环的修正，计算左右轮的期望速度
        int32_t target_speed_left  = g_base_target_speed - yaw_correction;
        int32_t target_speed_right = g_base_target_speed + yaw_correction;

        // 2. 读取编码器测得的实际速度 (现在可以安全地获取已更新的值)
        int32_t actual_speed_left = Encoder_Get_Speed(ENCODER_CH1);
        int32_t actual_speed_right = Encoder_Get_Speed(ENCODER_CH2); 
        
        // (可选) 将调试信息存入全局变量，以便在主循环中显示
        g_display_speed_left = actual_speed_left;
        g_display_speed_right = actual_speed_right;
        
        // 3. 分别计算左右轮的速度PID输出PWM值
        PID_Set_Setpoint(&left_motor_pid, target_speed_left);
        int32_t pwm_left = PID_Calculate(&left_motor_pid, actual_speed_left);

        PID_Set_Setpoint(&right_motor_pid, target_speed_right);
        int32_t pwm_right = PID_Calculate(&right_motor_pid, actual_speed_right);

        // 4. 执行电机控制
        Motor_Control(pwm_left, pwm_right);
        
        // 检查是否到达目标距离
        //-----------------------------------------------------
        // 现在可以安全地获取已更新的值
        int32_t left_counts = Encoder_Get_Counts(ENCODER_CH1);
        int32_t right_counts = Encoder_Get_Counts(ENCODER_CH2);
        int32_t current_distance = (left_counts + right_counts) / 2;
        
        // (可选) 将调试信息存入全局变量
        g_display_counts_left = left_counts;
        g_display_counts_right = right_counts;
        
        if (abs(current_distance) >= abs(g_target_distance_counts)) 
        {
            Motor_Brake(2); // 到达目标，刹车
            // 读取当前两轮速度，确保两轮速度都小于3再将状态置空闲
            int32_t speed_left = Encoder_Get_Speed(ENCODER_CH1);
            int32_t speed_right = Encoder_Get_Speed(ENCODER_CH2);
            if (abs(speed_left) < 1 && abs(speed_right) < 1) {
                if (stop_time == 0) {
                    stop_time = HAL_GetTick();
                }
                if (HAL_GetTick() - stop_time >= 80) {
                    g_motion_state = MOTION_STATE_IDLE; // 任务完成，返回空闲状态
                    stop_time = 0; // 重置计时
                }
            } else {
                stop_time = 0; // 只要速度不满足条件就重置计时
            }
            is_yaw_filter_inited = 0; // 重置滤波器初始化标志
        }
    }
    else if (g_motion_state == MOTION_STATE_TURN || g_motion_state == MOTION_STATE_TURN_TO_ANGLE)
    {
        // --- 任务：执行原地转向 (包括相对角度和绝对角度) ---

        // 1. 计算角度误差 (使用0-360度体系)
        float error = g_target_yaw - yaw_360;
        // 归一化误差到最短路径(-180 ~ +180)
        if (error > 180.0f)       error -= 360.0f;
        else if (error < -180.0f) error += 360.0f;

        // 2. 检查是否到达目标 (为调试修改)
        // 检查角度误差和角速度，确保转向稳定完成
        if ((fabsf(error) < 1.5f && fabsf(g_gz_dps) < 0.5f) ) {
            Motor_Brake(2);
            g_motion_state = MOTION_STATE_IDLE; // 满足角速度和角度要求就置空闲
            return; // 任务完成
        }

        // 3. PID计算 (处理0/360边界)
        // 通过构造一个假的测量值来让PID控制器使用我们已经计算好的归一化误差 `error`
        // 假测量值 = 目标值 - 误差值
        // 这样PID内部计算的误差 = 目标值 - 假测量值 = 误差值
        float fake_measurement = g_target_yaw - error;
        int32_t pwm_output = PID_Calculate_Float(&pid_angle, fake_measurement);

        // 死区补偿：如果输出绝对值小于60，线性映射到85；60到85直接设为85
        if (pwm_output > 0 && pwm_output < 60) {
            pwm_output = pwm_output * (85.0f / 60.0f);
        } else if (pwm_output > 0 && pwm_output < 85) {
            pwm_output = 85;
        } else if (pwm_output < 0 && pwm_output > -60) {
            pwm_output = pwm_output * (85.0f / 60.0f);
        } else if (pwm_output < 0 && pwm_output > -85) {
            pwm_output = -85;
        }

        
        
        Motor_Control(-pwm_output, pwm_output);
        
    }
    else if (g_motion_state == MOTION_STATE_TRACK)
    {
        // --- 任务：执行循迹（串级PID：外环修正内环目标） ---
        static uint32_t line_lost_start_time = 0;
        static int16_t last_known_error = 0;
        const uint32_t LINE_LOST_TIMEOUT_MS = 80; // 丢线容忍时间(ms)

        int16_t current_line_error = Sensor_Get_Line_Error();

        if (current_line_error == -1) { // 丢线了
            if (line_lost_start_time == 0) {
                // 第一次检测到丢线，记录时间戳
                line_lost_start_time = HAL_GetTick();
            } else if (HAL_GetTick() - line_lost_start_time > LINE_LOST_TIMEOUT_MS) {
                // 丢线时间超过容忍值，停止循迹
                Motion_Stop();
                line_lost_start_time = 0; // 重置计时器
                last_known_error = 0; // 重置误差
                return;
            }
            // 在容忍时间内，继续使用上一次有效的误差值
        } else {
            // 重新找到线，重置计时器并更新最后误差
            line_lost_start_time = 0;
            last_known_error = current_line_error;
        }
        
        // 0. 核心更新：更新编码器，获取当前速度
        // Encoder_Update(ENCODER_CH1); // 已移到函数顶部
        // Encoder_Update(ENCODER_CH2); // 已移到函数顶部
        
        // 1. 外环(循迹环)PID计算速度修正量
        PID_Set_Setpoint(&pid_track, 0); // 目标是误差为0
        int32_t speed_correction = PID_Calculate(&pid_track, last_known_error); // 使用最后一次有效误差
        
        // 2. 根据循迹修正，计算左右轮的期望速度
        int32_t target_speed_left  = g_base_target_speed - speed_correction;
        int32_t target_speed_right = g_base_target_speed + speed_correction;

        // 3. 内环(速度环)PID计算
        int32_t actual_speed_left = Encoder_Get_Speed(ENCODER_CH1);
        int32_t actual_speed_right = Encoder_Get_Speed(ENCODER_CH2);
        
        PID_Set_Setpoint(&left_motor_pid, target_speed_left);
        int32_t pwm_left = PID_Calculate(&left_motor_pid, actual_speed_left);

        PID_Set_Setpoint(&right_motor_pid, target_speed_right);
        int32_t pwm_right = PID_Calculate(&right_motor_pid, actual_speed_right);
        
        // 4. 执行电机控制
        Motor_Control(pwm_left, pwm_right);
    }
    else // 如果不是在执行任务，确保电机停止
    {
        Motor_Control(0, 0);
    }
}

int Motion_Set_PID(char motor, char term, float value)
{
    PID_Controller_t* pid_ptr = NULL;

    // 1. 根据 'motor' 参数选择PID控制器
    if (motor == 'L' || motor == 'l') {
        pid_ptr = &left_motor_pid;
    } else if (motor == 'R' || motor == 'r') {
        pid_ptr = &right_motor_pid;
    } else if (motor == 'A' || motor == 'a') {
        pid_ptr = &pid_angle;
    } else if (motor == 'T' || motor == 't') { // 新增：选择循迹环
        pid_ptr = &pid_track;
    } else {
        return -1; // 无效的电机选择
    }

    // 2. 根据 'term' 参数修改对应的PID增益
    if (term == 'P' || term == 'p') {
        pid_ptr->Kp = value;
    } else if (term == 'I' || term == 'i') {
        pid_ptr->Ki = value;
    } else if (term == 'D' || term == 'd') {
        pid_ptr->Kd = value;
    } else {
        return -1; // 无效的PID项选择
    }

    PID_Reset(pid_ptr); // 修改参数后重置PID状态，以避免积分累积问题
    return 0;
}

void Motion_Get_PID(void (*send_callback)(const char* str))
{
    char buffer[128];

    if (send_callback == NULL) {
        return;
    }

    // 格式化并发送左轮PID参数
    snprintf(buffer, sizeof(buffer), "Left PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
             left_motor_pid.Kp, left_motor_pid.Ki, left_motor_pid.Kd);
    send_callback(buffer);

    // 格式化并发送右轮PID参数
    snprintf(buffer, sizeof(buffer), "Right PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
             right_motor_pid.Kp, right_motor_pid.Ki, right_motor_pid.Kd);
    send_callback(buffer);

    // 格式化并发送角度环PID参数
    snprintf(buffer, sizeof(buffer), "Angle PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
             pid_angle.Kp, pid_angle.Ki, pid_angle.Kd);
    send_callback(buffer);
    
    // 新增：格式化并发送循迹环PID参数
    snprintf(buffer, sizeof(buffer), "Track PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
             pid_track.Kp, pid_track.Ki, pid_track.Kd);
    send_callback(buffer);
}

// --- 自动任务控制 ---

/**
 * @brief 启动任务1: 直行一段距离
 */
void Task1_Start(void) {
    g_current_task_id = 1;
    g_active_task_context = 1; // 虽然任务1简单，但为了统一也设置上下文
    g_current_task_state = TASK1_GO_STRAIGHT;
    Odometry_Reset_Pose(); // 安全地重置姿态
    Motion_Go_Straight_Start(60, 6500);
}


void Task2_Start(void) {
    g_current_task_id = 2;
    g_active_task_context = 2;
    g_current_task_state = TASK2_GO_STRAIGHT_1;
    Odometry_Reset_Pose(); // 安全地重置姿态
    Motion_Go_Straight_Start(60, 6500);
}

void Task3_Start(void) {
    g_current_task_id = 3;
    g_active_task_context = 3;
    g_current_task_state = TASK3_TURN_CW_50; 
    Odometry_Reset_Pose(); // 安全地重置姿态
    Motion_Turn_Degrees_Start(-55.0f);
}

void Task4_Start(void) {
    g_current_task_id = 4;
    g_active_task_context = 4;
    task4_loop_counter = 1; // 开始第一次循环
    g_current_task_state = TASK3_TURN_CW_50;
    Odometry_Reset_Pose(); // 安全地重置姿态
    Motion_Turn_Degrees_Start(-55.0f);
}

/**
 * @brief 停止所有任务并重置状态
 */
void Task_Stop(void) {
    Motion_Stop();
    Alert_AllOff(); // << 立即关闭所有提示
    g_current_task_id = 0;
    g_current_task_state = TASK_STATE_IDLE;
    g_active_task_context = 0;
    task4_loop_counter = 0;
}

/**
 * @brief 获取当前任务的总体状态
 * @return TaskStatus_t 任务状态 (IDLE, RUNNING, COMPLETE)
 */
TaskStatus_t Task_Get_Status(void) {
    if (g_current_task_id == 0) {
        return TASK_STATUS_IDLE;
    }
    if (g_current_task_state == TASK_STATE_DONE) {
        return TASK_STATUS_COMPLETE;
    }
    return TASK_STATUS_RUNNING;
}

void Task_Update(void) {
    switch(g_current_task_state) {

        // =================================================
        //                 TASK 1 LOGIC
        // =================================================
        case TASK1_GO_STRAIGHT:
            // 任务1: 执行一个简单的直行任务
            // 条件: 如果Motion_Go_Straight_Start()完成(通过判断运动状态变为空闲)
            // 动作: 标记整个任务为完成状态
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                Alert_Trigger(); // << 任务完成提示
                g_current_task_state = TASK_STATE_DONE;
            }
            break;

        // =================================================
        //                 TASK 2 LOGIC
        // =================================================
        case TASK2_GO_STRAIGHT_1:
            // 任务2: 第一步，直行一段距离
            // 条件: 直行完成
            // 动作: 切换到循迹模式
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                Alert_Trigger(); 
                Motion_Track_Start(40);
                g_current_task_state = TASK2_TRACK_1;
            }
            break;

        case TASK2_TRACK_1:
            // 任务2: 第二步，执行循迹
            // 条件: 循迹因为丢线而结束
            // 动作: 转向到离0度或180度最近的方向，准备返回
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                Alert_Trigger(); 
                float yaw_360 = g_yaw < 0.0f ? g_yaw + 360.0f : g_yaw;
                float target = fabsf(yaw_360 - 0.0f) < fabsf(yaw_360 - 180.0f) ? 0.0f : 183.0f;
                Motion_Turn_To_Angle_Start(target);
                g_current_task_state = TASK2_GO_STRAIGHT_2;
            }
            break;

        case TASK2_GO_STRAIGHT_2:
            // 任务2: 第三步，转向完成
            // 条件: 转向完成
            // 动作: 启动第二次直行
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                Motion_Go_Straight_Start(60, 6600);
                g_current_task_state = TASK2_GO_STRAIGHT_2_WAIT;
            }
            break;

        case TASK2_GO_STRAIGHT_2_WAIT:
            // 任务2: 第四步，等待第二次直行完成
            // 条件: 第二次直行完成
            // 动作: 开始第二次循迹
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                Motion_Track_Start(40);
                g_current_task_state = TASK2_TRACK_2;
                Alert_Trigger();
            }
            break;

        case TASK2_TRACK_2:
            // 任务2: 第五步，第二次循迹
            // 条件: 循迹结束
            // 动作: 再次转向到0或180度方向，摆正车头
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                float yaw_360 = g_yaw < 0.0f ? g_yaw + 360.0f : g_yaw;
                float target = fabsf(yaw_360 - 0.0f) < fabsf(yaw_360 - 180.0f) ? 0.0f : 180.0f;
                Motion_Turn_To_Angle_Start(target);
                g_current_task_state = TASK2_TURN_TO_ZERO;
                Alert_Trigger(); 
            }
            break;

        case TASK2_TURN_TO_ZERO:
            // 任务2: 第六步，最后一次转向
            // 条件: 转向完成
            // 动作: 标记任务2完成
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                g_current_task_state = TASK_STATE_DONE;
            }
            break;


        // =================================================
        //                 TASK 3 & 4 LOGIC
        // =================================================
        case TASK3_TURN_CW_50:
            // 任务3/4: 第一步，顺时针旋转，为前往目标点做准备
            // 条件: 旋转完成
            // 动作: 开始直行，通过Y坐标判断停止
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                g_current_task_state = TASK3_GO_TO_Y_NEG80;
                Motion_Go_Straight_Start(60, 999999); 
            }
            break;

        case TASK3_GO_TO_Y_NEG80:
            // 任务3/4: 第二步，沿对角线直行
            // 条件: 到达目标Y坐标 (-80cm)
            // 动作: 停止，然后转向0度（车头朝向X轴正方向）
            {
                Pose_t current_pose;
                Odometry_Get_Pose(&current_pose);
                if (current_pose.y <= -70.0f) {
                    Motion_Stop();
                    g_current_task_state = TASK3_TURN_TO_0;
                    Motion_Turn_To_Angle_Start(0.0f);
                }
            }
            break;

        case TASK3_TURN_TO_0:
            // 任务3/4: 第三步，转向0度
            // 条件: 转向完成
            // 动作: 开始直行，通过X坐标判断停止
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                g_current_task_state = TASK3_GO_TO_X_100;
                Motion_Go_Straight_Start(60, 999999);
            }
            break;

        case TASK3_GO_TO_X_100:
            // 任务3/4: 第四步，沿X轴直行
            // 条件: 到达目标X坐标 (100cm)
            // 动作: 停止，然后开始第一次循迹
            {
                Pose_t current_pose;
                Odometry_Get_Pose(&current_pose);
                if (current_pose.x >= 95.0f) {
                    Motion_Stop();
                    g_current_task_state = TASK3_TRACK_1;
                    Motion_Track_Start(40);
                    Alert_Trigger(); 
                }
            }
            break;

        case TASK3_TRACK_1:
            // 任务3/4: 第五步，第一次循迹
            // 条件: 循迹结束
            // 动作: 逆时针旋转，为返回做准备
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                g_current_task_state = TASK3_TURN_CCW_50;
                Motion_Turn_Degrees_Start(75.0f);
                Alert_Trigger(); 
            }
            break;

        case TASK3_TURN_CCW_50:
            // 任务3/4: 第六步，逆时针旋转
            // 条件: 旋转完成
            // 动作: 开始直行，再次通过Y坐标判断
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                g_current_task_state = TASK3_GO_TO_Y_NEG80_2;
                Motion_Go_Straight_Start(60, 999999);
            }
            break;

        case TASK3_GO_TO_Y_NEG80_2:
            // 任务3/4: 第七步，返回时的对角线直行
            // 条件: 再次到达目标Y坐标 (-80cm)
            // 动作: 停止，然后转向180度（车头朝向X轴负方向）
            {
                Pose_t current_pose;
                Odometry_Get_Pose(&current_pose);
                float base_y = -70.0f;         // 第一圈目标Y
                float step = 2.0f;            // 每圈递增10cm
                float target_y = base_y + (task4_loop_counter - 1) * step;

                if (current_pose.y <= target_y) {
                Motion_Stop();
                g_current_task_state = TASK3_TURN_TO_180;
                Motion_Turn_To_Angle_Start(180.0f);
                }
            }
            break;

        case TASK3_TURN_TO_180:
            // 任务3/4: 第八步，转向180度
            // 条件: 转向完成
            // 动作: 开始直行，通过X坐标判断停止
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                g_current_task_state = TASK3_GO_TO_X_0;
                Motion_Go_Straight_Start(60, 999999);
            }
            break;

        case TASK3_GO_TO_X_0:
            // 任务3/4: 第九步，沿X轴负方向直行
            // 条件: 返回到X坐标原点 (0cm)
            // 动作: 停止，然后开始第二次循迹
            {
                Pose_t current_pose;
                Odometry_Get_Pose(&current_pose);
                if (current_pose.x <=15.0f) {
                    Motion_Stop();
                    g_current_task_state = TASK3_TRACK_2;
                    Motion_Track_Start(40);
                    Alert_Trigger(); 
                }
            }
            break;

        case TASK3_TRACK_2:
            // 任务3/4: 第十步，第二次循迹
            // 条件: 循迹结束
            // 动作: 判断是任务3还是任务4。
            //      如果是任务4且循环未满，则重置里程计并开始下一次循环。
            //      否则，标记整个任务完成。
            if (Motion_Get_State() == MOTION_STATE_IDLE) {
                if (g_active_task_context == 4 && task4_loop_counter < 4) {
                    task4_loop_counter++;
                    g_current_task_state = TASK3_TURN_CW_50;
                    Odometry_Reset_Pose(); 
                    Motion_Turn_To_Angle_Start(310.0f);
                    Alert_Trigger(); 
                } else {
                    g_current_task_state = TASK_STATE_DONE;
                    Alert_Trigger(); 
                }
            }
            break;


        // =================================================
        //                 COMMON STATES
        // =================================================
        case TASK_STATE_DONE:
            // 任务完成状态: 这是一个终端状态，等待UI或下一个任务启动指令来重置。
            // 它本身不执行任何操作。
            break;
        case TASK_STATE_IDLE:
        default:
            // 空闲、默认或未知状态: 确保系统处于安全状态。
            // 动作: 立即停止所有运动并重置所有任务变量。
            Task_Stop();
            break;
    }
}
