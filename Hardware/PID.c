#include "pid.h"
#include <stdlib.h> // For abs()

/**
 * @brief 初始化一个PID控制器实例
 * @param pid 指向PID控制器结构体的指针
 * @param kp 比例(Proportional)增益
 * @param ki 积分(Integral)增益
 * @param kd 微分(Derivative)增益
 * @param out_min 控制器输出的最小值
 * @param out_max 控制器输出的最大值
 * @param integral_limit 积分项最大值 (用于抗饱和)
 */
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, int32_t out_min, int32_t out_max, float integral_limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = 0;
    pid->integral = 0.0f;
    pid->last_error = 0;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_limit = integral_limit;
}

/**
 * @brief 计算PID控制器的输出值
 * @param pid 指向PID控制器结构体的指针
 * @param current_value 当前系统的测量值
 * @return 计算得到的控制输出值
 */
int32_t PID_Calculate(PID_Controller_t* pid, int32_t current_value)
{
    int32_t error;
    float p_out, i_out, d_out;
    float output;

    // 1. 计算当前误差
    error = pid->setpoint - current_value;

    // 2. 计算比例项
    p_out = pid->Kp * error;

    // 3. 计算积分项（始终累加，带积分限幅）
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    i_out = pid->Ki * pid->integral;

    // 4. 计算微分项（对误差做差分）
    d_out = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // 5. 计算总输出
    output = p_out + i_out + d_out;

    // 6. 对输出值进行限幅
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return (int32_t)output;
}

/**
 * @brief 重置PID控制器的内部状态
 * @param pid 指向PID控制器结构体的指针
 */
void PID_Reset(PID_Controller_t* pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0;
}

/**
 * @brief 动态修改PID控制器的目标值
 * @param pid 指向PID控制器结构体的指针
 * @param setpoint 新的目标值
 */
void PID_Set_Setpoint(PID_Controller_t* pid, int32_t setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Set_Setpoint_Float(PID_Controller_t* pid, float setpoint)
{
    pid->setpoint_float = setpoint;
}

int32_t PID_Calculate_Float(PID_Controller_t* pid, float current_value)
{
    float error = pid->setpoint_float - current_value;

    // Proportional term
    float p_term = pid->Kp * error;

    // Integral term
    pid->integral += error;
    // Integral windup guard
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->Ki * pid->integral;

    // Derivative term
    float d_term = pid->Kd * (error - pid->last_error_float);
    pid->last_error_float = error;

    // Total output
    float output = p_term + i_term + d_term;

    // Output saturation
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return (int32_t)output;
}
