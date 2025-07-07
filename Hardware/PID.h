#ifndef __PID_H
#define __PID_H

#include <stdint.h>

// PID控制器结构体
typedef struct {
    float Kp;  // 比例 Proportional
    float Ki;  // 积分 Integral
    float Kd;  // 微分 Derivative

    int32_t setpoint; // 目标值
    float setpoint_float;

    float integral;     // 积分累加值
    int32_t last_error; // 上一次的误差
    float last_error_float;

    int32_t output_min; // 输出限幅
    int32_t output_max;

    float integral_limit; // 积分限幅
} PID_Controller_t;

/**
 * @brief 初始化PID控制器
 * @param pid 指向PID控制器结构体的指针
 * @param kp P参数
 * @param ki I参数
 * @param kd D参数
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 * @param integral_limit 积分项最大值 (用于抗饱和)
 */
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, int32_t out_min, int32_t out_max, float integral_limit);

/**
 * @brief 计算PID输出
 * @param pid 指向PID控制器结构体的指针
 * @param current_value 当前测量值
 * @return 计算得到的控制输出值
 */
int32_t PID_Calculate(PID_Controller_t* pid, int32_t current_value);

/**
 * @brief 重置PID控制器的内部状态
 * @param pid 指向PID控制器结构体的指针
 */
void PID_Reset(PID_Controller_t* pid);

/**
 * @brief 动态修改PID控制器的目标值
 * @param pid 指向PID控制器结构体的指针
 * @param setpoint 新的目标值
 */
void PID_Set_Setpoint(PID_Controller_t* pid, int32_t setpoint);

void PID_Set_Setpoint_Float(PID_Controller_t* pid, float setpoint);

int32_t PID_Calculate_Float(PID_Controller_t* pid, float current_value);

#endif // __PID_H
