#include "MotorDriver.h"
#include "tim.h"      // 包含定时器句柄，例如 htim9
#include "main.h"     // 包含GPIO引脚宏定义


// 左轮 (MOTOR_ID 0)
#define LEFT_MOTOR_FORWARD_PORT   A_Forward_GPIO_Port
#define LEFT_MOTOR_FORWARD_PIN    A_Forward_Pin
#define LEFT_MOTOR_BACKWARD_PORT  A_Backward_GPIO_Port
#define LEFT_MOTOR_BACKWARD_PIN   A_Backward_Pin

// 右轮 (MOTOR_ID 1)
#define RIGHT_MOTOR_FORWARD_PORT  B_Forward_GPIO_Port
#define RIGHT_MOTOR_FORWARD_PIN   B_Forward_Pin
#define RIGHT_MOTOR_BACKWARD_PORT B_Backward_GPIO_Port
#define RIGHT_MOTOR_BACKWARD_PIN  B_Backward_Pin

// PWM 相关定义
#define MOTOR_PWM_TIMER       &htim9  //  PWM定时器句柄
#define LEFT_MOTOR_PWM_CHANNEL  TIM_CHANNEL_1
#define RIGHT_MOTOR_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR_PWM_MAX_VALUE   1000    // PWM最大计数值, 必须与定时器的 ARR(Period) 值一致
#define MAX_SPEED 1000  // 最大速度定义



/**
 * @brief 设置单个电机的速度和方向（模块内部使用）
 * @param motor_id 0=左轮, 1=右轮
 * @param speed 速度值 [-MOTOR_PWM_MAX_VALUE, MOTOR_PWM_MAX_VALUE]
 */
static void set_single_motor_speed(uint8_t motor_id, int16_t speed);



/**
 * @brief 初始化电机驱动模块
 */
void Motor_Init(void)
{
    // 启动左右轮的PWM通道
    HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL);

    // 默认上电后电机停止
    Motor_Control(0, 0);
}

/**
 * @brief 控制左右两个电机的速度和方向
 * @param left_speed  左轮速度，范围[-1000, 1000]。
 * @param right_speed 右轮速度，范围[-1000, 1000]。
 */
void Motor_Control(int16_t left_speed, int16_t right_speed)
{
    set_single_motor_speed(0, left_speed);  // 0 代表左轮
    set_single_motor_speed(1, right_speed); // 1 代表右轮
}

/**
 * @brief 单独或同时对轮子进行紧急刹车
 * @param motor_id 0=左轮，1=右轮，2=同时刹车两个轮子
 */
void Motor_Brake(uint8_t motor_id)
{
    if (motor_id == 0) {
        // 左轮刹车
        HAL_GPIO_WritePin(LEFT_MOTOR_FORWARD_PORT, LEFT_MOTOR_FORWARD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_MOTOR_BACKWARD_PORT, LEFT_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, 0);
    } else if (motor_id == 1) {
        // 右轮刹车
        HAL_GPIO_WritePin(RIGHT_MOTOR_FORWARD_PORT, RIGHT_MOTOR_FORWARD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_BACKWARD_PORT, RIGHT_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, 0);
    } else if (motor_id == 2) {
        // 同时刹车两个轮子
        HAL_GPIO_WritePin(LEFT_MOTOR_FORWARD_PORT, LEFT_MOTOR_FORWARD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_MOTOR_BACKWARD_PORT, LEFT_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, 0);
        HAL_GPIO_WritePin(RIGHT_MOTOR_FORWARD_PORT, RIGHT_MOTOR_FORWARD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_BACKWARD_PORT, RIGHT_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, 0);
    }
}



/**
 * @brief 设置单个电机的速度和方向（模块内部使用）
 */
static void set_single_motor_speed(uint8_t motor_id, int16_t speed)
{
    // 步骤1: 对输入速度进行限幅
    if (speed > MOTOR_PWM_MAX_VALUE) {
        speed = MOTOR_PWM_MAX_VALUE;
    }
    if (speed < -MOTOR_PWM_MAX_VALUE) {
        speed = -MOTOR_PWM_MAX_VALUE;
    }

    // 步骤2: 根据速度的正负号，设置方向引脚
    if (speed > 0) { // 正转
        if (motor_id == 0) { // 左轮
            HAL_GPIO_WritePin(LEFT_MOTOR_FORWARD_PORT, LEFT_MOTOR_FORWARD_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LEFT_MOTOR_BACKWARD_PORT, LEFT_MOTOR_BACKWARD_PIN, GPIO_PIN_RESET);
        } else { // 右轮
            HAL_GPIO_WritePin(RIGHT_MOTOR_FORWARD_PORT, RIGHT_MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RIGHT_MOTOR_BACKWARD_PORT, RIGHT_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
        }
    } else if (speed < 0) { // 反转
        if (motor_id == 0) { // 左轮
            HAL_GPIO_WritePin(LEFT_MOTOR_FORWARD_PORT, LEFT_MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LEFT_MOTOR_BACKWARD_PORT, LEFT_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
        } else { // 右轮
            HAL_GPIO_WritePin(RIGHT_MOTOR_FORWARD_PORT, RIGHT_MOTOR_FORWARD_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RIGHT_MOTOR_BACKWARD_PORT, RIGHT_MOTOR_BACKWARD_PIN, GPIO_PIN_RESET);
        }
    } else { // 速度为0，惰性停止 (Coasting)
        if (motor_id == 0) { // 左轮
            HAL_GPIO_WritePin(LEFT_MOTOR_FORWARD_PORT, LEFT_MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LEFT_MOTOR_BACKWARD_PORT, LEFT_MOTOR_BACKWARD_PIN, GPIO_PIN_RESET);
        } else { // 右轮
            HAL_GPIO_WritePin(RIGHT_MOTOR_FORWARD_PORT, RIGHT_MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RIGHT_MOTOR_BACKWARD_PORT, RIGHT_MOTOR_BACKWARD_PIN, GPIO_PIN_RESET);
        }
    }

    // 步骤3: 设置PWM占空比（速度的绝对值）
    uint16_t pwm_val = (speed > 0) ? speed : -speed;
    if (motor_id == 0) { // 左轮
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, pwm_val);
    } else { // 右轮
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, pwm_val);
    }
}
