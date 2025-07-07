#include "Encoder.h"
#include "tim.h" // tim.h中已声明htim3/htim8

// --- 模块私有（静态）变量 ---

// 用于计算增量的上一次硬件计数值
static int32_t encoder_last_count[2] = {0, 0}; 

// 用于存储计算结果的全局状态变量，volatile防止编译器过度优化
static volatile int32_t g_encoder_total_count[2] = {0, 0}; // 累计总路程
static volatile int32_t g_encoder_speed[2] = {0, 0};       // 最新计算出的瞬时速度

// !!! 这个值需要你自己标定 !!!
#define COUNTS_PER_CM 123.45f 

// --- 内部辅助函数 ---
static TIM_HandleTypeDef* Encoder_GetHandle(Encoder_Channel_t ch)
{
    if (ch == ENCODER_CH1) return &htim3;
    else if (ch == ENCODER_CH2) return &htim8;
    else return NULL;
}

// --- 公共接口函数实现 ---

/**
 * @brief 初始化并启动所有编码器
 */
void Encoder_Init(void)
{
    TIM_HandleTypeDef* htim;

    // 启动并重置通道1
    htim = Encoder_GetHandle(ENCODER_CH1);
    if (htim) {
        HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
        Encoder_Reset(ENCODER_CH1);
    }
    
    // 启动并重置通道2
    htim = Encoder_GetHandle(ENCODER_CH2);
    if (htim) {
        HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
        Encoder_Reset(ENCODER_CH2);
    }
}

/**
 * @brief 重置指定通道的编码器所有计数值
 */
void Encoder_Reset(Encoder_Channel_t ch)
{
    TIM_HandleTypeDef* htim = Encoder_GetHandle(ch);
    if (htim) {
        // 硬件计数器清零
        __HAL_TIM_SET_COUNTER(htim, 0); 
    }
    // 软件状态变量清零
    encoder_last_count[ch] = 0;
    g_encoder_total_count[ch] = 0;
    g_encoder_speed[ch] = 0;
}

/**
 * @brief 【核心】更新指定通道的编码器数据，在定时中断中被唯一调用
 */
void Encoder_Update(Encoder_Channel_t ch)
{
    TIM_HandleTypeDef* htim = Encoder_GetHandle(ch);
    if (!htim) return;

    // 1. 读取硬件计数器
    int32_t curr_count = (int32_t)__HAL_TIM_GET_COUNTER(htim);
    
    // 2. 计算增量（即速度）
    int32_t diff = curr_count - encoder_last_count[ch];
    encoder_last_count[ch] = curr_count; // 立刻更新last_count

    // 3. 处理硬件计数器溢出
    if (diff > 32767)       diff -= 65536;
    else if (diff < -32768) diff += 65536;

    // 4. 更新全局状态变量（速度和路程）
    g_encoder_speed[ch] = diff;
    g_encoder_total_count[ch] += diff;
}

/**
 * @brief 【纯读取】获取编码器瞬时速度
 */
int32_t Encoder_Get_Speed(Encoder_Channel_t ch)
{
    // 直接返回由Update函数算好的值
    int32_t speed = g_encoder_speed[ch];
    
    // 根据物理安装方向，对返回值取反
    if (ch == ENCODER_CH2) {
        return -speed;
    }
    return speed;
}

/**
 * @brief 【纯读取】获取编码器累计总脉冲数
 */
int32_t Encoder_Get_Counts(Encoder_Channel_t ch)
{
    // 直接返回由Update函数算好的值
    int32_t counts = g_encoder_total_count[ch];

    // 根据物理安装方向，对返回值取反
    if (ch == ENCODER_CH2) {
        return -counts;
    }
    return counts;
}

/**
 * @brief 【纯读取】获取编码器行驶距离（单位：厘米）
 */
float Encoder_Get_Distance_cm(Encoder_Channel_t ch)
{
    // 调用无副作用的Get_Counts函数
    return (float)Encoder_Get_Counts(ch) / COUNTS_PER_CM;
}
