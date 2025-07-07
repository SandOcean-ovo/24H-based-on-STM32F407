#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

// 编码器通道枚举
typedef enum {
    ENCODER_CH1 = 0, // 假设为左轮
    ENCODER_CH2 = 1  // 假设为右轮
} Encoder_Channel_t;

/**
 * @brief 初始化并启动所有编码器
 */
void Encoder_Init(void);

/**
 * @brief 重置指定通道的编码器所有计数值
 * @param ch 编码器通道 (ENCODER_CH1 或 ENCODER_CH2)
 */
void Encoder_Reset(Encoder_Channel_t ch);

/**
 * @brief 【核心】更新指定通道的编码器数据，应在定时中断中被唯一调用
 * @param ch 编码器通道
 */
void Encoder_Update(Encoder_Channel_t ch);

/**
 * @brief 获取编码器瞬时速度 (单位: 脉冲数/更新周期)
 * @param ch 编码器通道
 * @return int32_t 瞬时速度值 (已处理方向)
 */
int32_t Encoder_Get_Speed(Encoder_Channel_t ch);

/**
 * @brief 获取编码器累计总脉冲数
 * @param ch 编码器通道
 * @return int32_t 累计脉冲数 (已处理方向)
 */
int32_t Encoder_Get_Counts(Encoder_Channel_t ch);

/**
 * @brief 获取编码器行驶距离（单位：厘米）
 * @param ch 编码器通道
 * @return float 行驶距离（cm）
 */
float Encoder_Get_Distance_cm(Encoder_Channel_t ch);

#endif // __ENCODER_H
