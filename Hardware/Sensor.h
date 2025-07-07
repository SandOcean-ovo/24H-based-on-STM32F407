#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "Encoder.h"

/**
 * @brief 计算当前循迹的误差值
 * @return int16_t 误差值。0表示居中，负数表示线在左边，正数表示线在右边。
 * 如果完全没读到线，会返回一个极值（例如 +/-4000）。
 */
int16_t Sensor_GetError(void);

/**
 * @brief 读取8个传感器的原始数字量
 * @return uint8_t 一个8位字节，每一位代表一个传感器的状态 (1:白色, 0:黑色)。
 * 例如 0b11100111 表示中间两个传感器读到黑线。
 */
uint8_t Sensor_ReadRaw(void);

/**
 * @param ch 编码器通道
 * @return float 行驶距离（cm）
 */
float Sensor_Get_Distance_cm(Encoder_Channel_t ch);

/**
 * @brief  【循迹专用】读取串行灰度传感器并计算循迹误差
 * @return int16_t 误差值，负数表示线在左边，正数表示线在右边，0表示居中
 */
int16_t Sensor_Get_Line_Error(void);

/**
 * @brief 【底层】读取8位串行灰度传感器原始数据
 * @return uint8_t 8位数据，每一位代表一个传感器
 */
uint8_t gw_gray_serial_read(void);

void Serial_Sensor_Init(void);


#endif // __SENSOR_H
