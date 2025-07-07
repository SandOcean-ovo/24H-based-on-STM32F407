#include "KalmanFilter.h"

/**
 * @brief  初始化卡尔曼滤波器
 * @param  kalman: 卡尔曼滤波器结构体指针
 * @param  Q_angle: 角度过程噪声协方差
 * @param  Q_bias: 角速度偏置过程噪声协方差
 * @param  R_measure: 测量噪声协方差
 * @retval None
 */
void KalmanFilter_Init(KalmanFilter_t *kalman, float Q_angle, float Q_bias, float R_measure)
{
    kalman->Q_angle = Q_angle;
    kalman->Q_bias = Q_bias;
    kalman->R_measure = R_measure;
    
    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    
    // 初始化误差协方差矩阵
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

/**
 * @brief  卡尔曼滤波器更新
 * @param  kalman: 卡尔曼滤波器结构体指针
 * @param  newAngle: 新的角度测量值
 * @param  newRate: 新的角速度测量值
 * @param  dt: 时间间隔(秒)
 * @retval 滤波后的角度值
 */
float KalmanFilter_Update(KalmanFilter_t *kalman, float newAngle, float newRate, float dt)
{
    // 步骤1: 预测
    // 根据角速度预测角度
    kalman->angle += dt * (newRate - kalman->bias);
    
    // 更新误差协方差矩阵
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;
    
    // 步骤2: 更新
    // 计算卡尔曼增益
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;
    
    // 计算残差
    float y = newAngle - kalman->angle;
    
    // 更新状态
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;
    
    // 更新误差协方差矩阵
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];
    
    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;
    
    return kalman->angle;
}

/**
 * @brief  初始化姿态角卡尔曼滤波器
 * @param  filter: 姿态角滤波器结构体指针
 * @retval None
 */
void AttitudeKalmanFilter_Init(AttitudeKalmanFilter_t *filter)
{
    // 初始化俯仰角滤波器
    // Q_angle: 角度过程噪声 (0.001)
    // Q_bias: 角速度偏置过程噪声 (0.003)
    // R_measure: 测量噪声 (0.03)
    KalmanFilter_Init(&filter->pitch, 0.001f, 0.003f, 0.03f);
    
    // 初始化横滚角滤波器
    KalmanFilter_Init(&filter->roll, 0.001f, 0.003f, 0.03f);
    
    // 初始化偏航角滤波器 (偏航角噪声较大，因为MPU6050没有磁力计)
    KalmanFilter_Init(&filter->yaw, 0.001f, 0.003f, 0.1f);
}

/**
 * @brief  姿态角卡尔曼滤波器更新
 * @param  filter: 姿态角滤波器结构体指针
 * @param  pitch: 俯仰角指针
 * @param  roll: 横滚角指针
 * @param  yaw: 偏航角指针
 * @param  gyro_x: X轴角速度指针
 * @param  gyro_y: Y轴角速度指针
 * @param  gyro_z: Z轴角速度指针
 * @param  dt: 时间间隔(秒)
 * @retval None
 */
void AttitudeKalmanFilter_Update(AttitudeKalmanFilter_t *filter, 
                                float *pitch, float *roll, float *yaw,
                                float *gyro_x, float *gyro_y, float *gyro_z,
                                float dt)
{
    // 将陀螺仪数据从度/秒转换为弧度/秒
    float gyro_x_rad = *gyro_x * 0.0174533f;  // deg/s to rad/s
    float gyro_y_rad = *gyro_y * 0.0174533f;
    float gyro_z_rad = *gyro_z * 0.0174533f;
    
    // 更新各轴姿态角
    *pitch = KalmanFilter_Update(&filter->pitch, *pitch, gyro_x_rad, dt);
    *roll = KalmanFilter_Update(&filter->roll, *roll, gyro_y_rad, dt);
    *yaw = KalmanFilter_Update(&filter->yaw, *yaw, gyro_z_rad, dt);
} 
