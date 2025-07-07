#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

#include <math.h>

// 卡尔曼滤波器结构体
typedef struct {
    float Q_angle;    // 过程噪声协方差 - 角度
    float Q_bias;     // 过程噪声协方差 - 角速度偏置
    float R_measure;  // 测量噪声协方差
    
    float angle;      // 角度
    float bias;       // 角速度偏置
    
    float P[2][2];    // 误差协方差矩阵
} KalmanFilter_t;

// 函数声明
void KalmanFilter_Init(KalmanFilter_t *kalman, float Q_angle, float Q_bias, float R_measure);
float KalmanFilter_Update(KalmanFilter_t *kalman, float newAngle, float newRate, float dt);

// 三轴姿态角卡尔曼滤波器
typedef struct {
    KalmanFilter_t pitch;  // 俯仰角滤波器
    KalmanFilter_t roll;   // 横滚角滤波器
    KalmanFilter_t yaw;    // 偏航角滤波器
} AttitudeKalmanFilter_t;

// 姿态角滤波函数声明
void AttitudeKalmanFilter_Init(AttitudeKalmanFilter_t *filter);
void AttitudeKalmanFilter_Update(AttitudeKalmanFilter_t *filter, 
                                float *pitch, float *roll, float *yaw,
                                float *gyro_x, float *gyro_y, float *gyro_z,
                                float dt);

#endif 


