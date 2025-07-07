/*
 * MPU6050.h
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define ERROR_MPU_INIT      -1
#define ERROR_SET_SENSOR    -2
#define ERROR_CONFIG_FIFO   -3
#define ERROR_SET_RATE      -4
#define ERROR_LOAD_MOTION_DRIVER    -5
#define ERROR_SET_ORIENTATION       -6
#define ERROR_ENABLE_FEATURE        -7
#define ERROR_SET_FIFO_RATE         -8
#define ERROR_SELF_TEST             -9
#define ERROR_DMP_STATE             -10

#define DEFAULT_MPU_HZ  50  // 默认采样频率50Hz 应该和定时器中断频率一致
#define Q30  1073741824.0f


int MPU6050_DMP_init(void);
int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw, short *acX, short *acY, short *acZ, short *gyroX, short *gyroY, short *gyroZ);
int MPU6050_Get_Accel_G(float *ax_g, float *ay_g, float *az_g);
int MPU6050_Get_All_Data(float *pitch, float *roll, float *yaw, 
                        float *ax_g, float *ay_g, float *az_g,
                        float *gx_dps, float *gy_dps, float *gz_dps);
int MPU6050_Get_All_Data_Kalman(float *pitch, float *roll, float *yaw, 
                                float *ax_g, float *ay_g, float *az_g, float dt);

// --- 新增：独立数据获取功能 ---
int MPU6050_Data_Update(void);
float MPU6050_Get_Pitch(void);
float MPU6050_Get_Roll(void);
float MPU6050_Get_Yaw(void);
float MPU6050_Get_Accel_X_G(void);
float MPU6050_Get_Accel_Y_G(void);
float MPU6050_Get_Accel_Z_G(void);
float MPU6050_Get_Gyro_X_Dps(void);
float MPU6050_Get_Gyro_Y_Dps(void);
float MPU6050_Get_Gyro_Z_Dps(void);

#endif /* INC_MPU6050_H_ */
