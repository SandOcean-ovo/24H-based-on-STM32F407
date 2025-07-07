/*
 MPU6050.c
 */

#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include <stdio.h> // For NULL
#include <stdint.h> // For uint8_t
#include "KalmanFilter.h"

/**
 * @brief 传感器安装方向矩阵
 * @note  该矩阵定义了MPU6050传感器的物理坐标系与载体（例如无人机、小车）坐标系之间的转换关系。
 * DMP（数字运动处理器）会利用这个矩阵来正确地解算姿态。
 * 这是一个3x3的旋转矩阵，每一行代表载体的一个轴（X, Y, Z），
 * 行中的三个值分别表示传感器的X, Y, Z轴如何映射到这个载体轴上。
 *
 * 默认值 {-1, 0, 0, 0,-1, 0, 0, 0, 1} 表示：
 * - 载体X轴 = 传感器 -X 轴
 * - 载体Y轴 = 传感器 -Y 轴
 * - 载体Z轴 = 传感器 +Z 轴
 * 如果你的安装方向不同，必须修改这个矩阵。例如，标准安装（X对X, Y对Y, Z对Z）应为：
 * {1, 0, 0, 0, 1, 0, 0, 0, 1};
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

// 静态变量，用于缓存最新的传感器数据，以供独立函数调用
static float mpu_pitch = 0.0f;
static float mpu_roll = 0.0f;
static float mpu_yaw = 0.0f;
static float mpu_ax_g = 0.0f;
static float mpu_ay_g = 0.0f;
static float mpu_az_g = 0.0f;
static float mpu_gyro_x_dps = 0.0f;
static float mpu_gyro_y_dps = 0.0f;
static float mpu_gyro_z_dps = 0.0f;

/**
 * @brief  将方向矩阵的一行转换为一个标量值。
 * @note   这是一个辅助函数，由InvenSense官方提供，用于将 inv_orientation_matrix_to_scalar 函数。
 * 它根据一行中哪个元素是+1或-1，来确定一个2位的编码值，代表一个轴的方向。
 * @param  row: 指向3x3矩阵中某一行的指针。
 * @retval 一个代表该行方向的2位标量。
 */
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // 错误情况
    return b;
}

/**
 * @brief  将一个3x3的方向矩阵转换为一个DMP可以理解的标量值。
 * @note   DMP不直接使用矩阵，而是使用一个特定的标量值来配置其内部的坐标转换。
 * 此函数将矩阵的三行分别进行编码，并组合成一个最终的配置标量。
 * @param  mtx: 指向3x3方向矩阵的指针。
 * @retval 一个代表完整方向的标量值。
 */
static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 (二进制) -> 0x48 身份矩阵
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/**
 * @brief  运行MPU6050的自检程序，并根据结果设置DMP的初始偏置。
 * @note   此函数会触发传感器进行内部自检。自检会测量在激励状态下和非激励状态下
 * 传感器输出的差异，从而得到出厂时硬件的固有偏置(bias)。
 * 为了让DMP的姿态解算从一开始就更准确，需要将这些偏置值写入DMP。
 * @retval 0 表示自检通过并且偏置设置成功，-1 表示失败。
 */
static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    // 调用底层驱动，执行自检。 gyro和accel数组将包含计算出的硬件偏置。
    result = mpu_run_self_test(gyro, accel);
    // result == 0x3 (二进制0011) 表示陀螺仪(bit 0)和加速度计(bit 1)都通过了自检。
    if (result == 0x3) {
        /* 自检通过。我们可以信任这些偏置数据，所以将它们写入DMP。 */
        float sens;
        unsigned short accel_sens;

        // 获取陀螺仪当前的灵敏度，并将原始偏置转换为DMP需要的格式。
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro); // 将陀螺仪偏置写入DMP

        // 获取加速度计当前的灵敏度，并将原始偏置转换为DMP需要的格式。
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel); // 将加速度计偏置写入DMP
    } else {
        return -1; // 自检失败
    }

    return 0;
}

/**
 * @brief  初始化MPU6050及其DMP功能。
 * @note   这是一个总初始化函数，它按照正确的顺序调用所有必要的底层函数来配置传感器和DMP。
 * 如果其中任何一步失败，函数会立即返回一个错误码。
 * @retval 0 表示初始化完全成功，其他值为对应的错误码。
 */
int MPU6050_DMP_init(void)
{
    int ret;
    struct int_param_s int_param;

    // 步骤1: 初始化MPU6050芯片，包括复位、唤醒等基本操作。
    ret = mpu_init(&int_param);
    if(ret != 0)
    {
        return ERROR_MPU_INIT;
    }
    // 步骤2: 使能加速度计和陀螺仪这两个传感器。
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_SET_SENSOR;
    }
    // 步骤3: 配置FIFO（先进先出缓冲区），让传感器数据能够进入其中。
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_CONFIG_FIFO;
    }
    // 步骤4: 设置传感器的采样率。
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_RATE;
    }
    // 步骤5: 加载DMP固件。这是最关键的一步，将InvenSense提供的固件程序写入MPU6050内部。
    ret = dmp_load_motion_driver_firmware();
    if(ret != 0)
    {
        return ERROR_LOAD_MOTION_DRIVER;
    }
    // 步骤6: 设置传感器的安装方向，将物理安装矩阵转换为DMP能识别的标量。
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(ret != 0)
    {
        return ERROR_SET_ORIENTATION;
    }
    // 步骤7: 使能DMP的特定功能。
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |      // 开启六轴低功耗四元数解算
                               DMP_FEATURE_TAP |          // 开启敲击检测
                               DMP_FEATURE_ANDROID_ORIENT | // 开启安卓屏幕方向检测
                               DMP_FEATURE_SEND_RAW_ACCEL | // 发送原始加速度数据到FIFO
                               DMP_FEATURE_SEND_CAL_GYRO |  // 发送校准后的陀螺仪数据到FIFO
                               DMP_FEATURE_GYRO_CAL);        // 使能DMP内部的陀螺仪自动校准
    if(ret != 0)
    {
        return ERROR_ENABLE_FEATURE;
    }
    // 步骤8: 设置DMP将数据放入FIFO的速率。
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_FIFO_RATE;
    }
    // 步骤9: 运行硬件自检，获取并设置初始偏置。
    ret = run_self_test();
    if(ret != 0)
    {
        return ERROR_SELF_TEST;
    }
    // 步骤10: 正式开启DMP，让它开始工作。
    ret = mpu_set_dmp_state(1);
    if(ret != 0)
    {
        return ERROR_DMP_STATE;
    }

    return 0;
}

/**
 * @brief  从DMP的FIFO中读取数据，并解算出姿态角（俯仰、横滚、偏航）。
 * @param  pitch: 指向用于存储俯仰角(Pitch)的float变量的指针。
 * @param  roll:  指向用于存储横滚角(Roll)的float变量的指针。
 * @param  yaw:   指向用于存储偏航角(Yaw)的float变量的指针。
 * @param  acX:   指向用于存储X轴加速度数据的short变量的指针。
 * @param  acY:   指向用于存储Y轴加速度数据的short变量的指针。
 * @param  acZ:   指向用于存储Z轴加速度数据的short变量的指针。
 * @param  gyroX: 指向用于存储X轴陀螺仪数据的short变量的指针。
 * @param  gyroY: 指向用于存储Y轴陀螺仪数据的short变量的指针。
 * @param  gyroZ: 指向用于存储Z轴陀螺仪数据的short变量的指针。
 * @retval 0 表示成功获取并解算数据，-1 表示从FIFO读取数据失败。
 */
int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw, short *acX, short *acY, short *acZ, short *gyroX, short *gyroY, short *gyroZ)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];      // 陀螺仪原始数据
    short accel[3];     // 加速度计原始数据
    long quat[4];       // DMP输出的四元数原始值
    unsigned long timestamp; // 时间戳
    short sensors;      // 传感器掩码，指示包中包含哪些数据
    unsigned char more; // FIFO中剩余数据包数量

    // 从FIFO中读取一个DMP数据包
    if(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1; // 读取失败
    }

    // 检查数据包中是否包含有效的陀螺仪数据
    if (sensors & INV_XYZ_GYRO)
    {
        // 将读取到的gyro[]数组的值赋给通过指针传入的变量
        *gyroX = gyro[0];
        *gyroY = gyro[1];
        *gyroZ = gyro[2];
    }

		// 检查数据包中是否包含有效的加速度计数据
    if (sensors & INV_XYZ_ACCEL)
    {
        // 将读取到的accel[]数组的值赋给通过指针传入的变量
        *acX = accel[0];
        *acY = accel[1];
        *acZ = accel[2];
    }
		
    // 检查数据包中是否包含有效的四元数数据
    if(sensors & INV_WXYZ_QUAT)
    {
        // DMP输出的四元数是Q30格式的定点数，需要除以 2^30 (1073741824.0f) 转换为浮点数。
        // q0是实部, q1,q2,q3是虚部。
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        // 使用标准公式将四元数转换为欧拉角(俯仰、横滚、偏航)。
        // 结果乘以 57.3 (约等于 180/PI) 是为了将单位从弧度(rad)转换为度(°)。
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // 俯仰角
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // 横滚角
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // 偏航角
    }

    return 0; // 成功
}

/**
 * @brief  获取归一化后的加速度值（单位：g）
 * @param  ax_g: 指向用于存储X轴加速度(g)的float变量的指针
 * @param  ay_g: 指向用于存储Y轴加速度(g)的float变量的指针
 * @param  az_g: 指向用于存储Z轴加速度(g)的float变量的指针
 * @retval 0 表示成功获取数据，-1 表示获取数据失败
 * @note   此函数会自动将MPU6050的原始加速度数据转换为g单位
 */
int MPU6050_Get_Accel_G(float *ax_g, float *ay_g, float *az_g)
{
    short acc_x, acc_y, acc_z;
    float pitch, roll, yaw;
    
    const float ACCEL_SENSITIVITY = 16384.0f;  // MPU6050在±2g量程下的灵敏度
    
    // 获取原始数据
    if(MPU6050_DMP_Get_Data(&pitch, &roll, &yaw, &acc_x, &acc_y, &acc_z, NULL, NULL, NULL) != 0)
    {
        return -1; // 获取数据失败
    }
    
    // 将原始数据转换为g单位
    *ax_g = (float)acc_x / ACCEL_SENSITIVITY;
    *ay_g = (float)acc_y / ACCEL_SENSITIVITY;
    *az_g = (float)acc_z / ACCEL_SENSITIVITY;
    
    return 0; // 成功
}

/**
 * @brief  获取姿态角和g单位的加速度值（完整数据包）
 * @param  pitch: 指向用于存储俯仰角(Pitch)的float变量的指针
 * @param  roll:  指向用于存储横滚角(Roll)的float变量的指针
 * @param  yaw:   指向用于存储偏航角(Yaw)的float变量的指针
 * @param  ax_g:  指向用于存储X轴加速度(g)的float变量的指针
 * @param  ay_g:  指向用于存储Y轴加速度(g)的float变量的指针
 * @param  az_g:  指向用于存储Z轴加速度(g)的float变量的指针
 * @retval 0 表示成功获取数据，-1 表示获取数据失败
 * @note   此函数提供完整的封装，一次性获取所有需要的数据
 */
/*
int MPU6050_Get_All_Data(float *pitch, float *roll, float *yaw, 
                        float *ax_g, float *ay_g, float *az_g)
{
    short acc_x, acc_y, acc_z;
    const float ACCEL_SENSITIVITY = 16384.0f;  // MPU6050在±2g量程下的灵敏度
    
    // 获取原始数据
    if(MPU6050_DMP_Get_Data(pitch, roll, yaw, &acc_x, &acc_y, &acc_z, NULL, NULL, NULL) != 0)
    {
        return -1; // 获取数据失败
    }
    
    // 将原始加速度数据转换为g单位
    *ax_g = (float)acc_x / ACCEL_SENSITIVITY;
    *ay_g = (float)acc_y / ACCEL_SENSITIVITY;
    *az_g = (float)acc_z / ACCEL_SENSITIVITY;
    
    return 0; // 成功
}
*/

/**
 * @brief  获取所有9个传感器数据（姿态角、加速度、角速度）
 * @param  pitch: 指向用于存储俯仰角(Pitch)的float变量的指针，单位：度
 * @param  roll:  指向用于存储横滚角(Roll)的float变量的指针，单位：度
 * @param  yaw:   指向用于存储偏航角(Yaw)的float变量的指针，单位：度
 * @param  ax_g:  指向用于存储X轴加速度的float变量的指针，单位：g
 * @param  ay_g:  指向用于存储Y轴加速度的float变量的指针，单位：g
 * @param  az_g:  指向用于存储Z轴加速度的float变量的指针，单位：g
 * @param  gx_dps: 指向用于存储X轴角速度的float变量的指针，单位：度/秒
 * @param  gy_dps: 指向用于存储Y轴角速度的float变量的指针，单位：度/秒
 * @param  gz_dps: 指向用于存储Z轴角速度的float变量的指针，单位：度/秒
 * @retval 0 表示成功获取数据，-1 表示获取数据失败
 * @note   此函数一次性获取所有9个传感器数据，包括3个姿态角、3个加速度、3个角速度
 */
int MPU6050_Get_All_Data(float *pitch, float *roll, float *yaw, 
                        float *ax_g, float *ay_g, float *az_g,
                        float *gx_dps, float *gy_dps, float *gz_dps)
{
    short acc_x, acc_y, acc_z;
    short gyro_x, gyro_y, gyro_z;
    float gyro_sens;
    const float ACCEL_SENSITIVITY = 16384.0f;  // MPU6050在±2g量程下的灵敏度
    
    // 获取原始数据（包括角速度数据）
    if(MPU6050_DMP_Get_Data(pitch, roll, yaw, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z) != 0)
    {
        return -1; // 获取数据失败
    }
    
    // 将原始加速度数据转换为g单位
    *ax_g = (float)acc_x / ACCEL_SENSITIVITY;
    *ay_g = (float)acc_y / ACCEL_SENSITIVITY;
    *az_g = (float)acc_z / ACCEL_SENSITIVITY;
    
    // 获取陀螺仪灵敏度并转换角速度数据为度/秒
    mpu_get_gyro_sens(&gyro_sens);
    *gx_dps = (float)gyro_x / gyro_sens;
    *gy_dps = (float)gyro_y / gyro_sens;
    *gz_dps = (float)gyro_z / gyro_sens;
    
    return 0; // 成功
}

/**
 * @brief  获取姿态角和g单位的加速度值（完整数据包，经过卡尔曼滤波）
 * @param  pitch: 指向用于存储滤波后俯仰角(Pitch)的float变量的指针
 * @param  roll:  指向用于存储滤波后横滚角(Roll)的float变量的指针
 * @param  yaw:   指向用于存储滤波后偏航角(Yaw)的float变量的指针
 * @param  ax_g:  指向用于存储X轴加速度(g)的float变量的指针
 * @param  ay_g:  指向用于存储Y轴加速度(g)的float变量的指针
 * @param  az_g:  指向用于存储Z轴加速度(g)的float变量的指针
 * @param  dt:    采样周期(秒)
 * @retval 0 表示成功获取数据，-1 表示获取数据失败
 * @note   需在外部静态保存filter实例，或在此文件内静态保存。
 */
int MPU6050_Get_All_Data_Kalman(float *pitch, float *roll, float *yaw, 
                                float *ax_g, float *ay_g, float *az_g, float dt)
{
    static AttitudeKalmanFilter_t kalman_filter;
    static uint8_t is_kalman_inited = 0;
    if (!is_kalman_inited) {
        AttitudeKalmanFilter_Init(&kalman_filter);
        is_kalman_inited = 1;
    }
    short acc_x, acc_y, acc_z;
    short gyro_x, gyro_y, gyro_z;
    float raw_pitch, raw_roll, raw_yaw;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    const float ACCEL_SENSITIVITY = 16384.0f;
    float gyro_sens;

    // 获取原始数据
    if(MPU6050_DMP_Get_Data(&raw_pitch, &raw_roll, &raw_yaw, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z) != 0)
    {
        return -1; // 获取数据失败
    }
    // 角速度转换为度/秒
    mpu_get_gyro_sens(&gyro_sens);
    gyro_x_dps = (float)gyro_x / gyro_sens;
    gyro_y_dps = (float)gyro_y / gyro_sens;
    gyro_z_dps = (float)gyro_z / gyro_sens;

    // 卡尔曼滤波
    AttitudeKalmanFilter_Update(&kalman_filter, &raw_pitch, &raw_roll, &raw_yaw,
                               &gyro_x_dps, &gyro_y_dps, &gyro_z_dps, dt);
    *pitch = raw_pitch;
    *roll = raw_roll;
    *yaw = raw_yaw;
    *ax_g = (float)acc_x / ACCEL_SENSITIVITY;
    *ay_g = (float)acc_y / ACCEL_SENSITIVITY;
    *az_g = (float)acc_z / ACCEL_SENSITIVITY;
    return 0;
}

// --- 新增：独立数据获取功能 ---

/**
 * @brief  从DMP更新所有传感器数据到内部静态变量中。
 * @note   推荐在主循环中周期性地调用此函数来刷新数据。
 *         调用后，即可使用 Get_Pitch/Roll/Yaw 等函数获取最新的值。
 * @retval 0 表示成功, -1 表示失败
 */
int MPU6050_Data_Update(void)
{
    short acc_x, acc_y, acc_z;
    short gyro_x, gyro_y, gyro_z;
    float pitch, roll, yaw;
    const float ACCEL_SENSITIVITY = 16384.0f;
    float gyro_sens;

    // 从DMP获取最新的原始数据
    if(MPU6050_DMP_Get_Data(&pitch, &roll, &yaw, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z) != 0)
    {
        return -1; // 获取失败
    }

    // 更新所有静态变量
    mpu_pitch = pitch;
    mpu_roll = roll;
    mpu_yaw = yaw;
    mpu_ax_g = (float)acc_x / ACCEL_SENSITIVITY;
    mpu_ay_g = (float)acc_y / ACCEL_SENSITIVITY;
    mpu_az_g = (float)acc_z / ACCEL_SENSITIVITY;
    
    // 更新角速度数据 (dps: degrees per second)
    mpu_get_gyro_sens(&gyro_sens);
    mpu_gyro_x_dps = (float)gyro_x / gyro_sens;
    mpu_gyro_y_dps = (float)gyro_y / gyro_sens;
    mpu_gyro_z_dps = (float)gyro_z / gyro_sens;

    return 0;
}

/** @brief  获取最新更新的俯仰角 (Pitch), 单位:度 */
float MPU6050_Get_Pitch(void) { return mpu_pitch; }

/** @brief  获取最新更新的横滚角 (Roll), 单位:度 */
float MPU6050_Get_Roll(void) { return mpu_roll; }

/** @brief  获取最新更新的偏航角 (Yaw), 单位:度 */
float MPU6050_Get_Yaw(void) { return mpu_yaw; }

/** @brief  获取最新更新的X轴加速度 (g) */
float MPU6050_Get_Accel_X_G(void) { return mpu_ax_g; }

/** @brief  获取最新更新的Y轴加速度 (g) */
float MPU6050_Get_Accel_Y_G(void) { return mpu_ay_g; }

/** @brief  获取最新更新的Z轴加速度 (g) */
float MPU6050_Get_Accel_Z_G(void) { return mpu_az_g; }

/** @brief  获取最新更新的X轴角速度 (dps:度/秒) */
float MPU6050_Get_Gyro_X_Dps(void) { return mpu_gyro_x_dps; }

/** @brief  获取最新更新的Y轴角速度 (dps:度/秒) */
float MPU6050_Get_Gyro_Y_Dps(void) { return mpu_gyro_y_dps; }

/** @brief  获取最新更新的Z轴角速度 (dps:度/秒) */
float MPU6050_Get_Gyro_Z_Dps(void) { return mpu_gyro_z_dps; }
