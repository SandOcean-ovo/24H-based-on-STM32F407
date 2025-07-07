#include "Odometry.h"
#include "Encoder.h"
#include "main.h"   // 包含main.h以获取全局变量g_yaw
#include <math.h>

// --- 模块私有变量 ---
static Pose_t g_current_pose;
static int32_t last_left_counts = 0;
static int32_t last_right_counts = 0;

// --- 模块私有常量 ---
static const float CM_PER_COUNT = 99.75f / 6824.25f; // 厘米/编码器计数的转换因子
#define PI 3.14159265359f

// --- Debug Variables ---
volatile int32_t g_debug_delta_l = 0;
volatile int32_t g_debug_delta_r = 0;

void Odometry_Init(void) {
    g_current_pose.x = 0.0f;
    g_current_pose.y = 0.0f;
    g_current_pose.theta = 0.0f;
    Encoder_Reset(ENCODER_CH1);
    Encoder_Reset(ENCODER_CH2);
    last_left_counts = 0;
    last_right_counts = 0;
}

/**
 * @brief 重置里程计的姿态（坐标和角度），但不影响内部计数值
 * @note 这是一个安全的操作，可以在任务开始时调用，以将当前位置设为新的原点。
 */
void Odometry_Reset_Pose(void)
{
    // 只重置逻辑上的姿态信息
    g_current_pose.x = 0.0f;
    g_current_pose.y = 0.0f;
    // theta可以根据需求选择是否重置，这里我们重置它
    // g_current_pose.theta = 0.0f; // g_yaw是外部输入的，重置内部theta意义不大，保持与g_yaw同步即可
}

void Odometry_Sync_With_Encoder_Reset(void)
{
    // 只重置用于计算下一次增量的基准值
    last_left_counts = 0;
    last_right_counts = 0;
}

void Odometry_Update(void) {
    // 1. 获取当前编码器计数值
    int32_t current_left = Encoder_Get_Counts(ENCODER_CH1);
    int32_t current_right = Encoder_Get_Counts(ENCODER_CH2);

    // 2. 计算自上次更新以来的增量
    int32_t delta_l = current_left - last_left_counts;
    int32_t delta_r = current_right - last_right_counts;

    // -- 填充Debug变量 --
    g_debug_delta_l = delta_l;
    g_debug_delta_r = delta_r;

    // 3. 计算小车中心行驶的平均距离 (counts)
    float delta_d_counts = (float)(delta_l + delta_r) / 2.0f;
    
    // 将距离从counts转换为cm
    float delta_d_cm = delta_d_counts * CM_PER_COUNT;

    // 4. 获取当前航向角 (使用全局更新的g_yaw) 并转换为弧度
    float theta_rad = g_yaw * (PI / 180.0f);

    // 5. 将行驶距离分解到X和Y轴，并累加
    g_current_pose.x += delta_d_cm * cosf(theta_rad);
    g_current_pose.y += delta_d_cm * sinf(theta_rad);
    g_current_pose.theta = g_yaw; // 直接更新为最新的角度

    // 6. 保存当前计数值，为下次计算做准备
    last_left_counts = current_left;
    last_right_counts = current_right;
}

void Odometry_Get_Pose(Pose_t* pose_out)
{
    if (pose_out != NULL)
    {
        // --- 临界区保护 ---
        // 为了防止在复制结构体时被Odometry_Update中断打断，
        // 导致读取到不一致的数据（撕裂读），我们在此处临时禁用中断。
        __disable_irq(); // 关中断

        *pose_out = g_current_pose; // 这个复制操作现在是安全的

        __enable_irq();  // 恢复中断
        // --- 临界区结束 ---
    }
} 