#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <stdint.h>

/**
 * @brief  定义小车的位姿（坐标和方向）
 */
typedef struct {
    float x;     // X 坐标 (单位: 厘米 cm)
    float y;     // Y 坐标 (单位: 厘米 cm)
    float theta; // 航向角 (单位: 度)
} Pose_t;


/**
 * @brief  初始化里程计系统
 * @note   将坐标和航向角清零，并重置编码器计数
 */
void Odometry_Init(void);

/**
 * @brief  更新里程计状态
 * @note   该函数应在固定的定时器中断中被周期性调用
 */
void Odometry_Update(void);

/**
 * @brief  获取当前小车的位姿
 * @param  pose_out 一个指向 Pose_t 结构体的指针，用于接收结果
 * @retval 无
 */
void Odometry_Get_Pose(Pose_t* pose_out);

/**
 * @brief  【新增的软重置】同步里程计的基准计数值。
 * @note   这个函数【必须】在每次外部调用了 Encoder_Reset() 之后被立刻调用。
 * 它只重置用于计算增量的 last_counts，不会影响已累积的 {x, y, theta}。
 */
void Odometry_Sync_With_Encoder_Reset(void);

/**
 * @brief  【安全重置】重置里程计的姿态（坐标），不影响底层计数
 * @note   用于在任务开始时将当前位置定义为新的逻辑原点(0,0)
 */
void Odometry_Reset_Pose(void);

// --- Debug Variables ---
extern volatile int32_t g_debug_delta_l;
extern volatile int32_t g_debug_delta_r;

#endif // __ODOMETRY_H 