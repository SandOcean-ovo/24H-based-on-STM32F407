#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include <stdint.h>




// --- 运动状态机定义 ---
typedef enum {
    MOTION_STATE_IDLE,          // 空闲状态
    MOTION_STATE_GO_STRAIGHT,   // 正在执行直行任务
    MOTION_STATE_TURN,          // 正在执行（相对）转向任务
    MOTION_STATE_TURN_TO_ANGLE, // 正在执行转向到（绝对）角度的任务
    MOTION_STATE_TRACK          //正在执行循迹任务
} Motion_State_t;

typedef enum {
    TASK_STATE_IDLE,
    
    // --- Task 1 States ---
    TASK1_GO_STRAIGHT,

    // --- Task 2 States  ---
    TASK2_GO_STRAIGHT_1,
    TASK2_TRACK_1,
    TASK2_GO_STRAIGHT_2,
    TASK2_GO_STRAIGHT_2_WAIT,
    TASK2_TRACK_2,
    TASK2_TURN_TO_ZERO,

    // --- Task 3 States  ---
    TASK3_TURN_CW_50,           // 顺时针旋转50度
    TASK3_GO_TO_Y_NEG80,        // 直行直到y=-80
    TASK3_TURN_TO_0,            // 回正车头至0度
    TASK3_GO_TO_X_100,          // 直行直到x=100
    TASK3_TRACK_1,
    TASK3_TURN_CCW_50,          // 逆时针旋转50度
    TASK3_GO_TO_Y_NEG80_2,      // 直行直到y=-80 (第二次)
    TASK3_TURN_TO_180,          // 回正车头至180度
    TASK3_GO_TO_X_0,            // 直行直到x=0
    TASK3_TRACK_2,

    TASK_STATE_DONE
} TaskState_t;

// 新增：任务总体状态枚举
typedef enum {
    TASK_STATUS_IDLE,       // 任务空闲
    TASK_STATUS_RUNNING,    // 任务运行中
    TASK_STATUS_COMPLETE    // 任务已完成
} TaskStatus_t;

/* --- 外部全局变量声明 --- */
// 使用 'extern' 关键字告诉编译器，这些变量在其他文件(main.c)中定义。
// 'volatile' 关键字确保编译器不会错误地优化对这些变量的访问，
// 因为它们在中断和主循环中都被使用。
extern volatile float g_yaw;
extern volatile int32_t g_display_speed_left;
extern volatile int32_t g_display_speed_right;
extern volatile int32_t g_display_counts_left;
extern volatile int32_t g_display_counts_right;
extern volatile Motion_State_t g_motion_state;
extern TaskState_t g_current_task_state;
extern uint8_t g_current_task_id; // 向其他模块声明当前任务ID全局变量

void Motion_Init(void);

/**
 * @brief 获取当前小车的运动状态
 * @return Motion_State_t 当前状态
 */
Motion_State_t Motion_Get_State(void);

/**
 * @brief 运动控制更新函数，应在定时器中断中被周期性调用
 */
void Motion_Update_In_Interrupt(void);


// --- 运动控制命令 ---

/**
 * @brief [非阻塞式] 命令小车开始原地精确转向指定的角度
 * @note  此函数会立即返回。实际的运动控制由后台的定时器中断驱动。
 * @param angle_to_turn 要转向的角度。正数代表逆时针转(yaw增加)，负数代表顺时针转。
 */
void Motion_Turn_Degrees_Start(float angle_to_turn);

/**
 * @brief [非阻塞式] 命令小车开始直线行驶指定的脉冲数
 * @note  此函数会立即返回。实际的运动控制由后台的定时器中断驱动。
 * @param base_speed     基础目标速度 (单位: 脉冲数/控制周期)。正数前进，负数后退。
 * @param total_counts   要行驶的总编码器脉冲数。
 */
void Motion_Go_Straight_Start(int32_t base_speed, int32_t total_counts);

/**
 * @brief [非阻塞式] 命令小车开始循迹
 * @param base_speed 循迹时的基础速度
 */
void Motion_Track_Start(int32_t base_speed);

/**
 * @brief 停止当前所有运动
 */
void Motion_Stop(void);

/**
 * @brief 在线修改PID参数的接口
 * @param motor 'L'/'l'代表左轮, 'R'/'r'代表右轮, 'A'/'a'代表角度环(用于转向)
 * @param term  'P'/'p', 'I'/'i', 'D'/'d' 分别代表Kp, Ki, Kd
 * @param value 新的PID参数值
 * @retval 0 表示设置成功, -1 表示参数错误
 */
int Motion_Set_PID(char motor, char term, float value);

/**
 * @brief 获取当前PID参数，并通过回调函数发送
 * @param send_callback 一个函数指针，用于发送格式化后的字符串。
 *                      例如: void send_str(const char* str) { ... }
 */
void Motion_Get_PID(void (*send_callback)(const char* str));

/**
 * @brief [非阻塞式] 命令小车开始转向到（绝对）角度的任务
 * @param target_angle 目标角度
 */
void Motion_Turn_To_Angle_Start(float target_angle);

// 自动任务接口
void Task1_Start(void);
void Task2_Start(void);
void Task3_Start(void);
void Task4_Start(void);
void Task_Stop(void);
void Task_Update(void);
TaskStatus_t Task_Get_Status(void);

// 里程计相关的声明已被移至 Odometry.h

#endif
