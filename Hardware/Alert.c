#include "Alert.h"
#include "main.h"

// 定义声光提示状态机的状态
typedef enum {
    ALERT_STATE_IDLE,    // 空闲状态
    ALERT_STATE_ACTIVE   // 正在提示（灯亮、蜂鸣器响）
} AlertState_t;

// 模块内部变量
static AlertState_t g_alert_state = ALERT_STATE_IDLE;
static uint32_t g_alert_start_time = 0;

// 定义提示持续时间 (毫秒)
#define ALERT_DURATION_MS 50

/**
 * @brief  初始化声光提示模块。
 */
void Alert_Init(void)
{
    // GPIO本身的初始化由CubeMX在 main.c 的 MX_GPIO_Init() 中完成。
    // 此函数确保在启动时关闭所有提示并重置状态。
    Alert_AllOff();
}

/**
 * @brief  【非阻塞】触发一次标准的声光提示。
 */
void Alert_Trigger(void)
{
    // 仅在空闲状态下才能触发新的提示，防止重入
    if (g_alert_state == ALERT_STATE_IDLE)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        
        g_alert_start_time = HAL_GetTick(); // 记录开始时间
        g_alert_state = ALERT_STATE_ACTIVE; // 切换到活动状态
    }
}

/**
 * @brief  【核心】更新声光提示的状态机。
 */
void Alert_Update(void)
{
    // 只有在活动状态下才需要检查时间
    if (g_alert_state == ALERT_STATE_ACTIVE)
    {
        // 检查是否已达到指定的持续时间
        if (HAL_GetTick() - g_alert_start_time >= ALERT_DURATION_MS)
        {
            // 时间到，关闭所有提示并返回空闲状态
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            g_alert_state = ALERT_STATE_IDLE;
        }
    }
}

/**
 * @brief  立即关闭所有声光提示并重置状态。
 */
void Alert_AllOff(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    g_alert_state = ALERT_STATE_IDLE;
} 