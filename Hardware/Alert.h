#ifndef __ALERT_H
#define __ALERT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化声光提示模块。
 * @note   将内部状态机置于初始状态。
 */
void Alert_Init(void);

/**
 * @brief  【非阻塞】触发一次标准的声光提示。
 * @note   调用此函数会启动一次"蜂鸣+闪灯"的提示，但它会立即返回，
 *         不会产生阻塞。实际的关闭操作由 Alert_Update() 完成。
 */
void Alert_Trigger(void);

/**
 * @brief  【核心】更新声光提示的状态机。
 * @note   此函数必须在主循环或定时器中断中被周期性地调用，
 *         以处理非阻塞的延时和状态切换。
 */
void Alert_Update(void);

/**
 * @brief  立即关闭所有声光提示并重置状态。
 */
void Alert_AllOff(void);


#ifdef __cplusplus
}
#endif

#endif /* __ALERT_H */ 