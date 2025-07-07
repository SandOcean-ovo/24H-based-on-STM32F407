#ifndef __MOTOR_DRIVER_H_
#define __MOTOR_DRIVER_H_

#include "main.h"

// 电机ID定义
#define MOTOR_A_ID 1
#define MOTOR_B_ID 2

void Motor_Init(void);
void Motor_Control(int16_t left_speed, int16_t right_speed);
void Motor_Brake(uint8_t motor_id);

#endif // __MOTOR_DRIVER_H_ 
