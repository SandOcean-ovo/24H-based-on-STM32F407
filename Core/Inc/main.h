/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile float g_pitch, g_roll, g_yaw;
extern volatile float g_ax_g, g_ay_g, g_az_g;
extern volatile float g_gx_dps, g_gy_dps, g_gz_dps;
extern volatile int32_t g_display_speed_left;
extern volatile int32_t g_display_speed_right;
extern volatile int32_t g_display_counts_left;
extern volatile int32_t g_display_counts_right;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A_Speed_Pin GPIO_PIN_5
#define A_Speed_GPIO_Port GPIOE
#define B_Speed_Pin GPIO_PIN_6
#define B_Speed_GPIO_Port GPIOE
#define B_Backward_Pin GPIO_PIN_10
#define B_Backward_GPIO_Port GPIOF
#define B_Forward_Pin GPIO_PIN_0
#define B_Forward_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_2
#define BEEP_GPIO_Port GPIOA
#define Serial_IN_Pin GPIO_PIN_3
#define Serial_IN_GPIO_Port GPIOA
#define EA1_Pin GPIO_PIN_6
#define EA1_GPIO_Port GPIOA
#define EB1_Pin GPIO_PIN_7
#define EB1_GPIO_Port GPIOA
#define KEY_SELECT_Pin GPIO_PIN_5
#define KEY_SELECT_GPIO_Port GPIOC
#define Serial_OUT_Pin GPIO_PIN_8
#define Serial_OUT_GPIO_Port GPIOE
#define Sensor_1_Pin GPIO_PIN_9
#define Sensor_1_GPIO_Port GPIOE
#define Sensor_2_Pin GPIO_PIN_11
#define Sensor_2_GPIO_Port GPIOE
#define Sensor_3_Pin GPIO_PIN_13
#define Sensor_3_GPIO_Port GPIOE
#define Sensor_4_Pin GPIO_PIN_15
#define Sensor_4_GPIO_Port GPIOE
#define Sensor_6_Pin GPIO_PIN_14
#define Sensor_6_GPIO_Port GPIOD
#define KEY_OK_Pin GPIO_PIN_7
#define KEY_OK_GPIO_Port GPIOG
#define EA2_Pin GPIO_PIN_6
#define EA2_GPIO_Port GPIOC
#define EB2_Pin GPIO_PIN_7
#define EB2_GPIO_Port GPIOC
#define Sensor_5_Pin GPIO_PIN_0
#define Sensor_5_GPIO_Port GPIOD
#define Sensor_7_Pin GPIO_PIN_5
#define Sensor_7_GPIO_Port GPIOD
#define Sensor_8_Pin GPIO_PIN_10
#define Sensor_8_GPIO_Port GPIOG
#define A_Forward_Pin GPIO_PIN_11
#define A_Forward_GPIO_Port GPIOG
#define A_Backward_Pin GPIO_PIN_14
#define A_Backward_GPIO_Port GPIOG
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
