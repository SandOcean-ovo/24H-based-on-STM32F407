/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "OLED.h"
#include "OLED_Menu.h"
#include "MPU6050.h"
#include <stdio.h>
#include <string.h>
#include "MotorDriver.h"
#include "Encoder.h"
#include "PID.h"
#include "Sensor.h"
#include "Motion_control.h"
#include "Odometry.h"
#include "Alert.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// --- 动作控制相关变量 ---
volatile float g_pitch, g_roll, g_yaw;
volatile float g_ax_g, g_ay_g, g_az_g;
volatile float g_gx_dps, g_gy_dps, g_gz_dps;
volatile int32_t g_display_speed_left = 0;
volatile int32_t g_display_speed_right = 0;
volatile int32_t g_display_counts_left = 0;
volatile int32_t g_display_counts_right = 0;

int turn_index = 0;
int turn_total = 4;

// --- 串口通信接口变量 ---
#define RX_BUFFER_SIZE 64
uint8_t uart_rx_buffer[RX_BUFFER_SIZE]; // 接收缓冲区
uint8_t rx_data; // 单个字节接收变量
volatile uint16_t rx_index = 0;
volatile uint8_t command_ready = 0; // 指令接收完成标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 回调函数，用于将 Motion_Get_PID 格式化的字符串通过串口发送
void send_pid_str(const char* str)
{
    HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), 100);
}
 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();	
  Motor_Init();        
  Encoder_Init();
  Motion_Init();
	Serial_Sensor_Init();
  Alert_Init();
	int ret = 0;
	do {
		ret = MPU6050_DMP_init();
		char ret_buf[16];
		sprintf(ret_buf, "ret=%d\r\n", ret);
		HAL_UART_Transmit(&huart5, (uint8_t*)ret_buf, strlen(ret_buf), 100);
	}while(ret);

    // --- 在启动高频中断前，给MPU6050一点稳定时间 ---
    HAL_Delay(100);



  	// --- 启动10ms定时器中断，开始闭环控制 ---
	HAL_TIM_Base_Start_IT(&htim10);
	
	// --- 启动UART接收中断 ---
	HAL_UART_Receive_IT(&huart5, &rx_data, 1);
	
  // Motion_Go_Straight_Start(30, 6500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     Alert_Update();

     Menu_Handler();
    static uint32_t last_report_time = 0;
   if (HAL_GetTick() - last_report_time >= 100)
   {
       last_report_time = HAL_GetTick();
       char report_buf[128];
       Pose_t pose;
       Odometry_Get_Pose(&pose);
       // 发送当前坐标和yaw角
       snprintf(report_buf, sizeof(report_buf), "X:%.2f Y:%.2f Yaw:%.2f\r\n", pose.x, pose.y, g_yaw);
       HAL_UART_Transmit(&huart5, (uint8_t*)report_buf, strlen(report_buf), 100);
   }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  定时器周期溢出回调函数
  * @note   此函数在定时器中断中被调用，用于执行周期性任务，例如PID控制
  * @param  htim: 定时器句柄指针
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM10) {
		// 获取MPU6050的所有数据，包括Z轴角速度
		MPU6050_Get_All_Data(&g_pitch, &g_roll, &g_yaw, &g_ax_g, &g_ay_g, &g_az_g, &g_gx_dps, &g_gy_dps, &g_gz_dps);

    Motion_Update_In_Interrupt();

		
	}




}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确保是UART5触发的中断
    if(huart->Instance == UART5)
    {
        // 检查是否接收到指令结束符 (回车)
        if(rx_data == '\r' || rx_data == '\n')
        {
            if(rx_index > 0) // 确保不是空指令
            {
                uart_rx_buffer[rx_index] = '\0'; // 添加字符串结束符
                command_ready = 1; // 设置指令就绪标志
            }
        }
        else
        {
            // 将接收到的数据存入缓冲区
            if(rx_index < RX_BUFFER_SIZE - 1)
            {
                uart_rx_buffer[rx_index++] = rx_data;
            }
        }
        // 重新启动UART接收中断，准备接收下一个字节
        HAL_UART_Receive_IT(&huart5, &rx_data, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
