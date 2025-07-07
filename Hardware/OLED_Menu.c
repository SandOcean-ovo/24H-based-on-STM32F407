#include "OLED_Menu.h"
#include "main.h"
#include "oled.h"
#include "Motion_control.h" // 包含运动控制头文件
#include "Odometry.h"       // 包含里程计头文件
#include <stdio.h>

// 定义UI状态的枚举类型
typedef enum {
    UI_STATE_MENU,      // 主菜单界面
    UI_STATE_TASK_VIEW  // 任务状态查看界面
} UIState_t;

// 菜单项文本数组
const char *menu_items[] = {
    "1. Task 1",
    "2. Task 2",
    "3. Task 3",
    "4. Task 4"
};
int selected_item = 0;
int num_menu_items = sizeof(menu_items) / sizeof(char *);

// 当前的UI状态
static UIState_t current_ui_state = UI_STATE_MENU;
// 屏幕重绘标志位
static uint8_t screen_needs_redraw = 1;

// 函数声明
void display_menu(void);
void handle_menu_input(void);
void display_task_status(void);
void handle_task_view_input(void);

/**
  * @brief  显示主菜单
  */
void display_menu() 
{
    OLED_Clear();
    for (int i = 0; i < num_menu_items; i++) 
    {
        char display_str[32];
        if (i == selected_item) 
        {
            snprintf(display_str, sizeof(display_str), "-> %s", menu_items[i]);
        } else 
        {
            snprintf(display_str, sizeof(display_str), "   %s", menu_items[i]);
        }
        OLED_ShowString(i + 1, 1, display_str);
    }
}

/**
  * @brief  处理主菜单界面的按键输入
  */
void handle_menu_input() 
{
    // SELECT键: 切换菜单项
    if (HAL_GPIO_ReadPin(KEY_SELECT_GPIO_Port, KEY_SELECT_Pin) == GPIO_PIN_RESET) 
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(KEY_SELECT_GPIO_Port, KEY_SELECT_Pin) == GPIO_PIN_RESET) 
        {
            selected_item = (selected_item + 1) % num_menu_items;
            screen_needs_redraw = 1; // 仅设置标志，由主循环绘制
            while(HAL_GPIO_ReadPin(KEY_SELECT_GPIO_Port, KEY_SELECT_Pin) == GPIO_PIN_RESET);
        }
    }

    // OK键: 启动选中的任务
    if (HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET) 
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET) 
        {
            while(HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET);
            
            // 根据选项启动对应的任务
            switch (selected_item) {
                case 0: Task1_Start(); break;
                case 1: Task2_Start(); break;
                case 2: Task3_Start(); break;
                case 3: Task4_Start(); break;
            }
            current_ui_state = UI_STATE_TASK_VIEW; // 切换到任务查看界面
            
            // 立即重绘任务状态界面
            screen_needs_redraw = 1; // 确保display函数能执行
        }
    }
}

/**
  * @brief  显示任务运行状态
  */
void display_task_status(void)
{
    // 仅在需要时重绘
    if (!screen_needs_redraw) return;

    OLED_Clear();
    char buffer[32];
    
    TaskStatus_t status = Task_Get_Status();
    
    // Line 1: 显示任务状态
    switch(status)
    {
        case TASK_STATUS_RUNNING:
            snprintf(buffer, sizeof(buffer), "Task %d: Running", g_current_task_id);
            OLED_ShowString(1, 1, buffer);
            break;
        case TASK_STATUS_COMPLETE:
            snprintf(buffer, sizeof(buffer), "Task %d: Complete", g_current_task_id);
            OLED_ShowString(1, 1, buffer);
            break;
        default:
            OLED_ShowString(1, 1, "Task: Idle");
            break;
    }

    // Line 2: 显示操作提示
    if (status == TASK_STATUS_RUNNING) {
        OLED_ShowString(2, 1, "OK to Stop");
    } else {
        OLED_ShowString(2, 1, "OK to Back");
    }

    screen_needs_redraw = 0; // 清除重绘标志
}

/**
  * @brief  处理任务查看界面的按键输入
  */
void handle_task_view_input(void)
{
    // 任何状态下，OK键都用于停止任务并返回主菜单
    if (HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET) 
    {
        HAL_Delay(50); 
        if (HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET) 
        {
            while(HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET);

            Task_Stop(); 
            
            current_ui_state = UI_STATE_MENU; 
            
            // 关键：立即重绘主菜单界面，消除视觉延迟
            screen_needs_redraw = 1; // 确保display函数能执行

        }
    }
}


/**
  * @brief  菜单总处理函数，应在主循环中被反复调用
  */
void Menu_Handler(void)
{
    static TaskStatus_t last_task_status = TASK_STATUS_IDLE; 

    // 步骤一：根据当前状态，处理输入和逻辑
    // 输入处理函数可能会改变 current_ui_state
    switch (current_ui_state) 
    {
        case UI_STATE_MENU:
            handle_menu_input();
            break;

        case UI_STATE_TASK_VIEW:
            handle_task_view_input(); // 处理返回主菜单的按键
            Task_Update();            // 更新任务逻辑

            // 检查任务自身状态变化是否需要重绘
            TaskStatus_t current_status = Task_Get_Status();
            if (current_status != last_task_status)
            {
                screen_needs_redraw = 1; 
                last_task_status = current_status;
            }
            break;
    }

    // 步骤二：根据最终的状态，如果需要，执行绘图
    // 这是整个UI的唯一绘图出口，逻辑清晰
    if (screen_needs_redraw)
    {
        switch (current_ui_state)
        {
            case UI_STATE_MENU:
                display_menu();
                break;
            case UI_STATE_TASK_VIEW:
                display_task_status();
                break;
        }
        screen_needs_redraw = 0; // 绘制完成后，统一清除标志
    }
}
