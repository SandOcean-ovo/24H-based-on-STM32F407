#include "main.h"
#include "Sensor.h"
#include "OLED.h"
#include "Encoder.h"

// --- 重要的宏定义 ---

// 传感器数量
#define NUM_SENSORS 8

// 传感器权重（位置值），可以根据实际情况微调
// 权重越大，代表该传感器对最终误差值的影响越大
const uint16_t sensor_weights[NUM_SENSORS] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000};

// 传感器阵列的中心位置值 = (0 + 7000) / 2 = 3500
#define SENSOR_CENTER_VALUE 3500

// 定义每个传感器对应的GPIO端口和引脚
// !!! 注意：这里的定义需要你根据自己的实际接线来修改 !!!
GPIO_TypeDef* sensor_ports[NUM_SENSORS] = {
    Sensor_1_GPIO_Port, Sensor_2_GPIO_Port, Sensor_3_GPIO_Port, Sensor_4_GPIO_Port,
    Sensor_5_GPIO_Port, Sensor_6_GPIO_Port, Sensor_7_GPIO_Port, Sensor_8_GPIO_Port
};
uint16_t sensor_pins[NUM_SENSORS] = {
    Sensor_1_Pin, Sensor_2_Pin, Sensor_3_Pin, Sensor_4_Pin,
    Sensor_5_Pin, Sensor_6_Pin, Sensor_7_Pin, Sensor_8_Pin
};

// --- 私有变量 ---
static int16_t last_error = 0; // 用于存储上一次的有效误差值



uint8_t Sensor_ReadRaw(void)
{
    uint8_t raw_value = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (HAL_GPIO_ReadPin(sensor_ports[i], sensor_pins[i]) == GPIO_PIN_SET) {
            raw_value |= (1 << i); // 读到白色(1)，对应位置1
        }
    }
    return raw_value;
}

int16_t Sensor_GetError(void)
{
    uint32_t weighted_sum = 0; // 加权总和
    uint8_t black_count = 0;   // 检测到黑线的传感器数量
    bool line_detected = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        // 传感器读到0为黑色，1为白色
        if (HAL_GPIO_ReadPin(sensor_ports[i], sensor_pins[i]) == GPIO_PIN_RESET) {
            weighted_sum += sensor_weights[i];
            black_count++;
            line_detected = true;
        }
    }

    // --- 处理特殊情况 ---

    // 1. 如果没有检测到任何黑线 (小车可能完全偏离了)
    if (!line_detected) {
        // 返回上一次的误差值，让小车能根据记忆"找回"黑线
        // 如果上一次误差是正数（线在右边），就返回最大正误差，让车往右猛打
        // 如果上一次误差是负数（线在左边），就返回最大负误差，让车往左猛打
        if (last_error > 0) {
            return SENSOR_CENTER_VALUE; // 最大正误差
        } else {
            return -SENSOR_CENTER_VALUE; // 最大负误差
        }
    }

    // --- 计算误差 ---
    
    // 计算黑线的平均位置
    // 注意：这里必须先乘1再除，避免整型除法丢失精度
    int32_t line_position = weighted_sum / black_count; 

    // 计算相对于中心的误差值
    int16_t error = line_position - SENSOR_CENTER_VALUE;
    
    // 记录本次有效误差
    last_error = error;
    
    return error;
}


// ----------------- 新增：串行传感器循迹逻辑 -----------------

// 串行传感器权重，根据传感器从左到右的物理顺序赋值。
// 中心对称，负值代表线在车体左侧，正值在右侧。
// 这些值的绝对大小和间距会影响PID控制器的响应灵敏度。
static const int16_t g_serial_sensor_weights[8] = {-250, -130, -65, -30, 30, 65, 130, 250};
// 单独为串行传感器记录上一次误差，用于丢线后寻线
// (当前代码逻辑并未充分利用此变量，可用于未来功能扩展)
static int16_t g_serial_last_error = 0;

/**
 * @brief  【循迹专用】读取串行灰度传感器并计算循迹误差
 * @note   通过加权平均法计算黑线偏离中心的误差。
 * @return int16_t 误差值。
 *         - 负数: 黑线在车体左侧
 *         - 正数: 黑线在车体右侧
 *         - 0:    黑线在中心
 *         - -1:   【特殊】所有传感器都未检测到黑线 (完全丢线)
 */
int16_t Sensor_Get_Line_Error(void)
{
    // 1. 通过串行协议读取8个传感器的原始0/1状态 (0=黑, 1=白)
    uint8_t sensor_values = gw_gray_serial_read();
    
    // 2. 如果所有传感器都没读到线 (全为1)，说明完全偏离轨道
    if (sensor_values == 0xFF)
    {
        // 返回-1作为丢线标志，由上层运动控制逻辑处理
        return -1;
    }

    // 3. 加权平均计算
    // 遍历8个传感器的读数，累加所有检测到黑线的传感器的权重
    int32_t weighted_sum = 0; // 加权总和
    uint8_t black_count = 0;  // 检测到黑线的传感器数量
    for (int i = 0; i < 8; i++)
    {
        // 检查sensor_values的第i位是否为0 (0代表检测到黑线)
        if (!((sensor_values >> i) & 0x01)) 
        {
            weighted_sum += g_serial_sensor_weights[i];
            black_count++;
        }
    }
    
    // 4. 计算平均误差
    // 平均位置 = 加权总和 / 黑线传感器数量
    // 例如，如果只有最左边的传感器(-150)检测到黑线，误差就是 -150/1 = -150
    // 如果左边两个传感器(-150, -75)检测到，误差就是 (-150-75)/2 = -112.5
    int16_t error = 0;
    if (black_count > 0) { // 避免除以0的错误
        error = weighted_sum / black_count;
    }
    
    // 5. 记录本次有效误差，可用于未来的丢线补偿逻辑
    g_serial_last_error = error;
    
    return error;
}


/**
 * @brief  执行与串行灰度传感器的时序通信，读取8位数据
 * @note   这是一个特定的同步串行协议，通过控制时钟线(OUT)和读取数据线(IN)实现。
 *         需要外部提供微秒级延时函数 DWT_Delay_us()。
 * @return uint8_t 8位传感器数据，每一位对应一个传感器状态 (0=黑, 1=白)。
 */
uint8_t gw_gray_serial_read(void)
{
    uint8_t ret = 0;
    uint8_t i;

    // 循环8次，依次读取8个传感器的数据
    for (i = 0; i < 8; ++i) {
        // 1. 控制时钟线(OUT)产生一个下降沿 (高->低)
        HAL_GPIO_WritePin(Serial_OUT_GPIO_Port, Serial_OUT_Pin, GPIO_PIN_RESET);
        DWT_Delay_us(2); // 延时确保信号稳定

        // 2. 在时钟为低电平时，从数据线(IN)读取1位数据
        //    并将该位左移i位，通过或运算存入结果变量ret的对应位置
        //    例如，第一次循环(i=0)，读取的数据放在第0位；第二次(i=1)，放在第1位...
        ret |= (HAL_GPIO_ReadPin(Serial_IN_GPIO_Port, Serial_IN_Pin) << i);

        // 3. 控制时钟线(OUT)产生一个上升沿 (低->高)，结束本次位的读取
        HAL_GPIO_WritePin(Serial_OUT_GPIO_Port, Serial_OUT_Pin, GPIO_PIN_SET);
        DWT_Delay_us(5); // 延时为下一次读取做准备
    }
    return ret;
}

void Serial_Sensor_Init(void)
{
    DWT_Init();
}
