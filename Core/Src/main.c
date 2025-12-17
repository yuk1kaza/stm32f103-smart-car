/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  * stm32f103_car_v1.0智能小车，连接了�????个l298n、两个电机�?�一个超声波模块、五个红外模块�?�一块四线OLED屏幕�????
  * 拥有超声波避障�?�循迹寻线�?�OLED显示等功能�??
  * 其中PA0、PA3分别连接了l298n驱动模块的PWM输入端INA INB, PA1、PA2、PA4、PA5分别连接了l298n驱动模块的输入端IN1、IN2、IN3、IN4
  * PB0、PB1、PB3、PB4、PB5分别连接了红外循迹寻线传感器模块的左外�?�左内�?�中间�?�右内�?�右外信号输出端�????
  * PB14和PB15分别连接了超声波模块的TRIG和ECHO信号输出端，
  * PB6连接了OLED屏幕的SCL信号输入端，PB7连接了OLED屏幕的SDA信号输入端�??
  * stm32f103_car_v1.0智能小车，连接了两个L298N、两个电机、一个超声波模块、五个红外模块、一个蓝牙模块、一块四线OLED屏幕。
  * 拥有超声波避障、循迹寻线、OLED显示等功能。
  * 
  * 硬件连接：
  * - PA0、PA3: L298N驱动模块的PWM输入端(INA/INB)
  * - PA1、PA2、PA4、PA5: L298N驱动模块的输入端(IN1/IN2/IN3/IN4)
  * - PB0、PB1、PB3、PB4、PB5: 红外循迹传感器(左外/左内/中间/右内/右外)
  * - PB14、PB15: 超声波模块(TRIG/ECHO)
  * - PB6、PB7: OLED屏幕(SCL/SDA)
  * - PA9、PA10: 蓝牙模块HC-05(TX/RX)


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
#include <stdio.h>
#include <string.h> 

#include "stm32f1xx_hal.h"  
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// PID控制器结构体（添加积分项）
typedef struct{
	float Kp;           // 比例系数
	float Ki;           // 积分系数
	float Kd;           // 微分系数
	int error;          // 当前误差
	int last_error;     // 上次误差
	float integral;     // 误差积分
	int output;         // PID输出
}PID_Controller;

// 运行模式枚举
typedef enum{
	MODE_STOP = 0,      // 停止模式
	MODE_LINE_TRACK,    // 循迹模式
	MODE_MANUAL,        // 手动模式（蓝牙控制）
	MODE_AVOID          // 避障模式（预留）
}RunMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PWM占空比定义（0-9999）
#define PWM_SPEED_MAX    9999   // 最大速度：100%
#define PWM_SPEED_MIN    500      // 最小速度：5%

// 循迹基础速度
#define BASE_SPEED       4000   // 循迹基础速度：40%da

// PID参数（根据STM32 PWM范围调整）
#define KP               15.0f  // 比例系数（调整为适合0-9999范围）
#define KI               0.05f  // 积分系数（小值，避免积分饱和）
#define KD               15.0f  // 微分系数（增大以抑制震荡）

// 积分限幅（防止积分饱和）
#define INTEGRAL_MAX     200.0f

// 红外传感器权重（根据STM32 PWM范围调整）
// 权重需要与PWM范围匹配，使得调整量在合理范围内
// 左外(-80) 左内(-40) 中间(0) 右内(40) 右外(80)
#define WEIGHT_LEFT_OUT  -80
#define WEIGHT_LEFT_IN   -40
#define WEIGHT_MID        0
#define WEIGHT_RIGHT_IN   40
#define WEIGHT_RIGHT_OUT  80

// 超声波测距参数
#define ULTRASONIC_TIMEOUT  30000   // 超时时间（微秒）：对应最大距离约5米
#define SOUND_SPEED         0.034f  // 声速：0.034 cm/us (340m/s)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PID控制器实例
PID_Controller pid = {KP, KI, KD, 0, 0, 0.0f, 0};

// 运行模式
RunMode current_mode = MODE_LINE_TRACK;  // 默认循迹模式

// 蓝牙接收缓冲区
uint8_t bt_rx_buffer[1];
uint8_t bt_command = 0;

// 超声波测距变量
float ultrasonic_distance = 0.0f;  // 当前测量距离（cm）

// 站点停靠相关变量
uint16_t station_count = 0;        // 已停靠站点数
uint8_t stop_time_seconds = 10;    // 停靠时间（秒），可通过上位机设置
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 电机控制函数
void Motor_Stop(void);
void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_TurnLeft(uint16_t speed);
void Motor_TurnRight(uint16_t speed);
void Motor_DifferentialSpeed(int16_t left_speed, int16_t right_speed);

// 红外传感器读取函数
int8_t Read_IR_Sensors(void);
int Calculate_Error(void);

// PID控制函数（添加积分项）
void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
int PID_Calculate(PID_Controller *pid, int error);
void Line_Tracking_PID(void);

// 站点停靠函数
void Station_Stop(uint8_t stop_seconds);

// 蓝牙控制函数
void Bluetooth_Init(void);
void Bluetooth_Process(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 超声波测距函数
float Ultrasonic_GetDistance(void);
void Ultrasonic_SendDebugInfo(void);

// 串口发送函数
void UART_SendString(const char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  停止电机
  * @retval None
  */
void Motor_Stop(void)
{
	// 停止PWM输出
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	
	// 所有方向控制引脚设为低电平
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  前进
  * @param  speed: PWM占空比 (0-9999)
  * @retval None
  */
void Motor_Forward(uint16_t speed)
{
	// 左电机正转：IN1=1, IN2=0
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);
	
	// 右电机正转：IN3=1, IN4=0
	HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
	
	// 设置PWM占空比
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  后退
  * @param  speed: PWM占空比 (0-9999)
  * @retval None
  */
void Motor_Backward(uint16_t speed)
{
	// 左电机反转：IN1=0, IN2=1
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET);
	
	// 右电机反转：IN3=0, IN4=1
	HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);
	
	// 设置PWM占空比
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  左转（原地）
  * @param  speed: PWM占空比 (0-9999)
  * @retval None
  */
void Motor_TurnLeft(uint16_t speed)
{
	// 左电机反转：IN1=0, IN2=1
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET);
	
	// 右电机正转：IN3=1, IN4=0
	HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
	
	// 设置PWM占空比
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  右转（原地）
  * @param  speed: PWM占空比 (0-9999)
  * @retval None
  */
void Motor_TurnRight(uint16_t speed)
{
	// 左电机正转：IN1=1, IN2=0
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);
	
	// 右电机反转：IN3=0, IN4=1
	HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);
	
	// 设置PWM占空比
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  差速控制（左右电机不同速度）
  * @param  left_speed: 左电机速度 (-9999 ~ 9999，负数为反转)
  * @param  right_speed: 右电机速度 (-9999 ~ 9999，负数为反转)
  * @retval None
  */
void Motor_DifferentialSpeed(int16_t left_speed, int16_t right_speed)
{
	// 限幅
	if(left_speed > PWM_SPEED_MAX) left_speed = PWM_SPEED_MAX;
	if(left_speed < -PWM_SPEED_MAX) left_speed = -PWM_SPEED_MAX;
	if(right_speed > PWM_SPEED_MAX) right_speed = PWM_SPEED_MAX;
	if(right_speed < -PWM_SPEED_MAX) right_speed = -PWM_SPEED_MAX;
	
	// 左电机方向控制
	if(left_speed >= 0)
	{
		// 正转
		HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed);
	}
	else
	{
		// 反转
		HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -left_speed);
	}
	
	// 右电机方向控制
	if(right_speed >= 0)
	{
		// 正转
		HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_speed);
	}
	else
	{
		// 反转
		HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -right_speed);
	}
}

/**
  * @brief  读取红外传感器状态
  * @retval 传感器状态（5位二进制，1表示检测到黑线，0表示白色）
  *         bit4: 左外, bit3: 左内, bit2: 中间, bit1: 右内, bit0: 右外
  */
int8_t Read_IR_Sensors(void)
{
	int8_t sensor_value = 0;
	
	// 读取五个红外传感器（标准逻辑：黑线=高电平，白色=低电平）
	if(HAL_GPIO_ReadPin(GPIOB, LEFT1_Pin) == GPIO_PIN_SET)  // 左外 PB0
		sensor_value |= 0x10;
	
	if(HAL_GPIO_ReadPin(GPIOB, LEFT2_Pin) == GPIO_PIN_SET)  // 左内 PB1
		sensor_value |= 0x08;
	
	if(HAL_GPIO_ReadPin(GPIOB, MID_Pin) == GPIO_PIN_SET)    // 中间 PB3
		sensor_value |= 0x04;
	
	if(HAL_GPIO_ReadPin(GPIOB, RIGHT2_Pin) == GPIO_PIN_SET) // 右内 PB4
		sensor_value |= 0x02;
	
	if(HAL_GPIO_ReadPin(GPIOB, RIGHT1_Pin) == GPIO_PIN_SET) // 右外 PB5
		sensor_value |= 0x01;
	
	return sensor_value;
}

/**
  * @brief  计算位置误差（加权法，根据STM32 PWM范围调整）
  * @retval 位置误差（负数=偏左，正数=偏右）
  */
int Calculate_Error(void)
{
	int8_t sensors = Read_IR_Sensors();
	int error = 0;
	
	// 使用加权策略计算误差（权重已调整为适合0-9999 PWM范围）
	if(sensors & 0x10) error += WEIGHT_LEFT_OUT;   // 左外 -800
	if(sensors & 0x08) error += WEIGHT_LEFT_IN;    // 左内 -400
	if(sensors & 0x04) error += WEIGHT_MID;        // 中间  0
	if(sensors & 0x02) error += WEIGHT_RIGHT_IN;   // 右内 +400
	if(sensors & 0x01) error += WEIGHT_RIGHT_OUT;  // 右外 +800
	
	return error;
}

/**
  * @brief  初始化PID控制器
  * @param  pid: PID控制器指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @retval None
  */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->error = 0;
	pid->last_error = 0;
	pid->integral = 0.0f;
	pid->output = 0;
}

/**
  * @brief  PID计算（添加积分项）
  * @param  pid: PID控制器指针
  * @param  error: 当前误差
  * @retval PID输出值
  */
int PID_Calculate(PID_Controller *pid, int error)
{
	// 保存当前误差
	pid->error = error;
	
	// 计算积分项（带限幅，防止积分饱和）
	pid->integral += (float)error;
	if(pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
	if(pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;
	
	// PID计算：output = Kp*error + Ki*integral + Kd*(error - last_error)
	pid->output = (int)(pid->Kp * pid->error + 
	                    pid->Ki * pid->integral + 
	                    pid->Kd * (pid->error - pid->last_error));
	
	// 保存误差用于下次计算
	pid->last_error = error;
	
	return pid->output;
}

/**
  * @brief  站点停靠功能（停靠指定时间）
  * @param  stop_seconds: 停靠时间（秒）
  * @retval None
  */
void Station_Stop(uint8_t stop_seconds)
{
	Motor_Stop();
	station_count++;  // 站点数+1
	
	// 上报停靠状态
	char buffer[50];
	sprintf(buffer, "STATUS:STOP,STATION:%d\r\n", station_count);
	UART_SendString(buffer);
	
	// 停靠指定时间
	HAL_Delay(stop_seconds * 1000);
	
	// 清空PID积分
	pid.integral = 0.0f;
	
	// 上报恢复前进状态
	UART_SendString("STATUS:FORWARD\r\n");
}

/**
  * @brief  基于PID的循迹控制（添加积分项，优化全白/全黑处理，支持站点停靠）
  * @retval None
  */
void Line_Tracking_PID(void)
{
	int8_t sensors = Read_IR_Sensors();
	
	// 检测全黑（站点）：停靠指定时间
	if(sensors == 0x1F)  // 0b11111
	{
		Station_Stop(stop_time_seconds);
		return;
	}
	
	// 检测全白（脱离轨道）：保持上次方向继续运行
	if(sensors == 0x00)  // 0b00000
	{
		// 使用上次的误差继续运行，保持转向
		// 不更新积分项，避免积分发散
		int motor_adjust = (int)(pid.Kp * pid.last_error + 
		                         pid.Kd * (0 - pid.last_error));
		
		int16_t left_speed = BASE_SPEED + motor_adjust;
		int16_t right_speed = BASE_SPEED - motor_adjust;
		
		// 限幅（允许速度降到0）
		if(left_speed > PWM_SPEED_MAX) left_speed = PWM_SPEED_MAX;
		if(left_speed < 0) left_speed = 0;
		if(right_speed > PWM_SPEED_MAX) right_speed = PWM_SPEED_MAX;
		if(right_speed < 0) right_speed = 0;
		
		Motor_DifferentialSpeed(left_speed, right_speed);
		return;
	}
	
	// 正常循迹：计算误差（加权法）
	int error = Calculate_Error();
	
	// PID计算
	int motor_adjust = PID_Calculate(&pid, error);
	
	// 根据PID输出调整左右电机速度
	// leftSpeed = baseSpeed + motorAdjust
	// rightSpeed = baseSpeed - motorAdjust
	int16_t left_speed = BASE_SPEED + motor_adjust;
	int16_t right_speed = BASE_SPEED - motor_adjust;
	
	// 限幅（允许速度降到0，实现更大的转向差速）
	if(left_speed > PWM_SPEED_MAX) left_speed = PWM_SPEED_MAX;
	if(left_speed < 0) left_speed = 0;  // 允许降到0，但不反转
	if(right_speed > PWM_SPEED_MAX) right_speed = PWM_SPEED_MAX;
	if(right_speed < 0) right_speed = 0;  // 允许降到0，但不反转
	
	// 差速控制
	Motor_DifferentialSpeed(left_speed, right_speed);
}

/**
  * @brief  微秒级延时函数（使用TIM1）
  * @param  us: 延时微秒数
  * @retval None
  */
void delay_us(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // 设置计数器值为0
	while(__HAL_TIM_GET_COUNTER(&htim1) < us);  // 等待计数器达到us
}

/**
  * @brief  超声波测距函数
  * @retval 距离（cm），如果超时返回-1
  */
float Ultrasonic_GetDistance(void)
{
	uint32_t echo_time = 0;
	uint32_t timeout = 0;
	
	// 1. 发送10us的触发脉冲
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	
	// 2. 等待ECHO引脚变为高电平（超时保护）
	timeout = ULTRASONIC_TIMEOUT;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
	{
		if(--timeout == 0) return -1.0f;  // 超时
	}
	
	// 3. 开始计时（ECHO高电平持续时间）
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	
	// 4. 等待ECHO引脚变为低电平（超时保护）
	timeout = ULTRASONIC_TIMEOUT;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
	{
		if(--timeout == 0) return -1.0f;  // 超时
		if(__HAL_TIM_GET_COUNTER(&htim1) > ULTRASONIC_TIMEOUT) return -1.0f;  // 超时
	}
	
	// 5. 读取计时器值
	echo_time = __HAL_TIM_GET_COUNTER(&htim1);
	
	// 6. 计算距离：distance = (echo_time * sound_speed) / 2
	// echo_time单位：微秒，sound_speed = 0.034 cm/us
	// 除以2是因为声波往返
	float distance = (echo_time * SOUND_SPEED) / 2.0f;
	
	return distance;
}

/**
  * @brief  串口发送字符串
  * @param  str: 要发送的字符串
  * @retval None
  */
void UART_SendString(const char *str)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
}

/**
  * @brief  发送超声波调试信息到串口
  * @retval None
  */
void Ultrasonic_SendDebugInfo(void)
{
	char buffer[100];
	
	// 测量距离
	ultrasonic_distance = Ultrasonic_GetDistance();
	
	// 格式化输出
	if(ultrasonic_distance < 0)
	{
		sprintf(buffer, "Distance: ERROR (Timeout)\r\n");
	}
	else
	{
		sprintf(buffer, "Distance: %.2f cm\r\n", ultrasonic_distance);
	}
	
	// 发送到串口
	UART_SendString(buffer);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	// 初始化OLED显示屏（暂时注释，排查停顿问题）
	// HAL_Delay(20);
	// OLED_Init();
	
	// 禁用JTAG，释放PB3、PB4引脚作为普通GPIO
	// 保留SWD调试功能（PA13/PA14）
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
	
	// 启动TIM1（用于超声波微秒延时�?
	HAL_TIM_Base_Start(&htim1);
	
	// 启动PWM输出
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	// 初始化PID控制器（添加积分项）
	PID_Init(&pid, KP, KI, KD);
	
	// 初始化蓝牙模�?
	Bluetooth_Init();
	
	// 初始化电机为停止状�??
	Motor_Stop();
	
	// 短暂延时，等待系统稳�?
	HAL_Delay(500);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// 处理蓝牙命令
		Bluetooth_Process();
		
		// 根据当前模式执行相应功能
		switch(current_mode)
		{
			case MODE_STOP:
				// 停止模式：什么都不做
				Motor_Stop();
				break;
				
			case MODE_LINE_TRACK:
				// 循迹模式：执行PID循迹
				Line_Tracking_PID();
				break;
				
			case MODE_MANUAL:
				// 手动模式：由蓝牙命令控制，这里不需要额外处理
				break;
				
			case MODE_AVOID:
				// 避障模式：预留，暂时停止
				Motor_Stop();
				break;
				
			default:
				current_mode = MODE_STOP;
				break;
		}
		
		// 定时状态上报（每1秒）
		static uint32_t last_report_time = 0;
		if(HAL_GetTick() - last_report_time >= 1000)
		{
			last_report_time = HAL_GetTick();
			
			// 根据模式上报状态
			if(current_mode == MODE_LINE_TRACK)
			{
				// 循迹模式：上报前进状态
				UART_SendString("STATUS:FORWARD\r\n");
			}
			else if(current_mode == MODE_MANUAL)
			{
				// 手动模式：上报手动状态
				UART_SendString("STATUS:MANUAL\r\n");
			}
		}
		
		// 超声波调试：定时发送距离数据（每500ms）
		// 取消注释以启用自动超声波调试
		// static uint32_t last_ultrasonic_time = 0;
		// if(HAL_GetTick() - last_ultrasonic_time >= 500)
		// {
		// 	last_ultrasonic_time = HAL_GetTick();
		// 	Ultrasonic_SendDebugInfo();
		// }
		
		// 可选：OLED显示当前状态
		// OLED_NewFrame();
		// char mode_str[20];
		// sprintf(mode_str, "Mode: %d", current_mode);
		// OLED_PrintASCIIString(0, 0, mode_str, &afont8x6, 0);
		// OLED_ShowFrame();
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  初始化蓝牙模�?
  * @retval None
  */
void Bluetooth_Init(void)
{
	// 启动UART接收中断
	HAL_UART_Receive_IT(&huart1, bt_rx_buffer, 1);
}

/**
  * @brief  处理蓝牙命令
  * @retval None
  */
void Bluetooth_Process(void)
{
	if(bt_command == 0) return;  // 没有新命令
	
	uint8_t cmd = bt_command;
	bt_command = 0;  // 清除命令
	
	// 数字键0-9的处理（根据模式不同有不同功能）
	if(cmd >= '0' && cmd <= '9')
	{
		if(current_mode == MODE_STOP)
		{
			// 停止模式下：设置停靠时间
			stop_time_seconds = cmd - '0';  // ASCII转数字
			if(stop_time_seconds == 0) stop_time_seconds = 10;  // 0表示10秒
			char confirm[50];
			sprintf(confirm, "STOP_TIME_SET:%d\r\n", stop_time_seconds);
			UART_SendString(confirm);
		}
		else
		{
			// 其他模式下：切换模式
			switch(cmd)
			{
				case '0':  // 停止模式
					current_mode = MODE_STOP;
					Motor_Stop();
					break;
					
				case '1':  // 循迹模式
					current_mode = MODE_LINE_TRACK;
					pid.integral = 0.0f;  // 清空积分
					break;
					
				case '2':  // 手动模式
					current_mode = MODE_MANUAL;
					break;
			}
		}
		return;  // 处理完数字键后直接返回
	}
	
	// 其他命令处理
	switch(cmd)
	{
		// 手动控制（仅在手动模式下有效）
		case 'F':  // 前进
		case 'f':
			if(current_mode == MODE_MANUAL)
				Motor_Forward(BASE_SPEED);
			break;
			
		case 'B':  // 后退
		case 'b':
			if(current_mode == MODE_MANUAL)
				Motor_Backward(BASE_SPEED);
			break;
			
		case 'L':  // 左转
		case 'l':
			if(current_mode == MODE_MANUAL)
				Motor_TurnLeft(BASE_SPEED);
			break;
			
		case 'R':  // 右转
		case 'r':
			if(current_mode == MODE_MANUAL)
				Motor_TurnRight(BASE_SPEED);
			break;
			
		case 'S':  // 停止
		case 's':
			Motor_Stop();
			break;
		
		// 超声波测距（调试命令）
		case 'U':  // 超声波测距
		case 'u':
			Ultrasonic_SendDebugInfo();
			break;
		
		// 查询状态
		case 'Q':  // 查询当前状态
		case 'q':
		{
			char status_buffer[100];
			sprintf(status_buffer, "MODE:%d,STATION:%d,STOP_TIME:%d\r\n", 
			        current_mode, station_count, stop_time_seconds);
			UART_SendString(status_buffer);
			break;
		}
		
		default:
			break;
	}
}

/**
  * @brief  UART接收完成回调函数
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		// 保存接收到的命令
		bt_command = bt_rx_buffer[0];
		
		// 重新启动接收
		HAL_UART_Receive_IT(&huart1, bt_rx_buffer, 1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
