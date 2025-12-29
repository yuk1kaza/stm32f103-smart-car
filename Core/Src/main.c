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
#include "oled.h"
#include <string.h>
#include <stdio.h>
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
	MODE_LINE_TRACK,    // 循迹模式（含自动避障）
	MODE_MANUAL,        // 手动模式（蓝牙控制）
	MODE_AVOID          // 避障模式（独立使用，已集成至循迹模式）
}RunMode;

// 运动方向枚举
typedef enum {
	DIR_STOP = 0,
	DIR_FORWARD,
	DIR_BACKWARD,
	DIR_LEFT,
	DIR_RIGHT
} Direction;

// 车速等级枚举
typedef enum {
	SPEED_SLOW = 0,    // 慢速
	SPEED_MEDIUM,      // 中速
	SPEED_FAST         // 快速
} SpeedLevel;

// 系统状态结构体
typedef struct {
	Direction direction;      // 当前运动方向
	SpeedLevel speed_level;   // 车速等级
	uint16_t station_count;   // 停靠站点数
	uint8_t countdown;        // 倒计时（秒）
	uint8_t is_stopped;       // 是否停靠中
} SystemStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PWM占空比定义（0-9999）
#define PWM_SPEED_MAX    4300   // 最大速度：43%


// 循迹基础速度
#define BASE_SPEED       3500   // 循迹基础速度：35%

// PID参数（根据STM32 PWM范围调整）
#define KP               45.0f  // 比例系数（调整为适合0-9999范围）
#define KI               0.05f  // 积分系数（小值，避免积分饱和）
#define KD               30.0f  // 微分系数（增大以抑制震荡）

// 积分限幅（防止积分饱和）
#define INTEGRAL_MAX     200.0f

// 红外传感器权重（根据STM32 PWM范围调整）
// 权重需要与PWM范围匹配，使得调整量在合理范围内
// 左外(-800) 左内(-80) 中间(0) 右内(80) 右外(800)
#define WEIGHT_LEFT_OUT  -220
#define WEIGHT_LEFT_IN   -200
#define WEIGHT_MID        0
#define WEIGHT_RIGHT_IN   200
#define WEIGHT_RIGHT_OUT  220

// 超声波测距参数
#define ULTRASONIC_TIMEOUT  30000   // 超时时间（微秒）：对应最大距离约5米
#define SOUND_SPEED         0.034f  // 声速：0.034 cm/us (340m/s)

// 避障参数
#define SAFE_DISTANCE       10.0f   // 安全距离（cm）
#define TURN_90_TIME        600     // 90度转向时间（ms）
#define FORWARD_TIME        1000    // 避障前进时间（ms）
#define MAX_AVOID_DEPTH     3       // 最大递归避障次数
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

// 多字符命令缓冲区（用于接收STxx等命令）
#define CMD_BUFFER_SIZE 10
char cmd_buffer[CMD_BUFFER_SIZE];
uint8_t cmd_index = 0;
uint32_t last_rx_time = 0;

// 超声波测距变量
float ultrasonic_distance = 0.0f;  // 当前测量距离（cm）

// 避障状态变量
uint8_t is_avoiding = 0;           // 是否正在避障
uint8_t avoid_depth = 0;           // 当前避障递归深度

// 系统状态实例
SystemStatus sys_status = {DIR_STOP, SPEED_MEDIUM, 0, 0, 0};

// 站点停靠时间（秒），可通过上位机设置
uint8_t stop_time_seconds = 10;
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

// 站点停靠函数（已废弃，使用Station_Stop_With_Countdown代替）
// void Station_Stop(uint8_t stop_seconds);

// 蓝牙控制函数
void Bluetooth_Init(void);
void Bluetooth_Process(void);
void Process_Multi_Char_Command(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 超声波测距函数
float Ultrasonic_GetDistance(void);
void Ultrasonic_SendDebugInfo(void);

// 串口发送函数
void UART_SendString(const char *str);

// OLED显示函数
void OLED_UpdateStatus(void);
uint16_t Get_Speed_Percent(SpeedLevel level);
uint16_t Get_Speed_Value(SpeedLevel level);
void Station_Stop_With_Countdown(uint8_t stop_time);

// 避障函数
uint8_t Execute_Avoidance_Maneuver(void);
void Line_Tracking_With_Avoidance(void);
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
	
	// 更新状态
	sys_status.direction = DIR_STOP;
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
	
	// 更新状态
	sys_status.direction = DIR_FORWARD;
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
	
	// 更新状态
	sys_status.direction = DIR_BACKWARD;
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
	
	// 更新状态
	sys_status.direction = DIR_LEFT;
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
	
	// 更新状态
	sys_status.direction = DIR_RIGHT;
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
  * @brief  PID计算（添加积分项和抗饱和机制）
  * @param  pid: PID控制器指针
  * @param  error: 当前误差
  * @retval PID输出值
  */
int PID_Calculate(PID_Controller *pid, int error)
{
	// 保存当前误差
	pid->error = error;
	
	// 误差接近0时，清空积分（防止积分累积）
	if(error > -10 && error < 10)
	{
		pid->integral *= 0.5f;  // 快速衰减积分
	}
	else
	{
		// 计算积分项（带限幅，防止积分饱和）
		pid->integral += (float)error;
		if(pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
		if(pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;
	}
	
	// PID计算：output = Kp*error + Ki*integral + Kd*(error - last_error)
	pid->output = (int)(pid->Kp * pid->error + 
	                    pid->Ki * pid->integral + 
	                    pid->Kd * (pid->error - pid->last_error));
	
	// 保存误差用于下次计算
	pid->last_error = error;
	
	return pid->output;
}

/**
  * @brief  执行避障机动（右转90度→斜向前进→回归循迹）
  * @retval 1=需要继续避障, 0=可以返回循迹
  */
uint8_t Execute_Avoidance_Maneuver(void)
{
	uint16_t speed = Get_Speed_Value(sys_status.speed_level);
	
	// 上报避障开始
	char buffer[50];
	sprintf(buffer, "AVOID:START,DEPTH=%d\r\n", avoid_depth);
	UART_SendString(buffer);
	
	// 1. 右转90度，绕过障碍物
	Motor_TurnRight(3000);
	sys_status.direction = DIR_RIGHT;
	OLED_UpdateStatus();
	HAL_Delay(TURN_90_TIME);
	
	// 2. 斜向前进（左速<右速，实现左偏），等待检测到轨道
	while (Read_IR_Sensors() == 0) {
		// 如果前方距离仍然不安全，继续右转
		if(ultrasonic_distance > 0 && ultrasonic_distance < SAFE_DISTANCE)
		{
			Motor_TurnRight(6000);
			sys_status.direction = DIR_RIGHT;
			OLED_UpdateStatus();
			HAL_Delay(TURN_90_TIME);
		}
		
		// 左电机慢、右电机快，实现斜向左前进
		Motor_DifferentialSpeed(2000, 3000);
	}
    
	// 3. 回归循迹（已检测到轨道）
	Line_Tracking_PID();


	// 4. 停止并测距
	Motor_Stop();
	HAL_Delay(100);  // 稳定一下
	
	// 5. 测量新距离
	ultrasonic_distance = Ultrasonic_GetDistance();
	
	// 6. 判断是否需要继续避障
	if(ultrasonic_distance > 0 && ultrasonic_distance < SAFE_DISTANCE)
	{
		// 距离仍然不安全
		sprintf(buffer, "AVOID:CONTINUE,DIST=%.1f\r\n", ultrasonic_distance);
		UART_SendString(buffer);
		return 1;  // 需要继续避障
	}
	else
	{
		sprintf(buffer, "AVOID:END,DIST=%.1f\r\n", ultrasonic_distance);
		UART_SendString(buffer);
		return 0;  // 返回循迹
	}
}

/**
  * @brief  带避障功能的循迹控制
  * @retval None
  */
void Line_Tracking_With_Avoidance(void)
{
	// 检查是否需要避障（距离小于安全值）
	if(ultrasonic_distance > 0 && ultrasonic_distance < SAFE_DISTANCE && !is_avoiding)
	{
		// 触发避障
		is_avoiding = 1;
		avoid_depth = 0;
		
		// 停止当前运动
		Motor_Stop();
		HAL_Delay(200);
		
		// 递归避障，直到安全或达到最大次数
		while(avoid_depth < MAX_AVOID_DEPTH)
		{
			avoid_depth++;
			
			// 执行避障机动
			uint8_t need_continue = Execute_Avoidance_Maneuver();
			
			if(!need_continue)
			{
				// 安全了，退出避障
				break;
			}
			
			// 如果达到最大深度，强制退出
			if(avoid_depth >= MAX_AVOID_DEPTH)
			{
				UART_SendString("AVOID:MAX_DEPTH_REACHED\r\n");
				break;
			}
		}
		
		// 避障结束，清空PID积分
		pid.integral = 0.0f;
		pid.last_error = 0;
		
		// 恢复循迹状态
		is_avoiding = 0;
		avoid_depth = 0;
		
		return;
	}
	
	// 正常循迹（调用原有的PID循迹函数）
	Line_Tracking_PID();
}


/**
  * @brief  基于PID的循迹控制（添加积分项，优化全白/全黑处理，支持站点停靠，反向清零逻辑）
  * @retval None
  */
void Line_Tracking_PID(void)
{
	int8_t sensors = Read_IR_Sensors();
	
	// 检测全黑（站点）：停靠指定时间并显示倒计时
	if(sensors == 0x1F)  // 0b11111
	{
		Motor_Stop();
		Motor_Forward(3000);
		HAL_Delay(400);  // 稳定一下
		if (Read_IR_Sensors() == 0x1F) {
		Station_Stop_With_Countdown(stop_time_seconds);  // 使用设置的停靠时间
		}
		
		return;
	}
	
	// 检测全白（脱离轨道）：保持上次方向继续运行
	if(sensors == 0x00)  // 0b00000
	{
		// 使用上次的误差继续运行，保持转向
		// 不更新积分项，避免积分发散
		int motor_adjust = (int)(pid.Kp * pid.last_error);
		
		uint16_t base_speed = Get_Speed_Value(sys_status.speed_level);
		int16_t left_speed = base_speed + motor_adjust;
		int16_t right_speed = base_speed - motor_adjust;
		
		// 限幅（允许反转）
		if(left_speed > PWM_SPEED_MAX) left_speed = PWM_SPEED_MAX;
		if(left_speed < -PWM_SPEED_MAX) left_speed = -PWM_SPEED_MAX;
		if(right_speed > PWM_SPEED_MAX) right_speed = PWM_SPEED_MAX;
		if(right_speed < -PWM_SPEED_MAX) right_speed = -PWM_SPEED_MAX;
		
		Motor_DifferentialSpeed(left_speed, right_speed);
		sys_status.direction = DIR_FORWARD;
		return;
	}
	
	// 正常循迹：计算误差（加权法）
	int error = Calculate_Error();
	
	// 反向清零逻辑：减少抖动
	// 当小车左转时（上次误差<0），如果右侧传感器检测到黑线，说明已回到轨道，清零误差
	// 当小车右转时（上次误差>0），如果左侧传感器检测到黑线，说明已回到轨道，清零误差
	if(pid.last_error < -20)  // 上次是左转（偏左）
	{
		// 检查右侧传感器（右内或右外）是否检测到黑线
		if(HAL_GPIO_ReadPin(GPIOB, RIGHT2_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(GPIOB, RIGHT1_Pin) == GPIO_PIN_SET)  // 右内(bit1)或右外(bit0)
		{
			// 右侧检测到黑线，说明已经回到轨道，清零误差和积分
			error = 0;
			pid.integral = 0.0f;
			pid.last_error = 0;
		}
	}
	else if(pid.last_error > 20)  // 上次是右转（偏右）
	{
		// 检查左侧传感器（左内或左外）是否检测到黑线
		if(HAL_GPIO_ReadPin(GPIOB, LEFT2_Pin) == GPIO_PIN_SET|| HAL_GPIO_ReadPin(GPIOB, LEFT1_Pin) == GPIO_PIN_SET)  // 左内(bit3)或左外(bit4)
		{
			// 左侧检测到黑线，说明已经回到轨道，清零误差和积分
			error = 0;
			pid.integral = 0.0f;
			pid.last_error = 0;
		}
	}
	
	// PID计算
	int motor_adjust = PID_Calculate(&pid, error);
	
	// 根据PID输出调整左右电机速度
	uint16_t base_speed = Get_Speed_Value(sys_status.speed_level);
	int16_t left_speed = base_speed + motor_adjust;
	int16_t right_speed = base_speed - motor_adjust;
	
	// 限幅（允许反转，实现原地转向）
	if(left_speed > PWM_SPEED_MAX) left_speed = PWM_SPEED_MAX;
	if(left_speed < -PWM_SPEED_MAX) left_speed = -PWM_SPEED_MAX;
	if(right_speed > PWM_SPEED_MAX) right_speed = PWM_SPEED_MAX;
	if(right_speed < -PWM_SPEED_MAX) right_speed = -PWM_SPEED_MAX;
	
	// 差速控制
	Motor_DifferentialSpeed(left_speed, right_speed);
	
	// 更新运动方向
	if(error < -20) sys_status.direction = DIR_LEFT;
	else if(error > 20) sys_status.direction = DIR_RIGHT;
	else sys_status.direction = DIR_FORWARD;
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

/**
  * @brief  获取速度百分比
  * @param  level: 速度等级
  * @retval 速度百分比
  */
uint16_t Get_Speed_Percent(SpeedLevel level)
{
	switch(level)
	{
		case SPEED_SLOW:   return 30;
		case SPEED_MEDIUM: return 35;
		case SPEED_FAST:   return 40;
		default:           return 0;
	}
}

/**
  * @brief  获取速度值
  * @param  level: 速度等级
  * @retval PWM速度值
  */
uint16_t Get_Speed_Value(SpeedLevel level)
{
	switch(level)
	{
		case SPEED_SLOW:   return 3000;    // 30%
		case SPEED_MEDIUM: return 3500;    // 35%
		case SPEED_FAST:   return 4500;    // 45%
		default:           return 0;
	}
}

/**
  * @brief  更新OLED显示状态
  * @retval None
  */
void OLED_UpdateStatus(void)
{
	OLED_NewFrame();
	char buffer[32];
	
	// 第1行：运动方向
	const char* dir_str[] = {"STOP", "FORWARD", "BACK", "LEFT", "RIGHT"};
	sprintf(buffer, "Dir: %s", dir_str[sys_status.direction]);
	OLED_PrintASCIIString(0, 0, buffer, &afont8x6, OLED_COLOR_NORMAL);
	
	// 第2行：车速等级
	const char* speed_str[] = {"SLOW", "MID", "FAST"};
	uint16_t speed_percent = Get_Speed_Percent(sys_status.speed_level);
	sprintf(buffer, "Speed:%s %d%%", speed_str[sys_status.speed_level], speed_percent);
	OLED_PrintASCIIString(0, 16, buffer, &afont8x6, OLED_COLOR_NORMAL);
	
	// 第3行：站点数量
	sprintf(buffer, "Stations: %d", sys_status.station_count);
	OLED_PrintASCIIString(0, 32, buffer, &afont8x6, OLED_COLOR_NORMAL);
	
	// 第4行：倒计时（仅停靠时显示）
	if(sys_status.is_stopped)
	{
		sprintf(buffer, "Countdown: %ds", sys_status.countdown);
		OLED_PrintASCIIString(0, 48, buffer, &afont8x6, OLED_COLOR_NORMAL);
	}
	else if(is_avoiding)
	{
		// 显示避障状态
		sprintf(buffer, "AVOID D=%d", avoid_depth);
		OLED_PrintASCIIString(0, 48, buffer, &afont8x6, OLED_COLOR_NORMAL);
	}
	else
	{
		// 显示超声波距离
		if(ultrasonic_distance > 0)
		{
			sprintf(buffer, "Dist: %.1fcm", ultrasonic_distance);
			OLED_PrintASCIIString(0, 48, buffer, &afont8x6, OLED_COLOR_NORMAL);
		}
	}
	
	OLED_ShowFrame();
}

/**
  * @brief  站点停靠（带倒计时）
  * @param  stop_time: 停靠时间（秒）
  * @retval None
  */
void Station_Stop_With_Countdown(uint8_t stop_time)
{
	Motor_Stop();
	sys_status.is_stopped = 1;
	sys_status.station_count++;  // 站点数+1
	sys_status.direction = DIR_STOP;
	
	// 上报停靠状态
	char buffer[64];
	sprintf(buffer, "STATUS=STOP,STATION=%d,COUNTDOWN=%d\r\n",sys_status.station_count,stop_time);
	UART_SendString(buffer);
	
	// 倒计时
	for(uint8_t i = stop_time; i > 0; i--)
	{
		sys_status.countdown = i;
		
		sprintf(buffer,"STATUS=STOP,STATION=%d,COUNTDOWN=%d\r\n",sys_status.station_count,i);
    UART_SendString(buffer);
		
		OLED_UpdateStatus();  // 更新显示
		HAL_Delay(1000);  // 延时1秒
	}
	
	sys_status.is_stopped = 0;
	sys_status.countdown = 0;
	
	// 清空PID积分
	pid.integral = 0.0f;
	
	// 恢复前进
	sys_status.direction = DIR_FORWARD;
	Motor_Forward(3000);
	HAL_Delay(600);  // 稳定一下
	UART_SendString("STATUS=FORWARD\r\n");
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
	// 初始化OLED显示屏
	HAL_Delay(20);
	OLED_Init();
	
	// 显示启动信息
	OLED_NewFrame();
	OLED_PrintASCIIString(0, 0, "STM32 Car v1.2", &afont8x6, OLED_COLOR_NORMAL);
	OLED_PrintASCIIString(0, 16, "Initializing...", &afont8x6, OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	
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
				// 循迹模式：执行带避障的PID循迹控制
				Line_Tracking_With_Avoidance();
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
		
		// 高频状态上报（每100ms，10Hz）
		static uint32_t last_report_time = 0;
		if(HAL_GetTick() - last_report_time >= 100)
		{
			last_report_time = HAL_GetTick();
			
			// 构建完整状态信息
			char status_buffer[80];
			sprintf(status_buffer, "STATUS:MODE=%d,DIR=%d,SPEED=%d,STATION=%d,DIST=%.1f\r\n",
			        current_mode,
			        sys_status.direction,
			        sys_status.speed_level,
			        sys_status.station_count,
			        ultrasonic_distance > 0 ? ultrasonic_distance : 0.0f);
			UART_SendString(status_buffer);
		}
		
		
		
		// OLED显示当前状态（每200ms刷新一次）
		static uint32_t last_oled_time = 0;
		if(HAL_GetTick() - last_oled_time >= 200)
		{
			last_oled_time = HAL_GetTick();
			OLED_UpdateStatus();
		}
		
		// 超声波测距（每500ms测一次）
		static uint32_t last_ultrasonic_time = 0;
		if(HAL_GetTick() - last_ultrasonic_time >= 500)
		{
			last_ultrasonic_time = HAL_GetTick();
			ultrasonic_distance = Ultrasonic_GetDistance();
		}
		
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
  * @brief  处理多字符命令（如STxx）
  * @retval None
  */
void Process_Multi_Char_Command(void)
{
	// 检查是否是STxx命令（设置停靠时间）
	if(cmd_buffer[0] == 'S' && cmd_buffer[1] == 'T')
	{
		// 提取数字部分
		int time_value = 0;
		int i = 2;
		while(i < cmd_index && cmd_buffer[i] >= '0' && cmd_buffer[i] <= '9')
		{
			time_value = time_value * 10 + (cmd_buffer[i] - '0');
			i++;
		}
		
		// 设置停靠时间（范围：1-99秒）
		if(time_value >= 1 && time_value <= 99)
		{
			stop_time_seconds = time_value;
			char confirm[50];
			sprintf(confirm, "STOP_TIME_SET:%d\r\n", stop_time_seconds);
			UART_SendString(confirm);
		}
		else
		{
			UART_SendString("ERROR:INVALID_TIME_VALUE\r\n");
		}
	}
	// 可以在这里添加其他多字符命令的处理
	else
	{
		UART_SendString("ERROR:UNKNOWN_COMMAND\r\n");
	}
	
	// 清空命令缓冲区
	cmd_index = 0;
	memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
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
	
	// 数字键0-9的处理（仅用于模式切换）
	if(cmd >= '0' && cmd <= '9')
	{
		// 切换模式
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
		return;  // 处理完数字键后直接返回
	}
	
	// 其他命令处理
	switch(cmd)
	{
		// 手动控制（仅在手动模式下有效）
		case 'F':  // 前进
		case 'f':
			if(current_mode == MODE_MANUAL)
				Motor_Forward(Get_Speed_Value(sys_status.speed_level));
			break;
			
		case 'B':  // 后退
		case 'b':
			if(current_mode == MODE_MANUAL)
				Motor_Backward(Get_Speed_Value(sys_status.speed_level));
			break;
			
		case 'L':  // 左转
		case 'l':
			if(current_mode == MODE_MANUAL)
				Motor_TurnLeft(Get_Speed_Value(sys_status.speed_level));
			break;
			
		case 'R':  // 右转
		case 'r':
			if(current_mode == MODE_MANUAL)
				Motor_TurnRight(Get_Speed_Value(sys_status.speed_level));
			break;
			
		case 'S':  // 停止
		case 's':
			Motor_Stop();
			break;
		
		// 速度等级切换
		case '+':  // 加速
			if(sys_status.speed_level < SPEED_FAST)
			{
				sys_status.speed_level++;
				char buffer[30];
				sprintf(buffer, "SPEED:%d\r\n", sys_status.speed_level);
				UART_SendString(buffer);
			}
			break;
			
		case '-':  // 减速
			if(sys_status.speed_level > SPEED_SLOW)
			{
				sys_status.speed_level--;
				char buffer[30];
				sprintf(buffer, "SPEED:%d\r\n", sys_status.speed_level);
				UART_SendString(buffer);
			}
			break;
		
		// 超声波测距（调试命令）
		case 'U':  // 超声波测距
		case 'u':
			Ultrasonic_SendDebugInfo();
			break;
		
		// PID调试信息
		case 'P':  // PID调试
		case 'p':
		{
			char pid_buffer[100];
			int8_t sensors = Read_IR_Sensors();
			sprintf(pid_buffer, "PID:E=%d,I=%.1f,O=%d,S=0x%02X\r\n", 
			        pid.error, pid.integral, pid.output, sensors);
			UART_SendString(pid_buffer);
			break;
		}
		
		// 查询状态
		case 'Q':  // 查询当前状态
		case 'q':
		{
			char status_buffer[100];
			sprintf(status_buffer, "MODE:%d,DIR:%d,SPEED:%d,STATION:%d\r\n", 
			        current_mode, sys_status.direction, sys_status.speed_level, sys_status.station_count);
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
		uint8_t received_char = bt_rx_buffer[0];
		uint32_t current_time = HAL_GetTick();
		
		// 超时检测：如果距离上次接收超过500ms，清空缓冲区
		if(current_time - last_rx_time > 500)
		{
			cmd_index = 0;
			memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
		}
		last_rx_time = current_time;
		
		// 检查是否是回车或换行（命令结束符）
		if(received_char == '\r' || received_char == '\n')
		{
			if(cmd_index > 0)
			{
				// 立即处理多字符命令
				cmd_buffer[cmd_index] = '\0';  // 添加字符串结束符
				Process_Multi_Char_Command();
			}
		}
		// 检查是否是大写字母（可能是多字符命令的开始）
		else if(received_char >= 'A' && received_char <= 'Z')
		{
			// 如果缓冲区为空，开始新命令
			if(cmd_index == 0)
			{
				cmd_buffer[cmd_index++] = received_char;
			}
			// 如果缓冲区已有内容，继续添加
			else if(cmd_index < CMD_BUFFER_SIZE - 1)
			{
				cmd_buffer[cmd_index++] = received_char;
			}
		}
		// 检查是否是数字（可能是多字符命令的参数）
		else if(received_char >= '0' && received_char <= '9')
		{
			// 如果缓冲区有内容（正在接收多字符命令），添加数字
			if(cmd_index > 0 && cmd_index < CMD_BUFFER_SIZE - 1)
			{
				cmd_buffer[cmd_index++] = received_char;
			}
			// 如果缓冲区为空，作为单字符命令处理
			else if(cmd_index == 0)
			{
				bt_command = received_char;
			}
		}
		// 其他单字符命令（F/B/L/R/S/+/-/U/Q等）
		else
		{
			// 清空多字符命令缓冲区
			cmd_index = 0;
			memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
			
			// 保存单字符命令
			bt_command = received_char;
		}
		
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
