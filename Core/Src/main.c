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
  * stm32f103_car_v1.0智能小车，连接了�???个l298n、两个电机�?�一个超声波模块、五个红外模块�?�一块四线OLED屏幕�???
  * 拥有超声波避障�?�循迹寻线�?�OLED显示等功能�??
  * 其中PA0、PA3分别连接了l298n驱动模块的PWM输入端INA INB, PA1、PA2、PA4、PA5分别连接了l298n驱动模块的输入端IN1、IN2、IN3、IN4
  * PB0、PB1、PB3、PB4、PB5分别连接了红外循迹寻线传感器模块的左外�?�左内�?�中间�?�右内�?�右外信号输出端�???
  * PB14和PB15分别连接了超声波模块的TRIG和ECHO信号输出端，
  * PB6连接了OLED屏幕的SCL信号输入端，PB7连接了OLED屏幕的SDA信号输入端�??


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
// 小车结构�???
typedef struct{
	// 状�??(0:stop 1:running 2:pause)
	int state;
	
	// 速度（H:�??? L:低）
	char v;
	
	// 停靠站数�???
	int stasion_amount;
}CAR;

// PID控制器结构体
typedef struct{
	float Kp;           // 比例系数
	float Ki;           // 积分系数
	float Kd;           // 微分系数
	float error;        // 当前误差
	float last_error;   // 上次误差
	float integral;     // 误差积分
	float derivative;   // 误差微分
	float output;       // PID输出
}PID_Controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 调试模式选择（只能�?�择�?个为1，其他为0�?
#define DEBUG_ALL_PINS   0      // 测试�?有GPIOB引脚（找出实际连接）
#define DEBUG_IR_SENSOR  1      // 红外传感器调试模�?
#define DEBUG_MOTOR      0      // 电机调试模式
#define RUN_LINE_TRACK   0      // PID循迹运行模式

// PWM占空比定义（0-9999�???
#define PWM_SPEED_LOW    4000   // 低�?�：40%
#define PWM_SPEED_MID    5000   // 中�?�：50%
#define PWM_SPEED_HIGH   7000   // 高�?�：70%
#define PWM_SPEED_MAX    9999   // �???大�?�度�???100%

// 循迹基础速度
#define BASE_SPEED       4500   // 循迹基础速度�???45%

// PID参数
#define KP               40.0f  // 比例系数
#define KI               0.0f   // 积分系数
#define KD               10.0f  // 微分系数

// 红外传感器权重（用于计算位置误差�???
// 左外(-2) 左内(-1) 中间(0) 右内(1) 右外(2)
#define WEIGHT_LEFT_OUT  -2
#define WEIGHT_LEFT_IN   -1
#define WEIGHT_MID        0
#define WEIGHT_RIGHT_IN   1
#define WEIGHT_RIGHT_OUT  2

// 积分限幅
#define INTEGRAL_MAX     1000.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PID控制器实�???
const uint8_t Data[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
0x80, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xc0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0x7f, 0x3f, 0x3f, 0x0f, 0x07, 0x07, 
0x07, 0x07, 0x07, 0x07, 0x0f, 0x2f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x3f, 0x3f, 0x7f, 0xbf, 0x3f, 0x1f, 0x00, 0x00, 0x07, 0xc7, 0xe3, 0xe0, 0xf0, 0xf0, 0xf0, 0xe3, 0xe7, 0xc7, 
0x00, 0x18, 0x9f, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x3f, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xe0, 0xf0, 
0xf0, 0xf0, 0xfa, 0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xfc, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xe7, 
0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x1f, 0x1f, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x71, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x0f, 
0x07, 0x07, 0x07, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xf7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

};
const Image Img = {114, 64, Data};

PID_Controller pid = {KP, KI, KD, 0, 0, 0, 0, 0};
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

// 红外传感器读取函�???
int8_t Read_IR_Sensors(void);
float Calculate_Position_Error(uint8_t *sensor_status);

// PID控制函数
void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Calculate(PID_Controller *pid, float error);
void Line_Tracking_PID(void);

// 调试函数
void Debug_IR_Sensors(void);
void Debug_Motor_Test(void);
void Debug_All_GPIOB_Pins(void);

// 串口函数
void UART_SendChar(uint8_t ch);
void UART_SendString(const char *str);
int fputc(int ch, FILE *f);
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
	
	// �???有方向控制引脚设为低电平
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  前进
  * @param  speed: PWM占空�??? (0-9999)
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
	
	// 设置PWM占空�???
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  后�??
  * @param  speed: PWM占空�??? (0-9999)
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
	
	// 设置PWM占空�???
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  左转
  * @param  speed: PWM占空�??? (0-9999)
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
	
	// 设置PWM占空�???
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  右转
  * @param  speed: PWM占空�??? (0-9999)
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
	
	// 设置PWM占空�???
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

/**
  * @brief  差�?�控制（左右电机不同速度�???
  * @param  left_speed: 左电机�?�度 (-9999 ~ 9999，负数为反转)
  * @param  right_speed: 右电机�?�度 (-9999 ~ 9999，负数为反转)
  * @retval None
  */
void Motor_DifferentialSpeed(int16_t left_speed, int16_t right_speed)
{
	// 限幅
	if(left_speed > PWM_SPEED_MAX) left_speed = PWM_SPEED_MAX;
	if(left_speed < -PWM_SPEED_MAX) left_speed = -PWM_SPEED_MAX;
	if(right_speed > PWM_SPEED_MAX) right_speed = PWM_SPEED_MAX;
	if(right_speed < -PWM_SPEED_MAX) right_speed = -PWM_SPEED_MAX;
	
	// 左电机方向控�???
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
	
	// 右电机方向控�???
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
  * @brief  读取红外传感器状�??
  * @retval 传感器状态（5位二进制�??1表示�??测到黑线�??0表示白色�??
  *         bit4: 左外, bit3: 左内, bit2: 中间, bit1: 右内, bit0: 右外
  */
int8_t Read_IR_Sensors(void)
{
	int8_t sensor_value = 0;
	
	// 读取五个红外传感器
	// 左边三个：标准逻辑，黑线=高电平(1)，白色=低电平(0)
	
	if(HAL_GPIO_ReadPin(GPIOB, LEFT1_Pin) == GPIO_PIN_SET)  // 左外 PB0
		sensor_value |= 0x10;
	
	if(HAL_GPIO_ReadPin(GPIOB, LEFT2_Pin) == GPIO_PIN_SET)  // 左内 PB1
		sensor_value |= 0x08;
	
	if(HAL_GPIO_ReadPin(GPIOB, MID_Pin) == GPIO_PIN_SET)    // 中间 PB3
		sensor_value |= 0x04;
	
	// 右边两个：传感器输出异常（低电平1.7V被识别为高电平）
	// 解决方案：反转逻辑，RESET=黑线，SET=白色
	if(HAL_GPIO_ReadPin(GPIOB, RIGHT2_Pin) == GPIO_PIN_RESET) // 右内 PB4 (反转)
		sensor_value |= 0x02;
	
	if(HAL_GPIO_ReadPin(GPIOB, RIGHT1_Pin) == GPIO_PIN_RESET) // 右外 PB5 (反转)
		sensor_value |= 0x01;
	
	return sensor_value;
}

/**
  * @brief  计算位置误差（加权平均法�??
  * @param  sensor_status: 传感器状态指针，用于返回特殊状�??
  *         0: 正常, 1: 全黑, 2: 全白
  * @retval 位置误差 (-2.0 ~ 2.0，负数表示偏左，正数表示偏右)
  */
float Calculate_Position_Error(uint8_t *sensor_status)
{
	int8_t sensors = Read_IR_Sensors();
	float weighted_sum = 0;
	int8_t sensor_count = 0;
	
	// �???测全黑（�???有传感器都检测到黑线�???
	if(sensors == 0x1F)  // 0b11111
	{
		*sensor_status = 1;  // 全黑
		return 0;  // 返回0误差
	}
	
	// �???测全白（�???有传感器都没�???测到黑线�???
	if(sensors == 0x00)  // 0b00000
	{
		*sensor_status = 2;  // 全白
		return pid.last_error;  // 保持上次误差
	}
	
	*sensor_status = 0;  // 正常状�??
	
	// 计算加权�???
	if(sensors & 0x10) { weighted_sum += WEIGHT_LEFT_OUT; sensor_count++; }  // 左外
	if(sensors & 0x08) { weighted_sum += WEIGHT_LEFT_IN; sensor_count++; }   // 左内
	if(sensors & 0x04) { weighted_sum += WEIGHT_MID; sensor_count++; }       // 中间
	if(sensors & 0x02) { weighted_sum += WEIGHT_RIGHT_IN; sensor_count++; }  // 右内
	if(sensors & 0x01) { weighted_sum += WEIGHT_RIGHT_OUT; sensor_count++; } // 右外
	
	// 如果没有传感器检测到黑线（不应该发生，已在上面处理）
	if(sensor_count == 0)
		return pid.last_error;
	
	// 返回加权平均�???
	return weighted_sum / sensor_count;
}

/**
  * @brief  初始化PID控制�???
  * @param  pid: PID控制器指�???
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
	pid->integral = 0;
	pid->derivative = 0;
	pid->output = 0;
}

/**
  * @brief  PID计算
  * @param  pid: PID控制器指�???
  * @param  error: 当前误差
  * @retval PID输出�???
  */
float PID_Calculate(PID_Controller *pid, float error)
{
	// 保存当前误差
	pid->error = error;
	
	// 计算积分项（带限幅）
	pid->integral += error;
	if(pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
	if(pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;
	
	// 计算微分�???
	pid->derivative = error - pid->last_error;
	
	// PID输出
	pid->output = pid->Kp * pid->error + 
	              pid->Ki * pid->integral + 
	              pid->Kd * pid->derivative;
	
	// 保存误差用于下次计算
	pid->last_error = error;
	
	return pid->output;
}

/**
  * @brief  基于PID的循迹控�???
  * @retval None
  */
void Line_Tracking_PID(void)
{
	uint8_t sensor_status = 0;
	
	// 计算位置误差
	float position_error = Calculate_Position_Error(&sensor_status);
	
	// �???查特殊状�???
	if(sensor_status == 1)  // 全黑：停�???
	{
		Motor_Stop();
		return;
	}
	else if(sensor_status == 2)  // 全白：停�???
	{
		Motor_Stop();
		return;
	}
	
	// 正常循迹：PID计算
	float pid_output = PID_Calculate(&pid, position_error);
	
	// 根据PID输出调整左右电机速度
	int16_t left_speed = BASE_SPEED - (int16_t)pid_output;
	int16_t right_speed = BASE_SPEED + (int16_t)pid_output;
	
	// 差�?�控�???
	Motor_DifferentialSpeed(left_speed, right_speed);
}

/**
  * @brief  红外传感器调试函�???
  * @retval None
  */
void Debug_IR_Sensors(void)
{
	int8_t sensors = Read_IR_Sensors();
	char buffer[120];
	
	// 读取各个传感器状�???
	uint8_t left_out = (sensors & 0x10) ? 1 : 0;   // 左外
	uint8_t left_in  = (sensors & 0x08) ? 1 : 0;   // 左内
	uint8_t mid      = (sensors & 0x04) ? 1 : 0;   // 中间
	uint8_t right_in = (sensors & 0x02) ? 1 : 0;   // 右内
	uint8_t right_out= (sensors & 0x01) ? 1 : 0;   // 右外
	
	// 读取原始GPIO电平（用于调试）
	uint8_t raw_left_out  = HAL_GPIO_ReadPin(GPIOB, LEFT1_Pin);
	uint8_t raw_left_in   = HAL_GPIO_ReadPin(GPIOB, LEFT2_Pin);
	uint8_t raw_mid       = HAL_GPIO_ReadPin(GPIOB, MID_Pin);
	uint8_t raw_right_in  = HAL_GPIO_ReadPin(GPIOB, RIGHT2_Pin);
	uint8_t raw_right_out = HAL_GPIO_ReadPin(GPIOB, RIGHT1_Pin);
	
	// 使用sprintf格式化字符串（不依赖printf重定向）
	sprintf(buffer, "IR: [%d][%d][%d][%d][%d] Raw:0x%02X\r\n", 
	        left_out, left_in, mid, right_in, right_out, sensors);
	UART_SendString(buffer);
	
	// 输出原始GPIO电平（用于判断传感器极�?�）
	sprintf(buffer, "GPIO: [%d][%d][%d][%d][%d]\r\n", 
	        raw_left_out, raw_left_in, raw_mid, raw_right_in, raw_right_out);
	UART_SendString(buffer);
	
	// 判断特殊状�??
	if(sensors == 0x1F)
		UART_SendString(">>> ALL BLACK <<<\r\n");
	else if(sensors == 0x00)
		UART_SendString(">>> ALL WHITE <<<\r\n");
	
	UART_SendString("\r\n");  // 空行分隔
	HAL_Delay(300);  // �???300ms输出�???�???
}

/**
  * @brief  测试�??有GPIOB引脚，找出实际连接的传感器引�??
  * @retval None
  */
void Debug_All_GPIOB_Pins(void)
{
	char buffer[150];
	
	// 读取GPIOB�??有引脚的状�??
	uint16_t gpiob_idr = GPIOB->IDR;
	
	sprintf(buffer, "GPIOB All Pins (0-15):\r\n");
	UART_SendString(buffer);
	
	sprintf(buffer, "PB0=%d PB1=%d PB2=%d PB3=%d PB4=%d PB5=%d PB6=%d PB7=%d\r\n",
	        (gpiob_idr & 0x0001) ? 1 : 0,
	        (gpiob_idr & 0x0002) ? 1 : 0,
	        (gpiob_idr & 0x0004) ? 1 : 0,
	        (gpiob_idr & 0x0008) ? 1 : 0,
	        (gpiob_idr & 0x0010) ? 1 : 0,
	        (gpiob_idr & 0x0020) ? 1 : 0,
	        (gpiob_idr & 0x0040) ? 1 : 0,
	        (gpiob_idr & 0x0080) ? 1 : 0);
	UART_SendString(buffer);
	
	sprintf(buffer, "PB8=%d PB9=%d PB10=%d PB11=%d PB12=%d PB13=%d PB14=%d PB15=%d\r\n",
	        (gpiob_idr & 0x0100) ? 1 : 0,
	        (gpiob_idr & 0x0200) ? 1 : 0,
	        (gpiob_idr & 0x0400) ? 1 : 0,
	        (gpiob_idr & 0x0800) ? 1 : 0,
	        (gpiob_idr & 0x1000) ? 1 : 0,
	        (gpiob_idr & 0x2000) ? 1 : 0,
	        (gpiob_idr & 0x4000) ? 1 : 0,
	        (gpiob_idr & 0x8000) ? 1 : 0);
	UART_SendString(buffer);
	
	sprintf(buffer, "IDR Raw: 0x%04X\r\n\r\n", gpiob_idr);
	UART_SendString(buffer);
	
	HAL_Delay(500);
}

/**
  * @brief  电机测试函数
  * @retval None
  */
void Debug_Motor_Test(void)
{
	UART_SendString("Motor Test: Forward\r\n");
	Motor_Forward(PWM_SPEED_MID);
	HAL_Delay(2000);
	
	UART_SendString("Motor Test: Stop\r\n");
	Motor_Stop();
	HAL_Delay(1000);
	
	UART_SendString("Motor Test: Backward\r\n");
	Motor_Backward(PWM_SPEED_MID);
	HAL_Delay(2000);
	
	UART_SendString("Motor Test: Stop\r\n");
	Motor_Stop();
	HAL_Delay(1000);
	
	UART_SendString("Motor Test: Turn Left\r\n");
	Motor_TurnLeft(PWM_SPEED_LOW);
	HAL_Delay(1000);
	
	UART_SendString("Motor Test: Stop\r\n");
	Motor_Stop();
	HAL_Delay(1000);
	
	UART_SendString("Motor Test: Turn Right\r\n");
	Motor_TurnRight(PWM_SPEED_LOW);
	HAL_Delay(1000);
	
	UART_SendString("Motor Test: Stop\r\n");
	Motor_Stop();
	HAL_Delay(2000);
}

/**
  * @brief  串口发�?�单个字符（底层方法，用于测试）
  * @param  ch: 要发送的字符
  * @retval None
  */
void UART_SendChar(uint8_t ch)
{
	// 等待发�?�缓冲区为空
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
	// 发�?�数�???
	huart1.Instance->DR = ch;
	// 等待发�?�完�???
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
}

/**
  * @brief  串口发�?�字符串（不使用printf�???
  * @param  str: 要发送的字符�???
  * @retval None
  */
void UART_SendString(const char *str)
{
	// 方法1：使用HAL库函�???
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);
	
	// 方法2：�?�字符发送（如果HAL库有问题，可以用这个�???
	/*
	while(*str)
	{
		UART_SendChar(*str++);
	}
	*/
}

/**
  * @brief  串口重定向（用于printf�???
  * @retval 字符
  */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return ch;
}

#ifdef __GNUC__
// 对于GCC编译器，�???要重定向_write函数
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 1000);
	return len;
}
#endif

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
		HAL_Delay(20);
 OLED_Init();
  /* USER CODE BEGIN 2 */
		HAL_Delay(20);
 OLED_Init();
	// 禁用JTAG，释放PB3、PB4引脚作为普通GPIO
	// 保留SWD调试功能（PA13/PA14）
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
	
	// 等待串口稳定
	HAL_Delay(100);
	
	// �???�???单的测试：发送单个字�???
	UART_SendChar('A');
	UART_SendChar('B');
	UART_SendChar('C');
	UART_SendChar('\r');
	UART_SendChar('\n');
	
	// 启动PWM输出
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	// 初始化PID控制�???
	PID_Init(&pid, KP, KI, KD);
	
	// 初始化电机为停止状�??
	Motor_Stop();
	
	// 测试串口字符串发�??
	UART_SendString("\r\n=== STM32 Smart Car ===\r\n");
	UART_SendString("UART Test OK!\r\n");
	
	// 测试GPIO寄存器配�??
	char test_buffer[100];
	sprintf(test_buffer, "GPIOB CRL: 0x%08lX\r\n", GPIOB->CRL);
	UART_SendString(test_buffer);
	sprintf(test_buffer, "GPIOB IDR: 0x%04X\r\n", GPIOB->IDR);
	UART_SendString(test_buffer);
	UART_SendString("\r\n");
	
	// 根据调试模式输出提示信息
	#if DEBUG_ALL_PINS
		UART_SendString("\r\n=== Test All GPIOB Pins ===\r\n");
		UART_SendString("Move sensors to find which pins change!\r\n\r\n");
		HAL_Delay(1000);
	#elif DEBUG_IR_SENSOR
		UART_SendString("\r\n=== IR Sensor Debug Mode ===\r\n");
		UART_SendString("Sensor Order: [L-Out][L-In][Mid][R-In][R-Out]\r\n");
		UART_SendString("1=Black Line, 0=White\r\n\r\n");
		HAL_Delay(1000);
	#elif DEBUG_MOTOR
		UART_SendString("\r\n=== Motor Debug Mode ===\r\n");
		UART_SendString("Test: Forward->Back->Left->Right\r\n\r\n");
		HAL_Delay(2000);
	#elif RUN_LINE_TRACK
		UART_SendString("\r\n=== Line Tracking Mode ===\r\n");
		UART_SendString("Starting PID Line Tracking...\r\n\r\n");
		HAL_Delay(2000);
	#else
		UART_SendString("\r\n!!! Please Select Debug Mode !!!\r\n");
	#endif
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     OLED_NewFrame();
	OLED_PrintString(1,1, "前进 运动 快数量：     倒计时：", 	&font16x16,0);
		OLED_PrintASCIIString(48,16,"1",&afont16x8,0);
		for(uint8_t i=0;i<=15;i++){
			OLED_PrintASCIIString(64,33,"15",&afont16x8,0);
    	OLED_ShowFrame();

		}
		OLED_ShowFrame();
    HAL_Delay(20);

		/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		#if DEBUG_ALL_PINS
			// 测试�??有GPIOB引脚
			Debug_All_GPIOB_Pins();
			
		#elif DEBUG_IR_SENSOR
			// 红外传感器调试模�??
			Debug_IR_Sensors();
			
		#elif DEBUG_MOTOR
			// 电机调试模式
			Debug_Motor_Test();
			
		#elif RUN_LINE_TRACK
			// PID循迹控制（实时执行）
			Line_Tracking_PID();
			// 不添加延时，保持连续控制，避免顿�??
			
		#else
			// 默认：停止状�??
			Motor_Stop();
			HAL_Delay(1000);
			UART_SendString("Please Select Debug Mode!\r\n");
		#endif
		
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
