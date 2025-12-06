# STM32F103 Smart Car Project

基于STM32F103C8的智能小车项目，具有循迹、避障等功能。

## 项目简介

stm32f103_car_v1.0智能小车，连接了一个L298N电机驱动、两个电机、一个超声波模块、五个红外循迹传感器、一块四线OLED屏幕，拥有超声波避障、循迹寻线、OLED显示等功能。

## 硬件配置

### 主控芯片
- STM32F103C8T6

### 引脚连接

#### 电机驱动 (L298N)
- PA0 (TIM2_CH1) → PWM输入端INA (左电机PWM)
- PA3 (TIM2_CH4) → PWM输入端INB (右电机PWM)
- PA1 → IN1 (左电机方向控制)
- PA2 → IN2 (左电机方向控制)
- PA4 → IN3 (右电机方向控制)
- PA5 → IN4 (右电机方向控制)

#### 红外循迹传感器
- PB0 → LEFT1 (左外传感器)
- PB1 → LEFT2 (左内传感器)
- PB3 → MID (中间传感器)
- PB4 → RIGHT2 (右内传感器) *注：需要反转逻辑
- PB5 → RIGHT1 (右外传感器) *注：需要反转逻辑

#### 超声波模块
- PB14 → TRIG (触发信号)
- PB15 → ECHO (回响信号)

#### OLED屏幕 (I2C)
- PB6 → SCL (I2C时钟)
- PB7 → SDA (I2C数据)

#### 串口调试 (USART1)
- PA9 → TX
- PA10 → RX
- 波特率：115200

## 功能特性

### 1. 红外循迹
- 5路红外传感器
- PID算法控制
- 可调节PID参数

### 2. 超声波避障
- HC-SR04超声波模块
- 实时距离检测

### 3. OLED显示
- 128x64 OLED屏幕
- I2C接口

### 4. 串口调试
- 实时传感器数据输出
- 多种调试模式

## 调试模式

在 `Core/Src/main.c` 中可以切换不同的调试模式：

```c
#define DEBUG_ALL_PINS   0      // 测试所有GPIOB引脚
#define DEBUG_IR_SENSOR  1      // 红外传感器调试模式
#define DEBUG_MOTOR      0      // 电机调试模式
#define RUN_LINE_TRACK   0      // PID循迹运行模式
```

### 调试模式说明

1. **DEBUG_IR_SENSOR** - 红外传感器调试
   - 实时显示5个传感器状态
   - 检测全黑/全白状态

2. **DEBUG_MOTOR** - 电机测试
   - 循环测试前进、后退、左转、右转

3. **RUN_LINE_TRACK** - PID循迹
   - 自动循迹功能
   - 全黑/全白自动停止

## PID参数

```c
#define KP  80.0f           // 比例系数
#define KI  0.0f            // 积分系数
#define KD  20.0f           // 微分系数
#define BASE_SPEED  4500    // 基础速度 (45%)
```

### 参数调整建议
- 震荡严重：减小KP或增大KD
- 响应慢：增大KP
- 有稳态误差：适当增加KI (0.1-1.0)
- 速度太快：降低BASE_SPEED

## 开发环境

- IDE: Keil MDK-ARM (Keil5)
- 配置工具: STM32CubeMX
- 调试工具: ST-Link / USB转TTL

## 编译和烧录

1. 打开 `MDK-ARM/test1119.uvprojx`
2. 编译项目 (F7)
3. 使用ST-Link烧录到STM32

## 串口调试

1. 连接USB转TTL到PA9(TX)和PA10(RX)
2. 打开串口助手
3. 设置波特率：115200
4. 观察调试输出

## 注意事项

1. **JTAG禁用**：代码中已禁用JTAG以释放PB3、PB4引脚，保留SWD调试功能
2. **右路传感器**：RIGHT2和RIGHT1使用反转逻辑（硬件特性）
3. **电源要求**：确保电机和STM32分别供电，共地

## 作者

- 邮箱: weiyongxin2004@163.com

## 许可证

MIT License

## 更新日志

### v1.0 (2025-12-03)
- 初始版本
- 实现红外循迹功能
- 实现PID控制算法
- 添加多种调试模式
- 支持串口调试输出
