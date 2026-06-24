#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "FreeRTOS.h"

/* 任务栈单位为 word，不是 byte。 */
#define APP_LED_TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE)
#define APP_LED_TASK_PRIORITY           (1U)

/* UART0 测试发送任务只发送短字符串，栈保持最小配置。 */
#define APP_UART_TEST_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE)
#define APP_UART_TEST_TASK_PRIORITY     (1U)

/* IMU 姿态读取包含 I2C、FIFO 解析和三角函数计算，栈空间单独放大。 */
#define APP_IMU_UART_TASK_STACK_WORDS   (configMINIMAL_STACK_SIZE * 4U)
#define APP_IMU_UART_TASK_PRIORITY      (1U)

/* 电机1按键控制任务：轮询按键并触发定长旋转，含串口输出，栈适当放大。 */
#define APP_MOTOR_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 2U)
#define APP_MOTOR_TEST_TASK_PRIORITY    (1U)

/* PB22 心跳灯翻转周期。 */
#define APP_LED1_PERIOD_TICKS           pdMS_TO_TICKS(300U)

/* UART0 串口测试信息发送周期，历史发送测试保留。 */
#define APP_UART_TEST_PERIOD_TICKS      pdMS_TO_TICKS(1000U)

/* UART0 接收命令轮询周期。 */
#define APP_UART_RX_POLL_PERIOD_TICKS   pdMS_TO_TICKS(10U)

/* IMU 对外读取/串口输出周期：10ms，即 100Hz。 */
#define APP_IMU_UART_PERIOD_TICKS       pdMS_TO_TICKS(10U)

/* 电机1按键扫描周期：20ms，兼作简单去抖。 */
#define APP_MOTOR_KEY_POLL_TICKS        pdMS_TO_TICKS(20U)

/*
 * IMU I2C 引脚物理测试开关。
 * 置 1 后 IMU 任务不初始化传感器，而是用开漏模拟方式低频翻转 PB2/PB3，
 * 并直接读取 GPIO 输入寄存器回报实际电平，用于排查 MCU 端口和外部拉低问题。
 */
#define APP_IMU_I2C_PIN_TEST_ENABLE     (0U)
#define APP_IMU_I2C_PIN_TEST_PERIOD_TICKS \
                                        pdMS_TO_TICKS(500U)

#endif
