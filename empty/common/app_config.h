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

/* 外设测试任务：驱动 OLED 软件 I2C、LED2/LED3、蜂鸣器；OLED 刷屏占栈，放大到 3 倍。 */
#define APP_PERIPH_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 3U)
#define APP_PERIPH_TEST_TASK_PRIORITY    (1U)

/* PB22 心跳灯翻转周期。 */
#define APP_LED1_PERIOD_TICKS           pdMS_TO_TICKS(300U)

/* UART0 串口测试信息发送周期，历史发送测试保留。 */
#define APP_UART_TEST_PERIOD_TICKS      pdMS_TO_TICKS(1000U)

/* UART0 接收命令轮询周期。 */
#define APP_UART_RX_POLL_PERIOD_TICKS   pdMS_TO_TICKS(10U)

/* IMU 对外读取周期：10ms，即 100Hz。 */
#define APP_IMU_UART_PERIOD_TICKS       pdMS_TO_TICKS(10U)

/*
 * IMU + 激光测距1 串口打印节流：每 N 个读取周期打印一整行。
 * 读取保持 100Hz（持续排空 IMU FIFO）；整行含欧拉角+三轴加速度+三轴角速度+激光测距约 120 字节。
 * 打印降到 100Hz/N=5Hz（N=20），刷新不快、便于在串口助手里阅读，同时减轻单串口负载。
 * 若想更慢，把 N 调大（如 50→2Hz）。
 */
#define APP_IMU_PRINT_DIVIDER           (20U)

/* 电机1按键扫描周期：20ms，兼作简单去抖。 */
#define APP_MOTOR_KEY_POLL_TICKS        pdMS_TO_TICKS(20U)

/* 外设测试任务刷新周期：500ms。OLED 全屏软件 I2C 刷屏约几十 ms，周期不宜过短。 */
#define APP_PERIPH_TEST_PERIOD_TICKS    pdMS_TO_TICKS(500U)

/* 舵机1 测试任务：20ms 周期，逐 us 调整脉宽实现慢速平滑摆动。 */
#define APP_SERVO_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 2U)
#define APP_SERVO_TEST_TASK_PRIORITY    (1U)
#define APP_SERVO_TEST_PERIOD_TICKS     pdMS_TO_TICKS(20U)

/*
 * IMU I2C 引脚物理测试开关。
 * 置 1 后 IMU 任务不初始化传感器，而是用开漏模拟方式低频翻转 PB2/PB3，
 * 并直接读取 GPIO 输入寄存器回报实际电平，用于排查 MCU 端口和外部拉低问题。
 */
#define APP_IMU_I2C_PIN_TEST_ENABLE     (0U)
#define APP_IMU_I2C_PIN_TEST_PERIOD_TICKS \
                                        pdMS_TO_TICKS(500U)

#endif
