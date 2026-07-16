#include "app_main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_imu_uart_task.h"
#include "app_led_task.h"
#include "app_motor_test_task.h"
#include "app_periph_test_task.h"
#include "app_servo_test_task.h"
#include "app_uart_test_task.h"
#include "bsp_uart.h"
#include "laser_ld14.h"

/*
 * v1.1 板 OLED 恢复到板载 PB8/PB9（软件 I2C），TMC 细分改用 PB0/PB1，两者不再冲突。
 * OLED 调试显示、LED2/LED3 与蜂鸣器测试统一放在外设测试任务中。
 */

void App_Init(void)
{
    /*
     * LED1(PB25) 作为系统心跳灯，烧录后若能持续闪烁，说明 FreeRTOS 调度已运行。
     * UART0 命令仍可返回文本，但不要再用 LED1 常亮/熄灭判断串口命令状态。
     */
    AppLedTask_Init();
    AppUartTestTask_Init();
    AppPeriphTestTask_Init();
    AppServoTestTask_Init();

    /*
     * 【全任务并行，2026-07-16】LED+串口+OLED+蜂鸣器+舵机 已验证正常。
     * 本阶段放开电机1（四按键定圈旋转）与 IMU（陀螺仪姿态串口打印）。
     * UART0 已改用递归互斥量替代挂起调度器，多任务共享串口不再造成全局卡顿。
     */
    AppMotorTestTask_Init();

    /*
     * 激光测距1（UART2/PB15/PB16，230400 8N1）：
     * 接收走 UART2 RX 中断，逐字节喂给 LaserLd14 解析器（无独立任务）；
     * 解析出的距离由 IMU 任务在打印整行时一并输出（见 app_imu_uart_task.c）。
     * 先复位解析器，再注册回调并放开中断。
     */
    LaserLd14_Reset();
    BspUart2_Init(LaserLd14_FeedByte);

    AppImuUartTask_Init();
}
