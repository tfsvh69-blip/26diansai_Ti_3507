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
 * ============================ 应用层顶层编排 ============================
 *
 * App_Init() 是整个系统的"总装线"：硬件已由 BspBoard_Init() 就绪后，
 * 这里把各功能拆成独立的 FreeRTOS 任务分别创建，任务之间不互相调用，
 * 各自按周期跑，靠 BSP/模块层的线程安全接口共享硬件。启动关系：
 *
 *   main() → BspBoard_Init()(裸机初始化外设) → App_Init()(建任务) → vTaskStartScheduler()
 *
 * 【任务清单】（周期/优先级/栈集中在 common/app_config.h）
 *   LED1      心跳灯，300ms 翻转 PB25，用来一眼确认调度器活着
 *   UART0TX   UART0 接收回显（调试串口自检）
 *   PERIPH    500ms 刷 OLED + LED2/LED3 心跳 + 蜂鸣器（外设总验证 + 电机状态显示）
 *   SERVOSWEEP 4 个舵机各自独立错相摆动（800↔2200us，不用按键，演示独立控制）
 *   MOTORTEST KEY1/KEY2 让 4 个电机一起正/反转 2 圈（测试四路电机，梯形加减速）
 *   IMU100Hz  100Hz 读六轴姿态，5Hz 把"姿态 + 激光测距1"整行发到 UART0
 *
 * 【中断驱动（非任务）】
 *   激光测距1：UART2 RX 中断逐字节喂 module/laser 解析器，见下方注释。
 *   电机 STEP：TIMG0_IRQHandler 做梯形斜坡（bsp_motor.c）。
 *
 * 【数据流】
 *   IMU(软件I2C) ─┐
 *                 ├─► IMU100Hz 任务拼成一整行 ─► UART0(递归锁保证整行原子) ─► 串口助手
 *   激光(UART2中断)┘   （激光距离经 module/laser 全局状态，由 IMU 任务读取后追加到行尾）
 *
 * 设计约定：任务间不共享业务全局变量；跨层只通过 bsp 层、module 层的接口访问。
 * OLED 只读电机诊断快照 g_motorDiag；激光只读 module/laser 的解析结果。
 *
 * 硬件备注：v1.1 板 OLED 在板载 PB8/PB9（软件 I2C），TMC 细分改用 PB0/PB1，两者不再冲突。
 * ======================================================================
 */

void App_Init(void)
{
    /* 心跳灯：每 300ms 翻转 LED1(PB25)，一眼确认 FreeRTOS 调度在跑。 */
    AppLedTask_Init();

    /* UART0 接收自检：收到非换行字符回 "UART RX OK"，验证调试串口收发。 */
    AppUartTestTask_Init();

    /* 外设综合验证：每 500ms 刷 OLED + LED2/LED3 心跳 + 蜂鸣器，并显示电机测试状态。 */
    AppPeriphTestTask_Init();

    /* 舵机测试：4 个舵机各自独立错相摆动(800↔2200us，不用按键)，演示四路可完全独立控制。 */
    AppServoTestTask_Init();

    /* 电机测试：KEY1/KEY2 让 4 个电机一起正/反转 2 圈，验证四路步进电机是否都正常。 */
    AppMotorTestTask_Init();

    /*
     * 激光测距1（UART2/PB15/PB16，230400 8N1）：不是任务，走 UART2 RX 中断，
     * 逐字节喂给 LaserLd14 解析器；解析出的距离由 IMU 任务在打印整行时一并输出。
     * 先复位解析器，再注册回调并放开中断。
     */
    LaserLd14_Reset();
    BspUart2_Init(LaserLd14_FeedByte);

    /* IMU 姿态：100Hz 读六轴，5Hz 把"姿态 + 激光测距1"整行发到 UART0。 */
    AppImuUartTask_Init();
}
