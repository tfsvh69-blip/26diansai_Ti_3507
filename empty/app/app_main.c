#include "app_main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_imu_uart_task.h"
#include "app_led_task.h"
#include "app_motor_test_task.h"
#include "app_uart_test_task.h"

/*
 * 注意：天猛星扩展板 v1.0 把原 OLED 的 PB8/PB9 改作 TMC 细分 MS1/MS2，
 * 本板不再驱动 OLED，故移除 OLED 启动屏，避免与电机细分引脚冲突。
 */

void App_Init(void)
{
    /*
     * PB22 作为系统心跳灯，烧录后若能持续闪烁，说明 FreeRTOS 调度已运行。
     * UART0 命令仍可返回文本，但不要再用 PB22 常亮/熄灭判断串口命令状态。
     */
    AppLedTask_Init();
    AppUartTestTask_Init();
    AppImuUartTask_Init();
    AppMotorTestTask_Init();
}
