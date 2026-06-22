#include "app_main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_imu_uart_task.h"
#include "app_led_task.h"
#include "app_uart_test_task.h"
#include "OLED.h"

static void App_ShowBootScreen(void)
{
    /*
     * OLED 当前使用 GPIO 模拟 I2C。
     * SCL 接 PB9，SDA 接 PB8；上电显示接线信息便于直接排查。
     */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "MSPM0G3507", OLED_8X16);
    OLED_ShowString(0, 16, "FreeRTOS OK", OLED_8X16);
    OLED_ShowString(0, 32, "IMU SWI2C 0x6A", OLED_6X8);
    OLED_ShowString(0, 48, "PB22 Heartbeat", OLED_6X8);
    OLED_Update();
}

void App_Init(void)
{
    App_ShowBootScreen();

    /*
     * PB22 作为系统心跳灯，烧录后若能持续闪烁，说明 FreeRTOS 调度已运行。
     * UART0 命令仍可返回文本，但不要再用 PB22 常亮/熄灭判断串口命令状态。
     */
    AppLedTask_Init();
    AppUartTestTask_Init();
    AppImuUartTask_Init();
}
