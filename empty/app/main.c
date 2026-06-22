/*
 * MSPM0G3507 FreeRTOS 工程入口。
 */

#include "FreeRTOS.h"
#include "task.h"

#include "app_main.h"
#include "bsp_board.h"
#include "bsp_led.h"
#include "bsp_uart.h"

int main(void)
{
    BspBoard_Init();

    /*
     * 最早期启动检查点：此处还没有进入 OLED 初始化和 FreeRTOS 调度。
     * 若 PB22 被点亮但串口无输出，优先排查 UART0 接线、波特率和串口助手。
     */
    BspLed_On(BSP_LED_1);
    BspUart0_SendString("BOOT: board init ok\r\n");

    App_Init();

    /*
     * 若能看到上一条 BOOT 但看不到本条，说明可能卡在 App_Init 内部，
     * 重点检查 OLED 初始化、任务创建断言或新增模块初始化。
     */
    BspUart0_SendString("BOOT: start scheduler\r\n");

    vTaskStartScheduler();

    /* 调度器正常不会返回，返回通常表示堆空间不足或配置异常。 */
    for (;;) {
        BspLed_Toggle(BSP_LED_1);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
