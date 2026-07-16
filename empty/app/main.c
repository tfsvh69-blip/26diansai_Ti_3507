/*
 * MSPM0G3507 FreeRTOS 工程入口。
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_main.h"
#include "bsp_board.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    BspBoard_Init();

    /*
     * 最早期启动检查点：此处还没有创建任务、未进入 FreeRTOS 调度。
     * 若 LED1(PB25) 被点亮但串口无输出，优先排查 UART0 接线、波特率和串口助手。
     *
     * 注：早期用于 LED3/舵机引脚排查的两段裸机诊断（约 5 秒忙等）已删除，
     * 现在上电即进入调度器，电机等任务立即启动，不再有上电等待。
     */
    BspLed_On(BSP_LED_1);
    BspUart0_SendString("BOOT: board init ok\r\n");

    /*
     * 报告系统时钟参考源：确认 40MHz 外部晶振是否成功起振。
     * HFXT OK 表示 MCLK 80MHz 由外部晶振锁定；否则说明晶振未起振，已自动回退内部 SYSOSC。
     */
    if (g_sysClockUsingHFXT) {
        BspUart0_SendString("BOOT: MCLK 80MHz <- HFXT 40MHz OK\r\n");
    } else {
        BspUart0_SendString("BOOT: HFXT FAIL, MCLK 80MHz <- internal SYSOSC\r\n");
    }

    App_Init();

    /*
     * 若能看到上一条 BOOT 但看不到本条，说明可能卡在 App_Init 内部，
     * 重点检查任务创建断言或新增模块初始化。
     */
    BspUart0_SendString("BOOT: start scheduler\r\n");

    vTaskStartScheduler();

    /* 调度器正常不会返回，返回通常表示堆空间不足或配置异常。 */
    for (;;) {
        BspLed_Toggle(BSP_LED_1);
    }
}

/*
 * 故障可视化辅助（在"关中断"上下文中调用）：
 * 不能用会加锁的 BspUart0_SendString（互斥量在关中断时无法阻塞/切换会死锁），
 * 只用不加锁的 BspUart0_SendByte 直接轮询发送。
 */
static void App_FaultPuts(const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
        BspUart0_SendByte((uint8_t)*s);
        s++;
    }
}

/* 关中断后快闪 LED1 表示系统已挂（配合上面打印的原因定位），永不返回。 */
static void App_FaultBlink(void)
{
    for (;;) {
        BspLed_Toggle(BSP_LED_1);
        for (volatile uint32_t i = 0U; i < 2000000U; i++) {
            /* 关中断下的粗略忙等 (~0.1s @80MHz)，让 LED1 明显快闪。 */
        }
    }
}

/* FreeRTOS configASSERT 失败入口（见 FreeRTOSConfig.h）。 */
void vAssertCalled(const char *file, unsigned long line)
{
    (void)file;
    taskDISABLE_INTERRUPTS();
    App_FaultPuts("\r\nFATAL: assert failed at line ");
    BspUart0_SendUint((uint32_t)line);
    App_FaultPuts("\r\n");
    App_FaultBlink();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    taskDISABLE_INTERRUPTS();
    App_FaultPuts("\r\nFATAL: stack overflow in task: ");
    App_FaultPuts((pcTaskName != NULL) ? pcTaskName : "?");
    App_FaultPuts("\r\n");
    App_FaultBlink();
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    App_FaultPuts("\r\nFATAL: malloc failed (heap exhausted)\r\n");
    App_FaultBlink();
}
