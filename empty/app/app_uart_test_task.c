#include "app_uart_test_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_uart.h"

/*
 * UART0 接收自检任务：每 10ms 轮询 UART0 RX，收到任意非换行字符回一句 "UART RX OK"。
 * 用途单一——验证调试串口的收发链路通不通，不承担业务命令解析。
 * 与 IMU 任务共享 UART0，靠 bsp_uart 的递归互斥量保证各自整行不被打断。
 */

static TaskHandle_t s_uartTestTaskHandle = NULL;

static void AppUartTestTask_Entry(void *argument)
{
    uint8_t rxByte;

    (void)argument;

    BspUart0_Lock();
    BspUart0_SendString("UART0 RX READY, LED1 heartbeat active\r\n");
    BspUart0_Unlock();

    for (;;) {
        /*
         * LED1(PB25) 已作为系统心跳灯，串口任务不再控制 LED。
         * 收到任意非换行字符后返回确认，便于单独验证 UART0 RX/TX 是否正常。
         */
        while (BspUart0_ReadByte(&rxByte)) {
            if ((rxByte != '\r') && (rxByte != '\n')) {
                BspUart0_Lock();
                BspUart0_SendString("UART RX OK\r\n");
                BspUart0_Unlock();
            }
        }

        vTaskDelay(APP_UART_RX_POLL_PERIOD_TICKS);
    }
}

void AppUartTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppUartTestTask_Entry,
                      "UART0TX",
                      APP_UART_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_UART_TEST_TASK_PRIORITY,
                      &s_uartTestTaskHandle);
    configASSERT(ret == pdPASS);
}
