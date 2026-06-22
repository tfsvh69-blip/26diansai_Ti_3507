#include "app_uart_test_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_uart.h"

static TaskHandle_t s_uartTestTaskHandle = NULL;

static void AppUartTestTask_Entry(void *argument)
{
    uint8_t rxByte;

    (void)argument;

    BspUart0_Lock();
    BspUart0_SendString("UART0 RX READY, PB22 heartbeat active\r\n");
    BspUart0_Unlock();

    for (;;) {
        /*
         * PB22 已作为系统心跳灯，串口任务不再控制 LED。
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
