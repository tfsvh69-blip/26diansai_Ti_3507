#include "bsp_uart.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"

void BspUart0_Lock(void)
{
    /*
     * UART0 当前只作为调试串口使用。
     * 任务内拼接多段日志时临时挂起调度器，保证整行日志不被其他任务插入。
     */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskSuspendAll();
    }
}

void BspUart0_Unlock(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) {
        (void)xTaskResumeAll();
    }
}

void BspUart0_SendByte(uint8_t byte)
{
    /*
     * 测试串口发送量很小，直接阻塞等待 TX FIFO 空位即可。
     * 不要在中断或高频控制环里调用该阻塞发送接口。
     */
    DL_UART_Main_transmitDataBlocking(UART_0_INST, byte);
}

void BspUart0_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }

    while (*str != '\0') {
        BspUart0_SendByte((uint8_t)*str);
        str++;
    }
}

bool BspUart0_ReadByte(uint8_t *byte)
{
    if (byte == NULL) {
        return false;
    }

    /*
     * 接收侧使用轮询方式读取 RX FIFO。
     * 当前数据量很小，不启用中断或 DMA，避免引入额外同步复杂度。
     */
    return DL_UART_Main_receiveDataCheck(UART_0_INST, byte);
}
