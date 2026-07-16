#include "bsp_uart.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "ti_msp_dl_config.h"

/*
 * UART0 互斥量：递归类型。
 *
 * 旧实现用 vTaskSuspendAll 挂起整个调度器来保证整行日志不被打断，
 * 但一整行日志（IMU 约 110 字节）在 115200 下发送需约 10ms，
 * 期间所有任务都被冻结，会造成按键/舵机/OLED 明显卡顿。
 *
 * 改为递归互斥量后：
 *   - 发送方仅独占 UART0，不再冻结调度器；其余任务照常按时间片轮转，从根上消除全局卡顿。
 *   - 递归类型允许同一任务在 Lock() 内再调用会自动加锁的 SendString()（嵌套 take/give 计数），不会自锁。
 *   - 只有其它想写 UART0 的任务才需要等待，保证整行日志原子输出。
 */
static SemaphoreHandle_t s_uartMutex = NULL;

/* 仅在调度器已运行且互斥量已创建时才真正加/解锁；启动早期为单线程，直接跳过。 */
static void BspUart0_Take(void)
{
    if ((s_uartMutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        (void)xSemaphoreTakeRecursive(s_uartMutex, portMAX_DELAY);
    }
}

static void BspUart0_Give(void)
{
    if ((s_uartMutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        (void)xSemaphoreGiveRecursive(s_uartMutex);
    }
}

void BspUart0_Init(void)
{
    /*
     * 必须在 vTaskStartScheduler() 之前、任一任务打印之前创建互斥量。
     * heap_4 在调度器启动前即可正常分配。
     */
    if (s_uartMutex == NULL) {
        s_uartMutex = xSemaphoreCreateRecursiveMutex();
        configASSERT(s_uartMutex != NULL);
    }
}

void BspUart0_Lock(void)
{
    /* 任务内拼接多段日志时先加锁，保证整行不被其它任务插入。 */
    BspUart0_Take();
}

void BspUart0_Unlock(void)
{
    BspUart0_Give();
}

void BspUart0_SendByte(uint8_t byte)
{
    /*
     * 测试串口发送量很小，直接阻塞等待 TX FIFO 空位即可。
     * 不要在中断或高频控制环里调用该阻塞发送接口。
     * 单字节本身即原子，不再单独加锁（多段拼接由调用方 Lock/Unlock 包裹）。
     */
    DL_UART_Main_transmitDataBlocking(UART_0_INST, byte);
}

void BspUart0_SendUint(uint32_t value)
{
    char buf[10];   /* uint32 十进制最多 10 位 */
    uint8_t idx = 0U;

    /*
     * 不加锁：单字节发送本身原子，由调用方决定是否用 Lock/Unlock 包裹整行；
     * 关中断的故障处理也能安全调用（不涉及互斥量）。合并了原先散在各任务里的多份同款实现。
     */
    if (value == 0U) {
        BspUart0_SendByte((uint8_t)'0');
        return;
    }
    while ((value > 0U) && (idx < sizeof(buf))) {
        buf[idx++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (idx > 0U) {
        BspUart0_SendByte((uint8_t)buf[--idx]);
    }
}

void BspUart0_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }

    /* 整个字符串自动加锁输出，单次调用即原子；配合递归锁可安全嵌套在 Lock() 内。 */
    BspUart0_Take();
    while (*str != '\0') {
        BspUart0_SendByte((uint8_t)*str);
        str++;
    }
    BspUart0_Give();
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

/* UART2 接收字节回调（激光测距解析器由 app 层注册）。 */
static BspUart2RxHandler_t s_uart2RxHandler = NULL;

void BspUart2_Init(BspUart2RxHandler_t handler)
{
    /* 先登记回调，再放开中断，避免中断先于回调就绪时丢字节（NULL 已在 ISR 内防护）。 */
    s_uart2RxHandler = handler;

    /* 同时使能 RX 与溢出错误中断：溢出时也能进 ISR 取空 FIFO 并清标志。 */
    DL_UART_Main_enableInterrupt(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_2_INST_IRQn);
}

/*
 * UART2 中断服务函数（激光测距1，230400 8N1）。
 * 每次中断把 RX FIFO 里的所有字节全部取空并逐个喂给解析回调，
 * 兼顾按字节触发(阈值1)与突发到达，避免尾字节滞留。
 * 符合 CLAUDE.md：ISR 内只做快速处理，不调用非 FromISR 的 FreeRTOS API。
 */
void UART2_IRQHandler(void)
{
    uint8_t byte;

    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            /*
             * RX 阈值到达或发生溢出都在此取空 FIFO。
             * 230400 连续流下若 ISR 被短暂拖延、4 字节 FIFO 溢出，
             * 取空数据后显式清 OVRERR 标志，避免错误位滞留；丢掉的字节由帧头重同步自恢复。
             */
            while (DL_UART_Main_receiveDataCheck(UART_2_INST, &byte)) {
                if (s_uart2RxHandler != NULL) {
                    s_uart2RxHandler(byte);
                }
            }
            DL_UART_Main_clearInterruptStatus(UART_2_INST,
                DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
            break;

        default:
            break;
    }
}
