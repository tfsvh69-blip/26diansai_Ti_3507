#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建 UART0 递归互斥量，必须在调度器启动前调用一次。 */
void BspUart0_Init(void);
void BspUart0_SendByte(uint8_t byte);
void BspUart0_SendString(const char *str);
bool BspUart0_ReadByte(uint8_t *byte);
/* 多段拼接日志时手动加/解锁；单次 SendString 已自动加锁，无需再包裹。 */
void BspUart0_Lock(void);
void BspUart0_Unlock(void);

/*
 * UART2（激光测距1，PB15=TX/PB16=RX，230400 8N1）接收字节回调类型。
 * 230400 波特率连续流下任务轮询来不及，故 RX 走中断逐字节回调，
 * 用回调注册解耦 bsp 与解析模块（bsp 不直接依赖 module 层）。
 */
typedef void (*BspUart2RxHandler_t)(uint8_t byte);

/*
 * 注册 UART2 接收回调并使能 RX 中断 + NVIC。
 * 需在 UART2 外设(SYSCFG_DL_UART_2_init)初始化之后、调度器启动前调用。
 * handler 会在 UART2 中断上下文中被逐字节调用，内部不得调用非 FromISR 的 FreeRTOS API。
 */
void BspUart2_Init(BspUart2RxHandler_t handler);

#ifdef __cplusplus
}
#endif

#endif
