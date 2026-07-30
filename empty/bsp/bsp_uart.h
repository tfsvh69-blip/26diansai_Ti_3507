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

/*
 * UART0 接收字节回调类型（视觉端下行报文，如 $PONG/$ACK/$X 帧）。
 * 上位机约 15 帧/秒连续下发，10~30ms 任务轮询节拍会让 4 字节 RX FIFO 溢出丢字节，
 * 故 RX 走中断逐字节回调；回调在 UART0 中断上下文中执行，内部不得调用非 FromISR 的 FreeRTOS API。
 */
typedef void (*BspUart0RxHandler_t)(uint8_t byte);

/*
 * 注册 UART0 接收回调并使能 RX + 溢出中断 + NVIC。
 * 需在 UART0 外设(SYSCFG_DL_UART_0_init)初始化之后、调度器启动前调用。
 * 注意：注册后 RX FIFO 由中断取走，轮询接口 BspUart0_ReadByte 将收不到字节
 * （二者互斥；接收中断与 UART_ECHO 轮询自检不可同时启用）。
 */
void BspUart0_SetRxHandler(BspUart0RxHandler_t handler);
/* 无锁输出无符号十进制整数（不加锁，供已持锁的日志拼接及关中断的故障处理复用）。 */
void BspUart0_SendUint(uint32_t value);
void BspUart0_SendString(const char *str);
bool BspUart0_ReadByte(uint8_t *byte);
/* 多段拼接日志时手动加/解锁；单次 SendString 已自动加锁，无需再包裹。 */
void BspUart0_Lock(void);
void BspUart0_Unlock(void);

/*
 * UART1（张大头 Emm42_V5.0 闭环步进驱动，PA17=TX/PB5=RX，115200 8N1）接收字节回调类型。
 * 驱动器回复帧短（4~8 字节）但到达时刻不确定，故 RX 走中断逐字节回调；
 * 回调在 UART1 中断上下文中执行，内部不得调用非 FromISR 的 FreeRTOS API。
 */
typedef void (*BspUart1RxHandler_t)(uint8_t byte);

/*
 * 注册 UART1 接收回调并使能 RX + 溢出中断 + NVIC。
 * 需在 UART1 外设(SYSCFG_DL_UART_1_init)初始化之后、调度器启动前调用。
 * handler 允许为 NULL（只发不收，回复字节被丢弃）。
 */
void BspUart1_Init(BspUart1RxHandler_t handler);

/*
 * 向 UART1 阻塞发送一串字节（Emm42 命令帧最长 20 字节，115200 下约 1.7ms）。
 * 供协议层组帧后整帧下发；不要在中断或高频控制环里调用。
 */
void BspUart1_SendBytes(const uint8_t *data, uint16_t len);

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
