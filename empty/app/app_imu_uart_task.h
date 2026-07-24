#ifndef APP_IMU_UART_TASK_H
#define APP_IMU_UART_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void AppImuUartTask_Init(void);

/*
 * 读取最新 Yaw（偏航角）快照，供其它任务（如 OLED UI）显示或使用。
 * 返回值：true 表示 IMU 已初始化成功且快照有效；false 表示 IMU 未就绪
 *         （初始化失败/重试中或 IMU 功能未启用），此时 *outYawCentideg 无意义。
 * 单位：厘度（0.01°），范围约 -18000~+18000（即 -180.00°~+180.00°）。
 * 线程安全：内部用临界区取一致快照，可在任意任务上下文调用。
 */
bool AppImuUartTask_GetYaw(int16_t *outYawCentideg);

#ifdef __cplusplus
}
#endif

#endif
