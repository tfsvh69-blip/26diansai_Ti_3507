#ifndef APP_MOTOR_STATUS_H
#define APP_MOTOR_STATUS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 电机1 运行状态快照（跨任务诊断共享）。
 *
 * 写入方：MOTOR1 任务（app_motor_test_task.c），每次状态变化时完整更新。
 * 读取方：PERIPH 任务（app_periph_test_task.c），在 OLED 上只读显示。
 * 各字段均为独立标量、单向单写，无需加锁。
 *
 * 字段语义随控制模式变化：
 *   位置模式（MoveSteps）：running=运动中，dirForward=方向，param=本次圈数
 *   连续模式（RunContinuous）：running=运动中，dirForward=方向，param=当前速度档(1..5)
 *   停止时：running=false，param 无意义
 */
typedef struct {
    volatile bool    running;    /* 电机是否在转（含减速停止过程视为运行中） */
    volatile bool    dirForward; /* true=正向(DIR 低)，false=反向 */
    volatile uint8_t param;      /* 位置模式=圈数，连续模式=速度档位(1..N) */
} AppMotorDiag_t;

/* 全局单例，定义在 app_motor_test_task.c 中。 */
extern AppMotorDiag_t g_motorDiag;

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_STATUS_H */
