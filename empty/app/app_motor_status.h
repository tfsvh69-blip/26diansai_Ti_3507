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
 * 写入方：MOTOR 任务（app_motor_test_task.c），每次状态变化时完整更新。
 * 读取方：PERIPH 任务（app_periph_test_task.c），在 OLED 上只读显示。
 * 三个字段有语义关联（running/方向/圈数应一致），写入和整帧读取都用
 * taskENTER_CRITICAL 包裹取快照，避免读到"running 已更新但方向/圈数还是旧值"的撕裂帧。
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
