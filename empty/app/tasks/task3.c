#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_motor.h"
/* 需要用到时再解开：#include "bsp_servo.h" / "app_imu_uart_task.h" / "laser_ld14.h" / "ball_parser.h" */

/* ==================================================================
 * 第 3 题：<在此写本题要求>
 *
 * 状态机骨架：在 OnLoop 的 switch 里逐状态写"动作 + 切换条件"。
 * OnLoop 每 30ms 调一次；电机/舵机的实际运动由底层 ISR 后台完成，这里只做决策。
 * ================================================================== */

/* 本题状态机的状态——按本题流程增删。 */
typedef enum {
    T3_STATE_IDLE = 0,   /* 待机：等待开始条件 */
    T3_STATE_RUN,        /* 运行主体 */
    T3_STATE_DONE        /* 完成 */
} Task3State_t;

static Task3State_t s_state;

void Task3_OnEnter(void)
{
    /* 进入本题：使能驱动、复位状态机。 */
    BspMotor_EnableAll();
    s_state = T3_STATE_IDLE;
    /* TODO: 本题进入时的一次性准备（如舵机归中、变量清零）。 */
}

void Task3_OnLoop(void)
{
    switch (s_state) {
    case T3_STATE_IDLE:
        /* TODO: 触发条件满足 → s_state = T3_STATE_RUN; */
        break;

    case T3_STATE_RUN:
        /* TODO: 本题动作，例如 BspMotor_SetSpeedRpm4(...);
         *       满足条件后切状态，如 if (寻迹到路口/激光到距离) s_state = T3_STATE_DONE; */
        break;

    case T3_STATE_DONE:
        BspMotor_StopAll();
        break;

    default:
        break;
    }
}

void Task3_OnExit(void)
{
    /* 退出本题：急停 + 失能，保证安全，并复位状态机。 */
    BspMotor_StopAll();
    BspMotor_DisableAll();
    s_state = T3_STATE_IDLE;
}
