#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_motor.h"
/* 需要用到时再解开：#include "bsp_servo.h" / "app_imu_uart_task.h" / "laser_ld14.h" / "ball_parser.h" */

/* ==================================================================
 * 第 1 题：<在此写本题要求，例如"直行 1 米后停车">
 *
 * 本文件是一个【可直接改的状态机示例】：进入后四轮一起前进一段固定距离，
 * 走完自动停车。你把它当模板：增删状态、替换动作与切换条件即可。
 *
 * 调用节拍：OnLoop 每 30ms 被 UIMENU 调一次；机动级决策足够。
 * ================================================================== */

/* 本题参数（示例值，按实车标定）。 */
#define T1_DRIVE_REVS     (10U)    /* 前进圈数（示例：走 10 圈电机） */
#define T1_DRIVE_RPM      (120U)   /* 前进转速 RPM */

/* 本题状态机的状态——按本题流程增删。 */
typedef enum {
    T1_STATE_START = 0,   /* 起步：下发前进指令 */
    T1_STATE_DRIVE,       /* 直行中：等走完设定距离 */
    T1_STATE_DONE         /* 完成：停车 */
} Task1State_t;

static Task1State_t s_state;

void Task1_OnEnter(void)
{
    /* 进入本题：使能驱动、复位状态机。真正的动作放到 OnLoop 的状态里。 */
    BspMotor_EnableAll();
    s_state = T1_STATE_START;
}

void Task1_OnLoop(void)
{
    switch (s_state) {
    case T1_STATE_START: {
        /* 四轮一起定距前进（正 steps = 前进）；走完各轮自动停。 */
        int32_t steps = (int32_t)((uint32_t)T1_DRIVE_REVS * BspMotor_StepsPerRev());
        BspMotor_MoveSteps4(steps, steps, steps, steps, T1_DRIVE_RPM);
        s_state = T1_STATE_DRIVE;
        break;
    }

    case T1_STATE_DRIVE:
        /* 切换条件示例：四轮都走完 → 进入完成。你也可以换成"寻迹到路口/激光到距离"。 */
        if (BspMotor_AllStopped()) {
            s_state = T1_STATE_DONE;
        }
        /* 例：改成寻迹转弯——if (寻迹到路口) { BspMotor_SetSpeedRpm4(120,-120,120,-120); ... } */
        break;

    case T1_STATE_DONE:
        BspMotor_StopAll();
        break;

    default:
        break;
    }
}

void Task1_OnExit(void)
{
    /* 退出本题：急停 + 失能，保证安全，并复位状态机。 */
    BspMotor_StopAll();
    BspMotor_DisableAll();
    s_state = T1_STATE_START;
}
