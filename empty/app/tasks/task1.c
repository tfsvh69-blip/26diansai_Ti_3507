#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_motor.h"

/* ==================================================================
 * 第 1 题：视觉端正常画面原始录像
 *
 * RobotCore 进入本题时先向视觉端发送 TASK START。本题不驱动任何电机，
 * 仅保持 5 秒，让视觉端写入没有任务文字、检测框或坐标叠加的正常相机画面；
 * 到时自动通知 RobotCore 发送同一次运行的 TASK STOP，视觉端据此保存文件。
 * ================================================================== */

/* 正常画面录像时长；UIMENU 轮询误差最多约一个 30ms 调用周期。 */
#define T1_RECORD_DURATION_MS    (5000U)

typedef enum {
    T1_STATE_RECORD = 0,
    T1_STATE_DONE
} Task1State_t;

static Task1State_t s_state;
static TickType_t   s_recordStartTick;

/* 题目一不允许遗留四路 STEP/DIR 电机动作，进入、退出和异常路径均收敛到失能。 */
static void Task1_StopAndDisableMotors(void)
{
    BspMotor_EmergencyStop(BSP_MOTOR_1);
    BspMotor_EmergencyStop(BSP_MOTOR_2);
    BspMotor_EmergencyStop(BSP_MOTOR_3);
    BspMotor_EmergencyStop(BSP_MOTOR_4);
    BspMotor_DisableAll();
}

void Task1_OnEnter(void)
{
    Task1_StopAndDisableMotors();
    s_recordStartTick = xTaskGetTickCount();
    s_state = T1_STATE_RECORD;
}

void Task1_OnLoop(void)
{
    TickType_t now;

    switch (s_state) {
    case T1_STATE_RECORD:
        now = xTaskGetTickCount();
        if ((TickType_t)(now - s_recordStartTick) >= pdMS_TO_TICKS(T1_RECORD_DURATION_MS)) {
            /* 仅在首次到时发送 STOP，避免每个 UI 周期重复结束录像。 */
            s_state = T1_STATE_DONE;
            RobotCore_NotifyTaskFinished(0U);
        }
        break;

    case T1_STATE_DONE:
        /* 已发送 STOP，保持静止，等待用户通过 K4 返回菜单。 */
        break;

    default:
        /* 异常状态优先保证电机失能，并结束本次录像。 */
        Task1_StopAndDisableMotors();
        s_state = T1_STATE_DONE;
        RobotCore_NotifyTaskFinished(0U);
        break;
    }
}

void Task1_OnExit(void)
{
    /* K4 提前退出时同样由 RobotCore 发送 STOP，视觉端保存已录片段。 */
    Task1_StopAndDisableMotors();
    s_recordStartTick = 0;
    s_state = T1_STATE_RECORD;
}
