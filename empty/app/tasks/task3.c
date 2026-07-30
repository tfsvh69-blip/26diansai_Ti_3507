#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ball_control_task.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 3 题：ID1 正反方向短时旋转测试
 *
 * 本题通过 UART1 向 Emm42 控制器发送命令，只控制地址 1 对应的摆杆升降电机。
 * 先按当前角色层的正方向低速转约 500ms，再反向低速转约 500ms，最后急停并
 * 失能。经实机确认，ID1 的正方向会使连杆向下移动；后续改为位置模式时也沿用
 * 此方向定义。若机械安装改变，只改 emm42_robot.c 中的角色方向标定表。
 * ================================================================== */

/* 当前速度测试使用零加速度档位，命令下发后立即到达目标速度。 */
#define T3_SLOW_RPM               (40)
#define T3_EMM_ACC                (0U)
#define T3_DIRECTION_MS           (500U)

/* 沿用任务二已验证的使能等待；失能等待按任务二的保守值设置。 */
#define T3_RESET_SETTLE_MS        (60U)
#define T3_ENABLE_SETTLE_MS       (180U)

typedef enum {
    T3_STATE_WAIT_BALL_RELEASE = 0, /* 等待后台钢球闭环释放 ID1 */
    T3_STATE_RESET_DISABLE,       /* 清除 ID1 的残留使能状态 */
    T3_STATE_RESET_WAIT,          /* 等待失能命令生效 */
    T3_STATE_ENABLE,              /* 使能 ID1 */
    T3_STATE_ENABLE_WAIT,         /* 等待控制器完成使能 */
    T3_STATE_FORWARD,             /* 当前正方向低速旋转 */
    T3_STATE_REVERSE,             /* 当前反方向低速旋转 */
    T3_STATE_STOP,                /* 已急停，下一拍失能 */
    T3_STATE_DONE                  /* 测试结束，保持静止 */
} Task3State_t;

static Task3State_t s_state;
static TickType_t   s_stateStartTick;

static bool Task3_Elapsed(TickType_t now, TickType_t then, uint32_t timeoutMs)
{
    return (TickType_t)(now - then) >= pdMS_TO_TICKS(timeoutMs);
}

void Task3_OnEnter(void)
{
    /*
     * 题目三是 ID1 独立方向测试，与后台钢球闭环不能同时占用同一电机。
     * 先通过命令队列请求闭环安全退出，再由 OnLoop 等待它真正释放。
     */
    AppBallControl_RequestStop();
    s_state = T3_STATE_WAIT_BALL_RELEASE;
    s_stateStartTick = 0U;
}

void Task3_OnLoop(void)
{
    TickType_t now = xTaskGetTickCount();

    switch (s_state) {
    case T3_STATE_WAIT_BALL_RELEASE:
        if (!AppBallControl_IsActive()) {
            s_state = T3_STATE_RESET_DISABLE;
        }
        break;

    case T3_STATE_RESET_DISABLE:
        /* 仅失能 ID1，绝不触碰任务二使用的 ID2/ID3。 */
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        s_stateStartTick = now;
        s_state = T3_STATE_RESET_WAIT;
        break;

    case T3_STATE_RESET_WAIT:
        if (Task3_Elapsed(now, s_stateStartTick, T3_RESET_SETTLE_MS)) {
            s_state = T3_STATE_ENABLE;
        }
        break;

    case T3_STATE_ENABLE:
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
        s_stateStartTick = now;
        s_state = T3_STATE_ENABLE_WAIT;
        break;

    case T3_STATE_ENABLE_WAIT:
        if (Task3_Elapsed(now, s_stateStartTick, T3_ENABLE_SETTLE_MS)) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT, T3_SLOW_RPM, T3_EMM_ACC);
            s_stateStartTick = now;
            s_state = T3_STATE_FORWARD;
        }
        break;

    case T3_STATE_FORWARD:
        if (Task3_Elapsed(now, s_stateStartTick, T3_DIRECTION_MS)) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT, -T3_SLOW_RPM, T3_EMM_ACC);
            s_stateStartTick = now;
            s_state = T3_STATE_REVERSE;
        }
        break;

    case T3_STATE_REVERSE:
        if (Task3_Elapsed(now, s_stateStartTick, T3_DIRECTION_MS)) {
            /* 先停止、下一次 UI 周期再失能，确保两条 UART 帧自然错开。 */
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            s_state = T3_STATE_STOP;
        }
        break;

    case T3_STATE_STOP:
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        s_state = T3_STATE_DONE;
        RobotCore_NotifyTaskFinished(2U);
        break;

    case T3_STATE_DONE:
        break;

    default:
        /* 状态异常时只对本题 ID1 做安全收尾。 */
        Emm42Robot_Stop(EMM42_ROBOT_LIFT);
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        s_state = T3_STATE_DONE;
        break;
    }
}

void Task3_OnExit(void)
{
    /* K4 退出时仅急停、失能 ID1；两帧间隔 5ms，避免共享总线连续帧干扰。 */
    Emm42Robot_Stop(EMM42_ROBOT_LIFT);
    vTaskDelay(pdMS_TO_TICKS(5U));
    Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);

    s_state = T3_STATE_WAIT_BALL_RELEASE;
    s_stateStartTick = 0U;
}
