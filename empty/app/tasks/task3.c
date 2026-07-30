#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ball_control_task.h"

/* ==================================================================
 * 第 3 题：钢珠第一阶段定位
 *
 * 上电后须先在菜单页让摆杆水平、钢珠置中并按 K4 建立全局位置原点。本题只
 * 复用后台闭环的执行器和该原点；所有可调控制参数均定义在本文件，不影响菜单
 * K4 闭环。当前仅进行第一阶段调参：325 附近出发 -> 415±10 低速稳定 200ms。
 * 后续 415→另一目标点的第二阶段将使用另一组 T3_* 参数单独加入，不能复用本阶段参数。
 * ================================================================== */

/* 赛题目标与验收门限。 */
#define T3_STAGE1_START_X_PX              (325)
#define T3_STAGE1_TARGET_X_PX             (415)
#define T3_STAGE1_TOLERANCE_PX            (10.0F)
#define T3_STAGE1_STABLE_MS               (200U)
#define T3_STAGE1_MAX_RUN_MS              (5000U)

/*
 * 第一阶段独立闭环参数。后续仅修改 T3_STAGE1_*，不要修改
 * app_ball_control_task.c 中的 BALL_CTRL_*；第二阶段会另建 T3_STAGE2_* 参数组。
 */
#define T3_STAGE1_FILTER_ALPHA            (0.70F)
#define T3_STAGE1_FILTER_BETA             (0.08F)
#define T3_STAGE1_OUTPUT_SIGN             (1.0F)
#define T3_STAGE1_KX_PULSE_PER_PX         (27.0F)
#define T3_STAGE1_KV_PULSE_PER_PXPS       (8.3F)
#define T3_STAGE1_LEVEL_TRIM_PULSE        (0)
#define T3_STAGE1_SETTLE_DEADBAND_PX      (12.0F)
#define T3_STAGE1_STICTION_PULSE          (3600.0F)
#define T3_STAGE1_STUCK_VELOCITY_PXPS     (6.0F)
#define T3_STAGE1_STUCK_TIME_MS           (150U)
#define T3_STAGE1_POS_RPM                 (185U)
#define T3_STAGE1_POS_ACC                 (0U)
#define T3_STAGE1_HOLD_POSITION_PX        (10.0F)
#define T3_STAGE1_HOLD_VELOCITY_PXPS      (15.0F)
#define T3_STAGE1_HOLD_TIME_MS            (200U)

typedef enum {
    T3_STATE_WAIT_BALL_RELEASE = 0,
    T3_STATE_START_FORWARD,
    T3_STATE_WAIT_FIRST_CONTROL,
    T3_STATE_TO_FORWARD,
    T3_STATE_FORWARD_STABLE,
    T3_STATE_DONE,
    T3_STATE_TIMEOUT
} Task3State_t;

/* 第一阶段独立 profile：进入 415±10px 后停止微调，避免为更小误差来回推球。 */
static const AppBallControlProfile_t s_task3ForwardProfile = {
    T3_STAGE1_FILTER_ALPHA,
    T3_STAGE1_FILTER_BETA,
    T3_STAGE1_OUTPUT_SIGN,
    T3_STAGE1_KX_PULSE_PER_PX,
    T3_STAGE1_KV_PULSE_PER_PXPS,
    T3_STAGE1_LEVEL_TRIM_PULSE,
    T3_STAGE1_SETTLE_DEADBAND_PX,
    T3_STAGE1_STICTION_PULSE,
    T3_STAGE1_STUCK_VELOCITY_PXPS,
    T3_STAGE1_STUCK_TIME_MS,
    T3_STAGE1_POS_RPM,
    T3_STAGE1_POS_ACC,
    T3_STAGE1_HOLD_POSITION_PX,
    T3_STAGE1_HOLD_VELOCITY_PXPS,
    T3_STAGE1_HOLD_TIME_MS
};

static Task3State_t s_state;
static TickType_t   s_runStartTick;
static TickType_t   s_stableStartTick;
static bool         s_timerStarted;
static bool         s_finishNotified;
static char         s_uiStatusBuf[20];

static float Task3_Abs(float value)
{
    return (value >= 0.0F) ? value : -value;
}

static bool Task3_Elapsed(TickType_t now, TickType_t then, uint32_t timeoutMs)
{
    return (TickType_t)(now - then) >= pdMS_TO_TICKS(timeoutMs);
}

static bool Task3_IsControlFrame(const AppBallControlStatus_t *status)
{
    return (status->state == APP_BALL_CONTROL_RUNNING) ||
           (status->state == APP_BALL_CONTROL_HOLDING);
}

static bool Task3_IsAtTarget(const AppBallControlStatus_t *status, int16_t targetPx,
                             float tolerancePx)
{
    return Task3_Abs(status->filteredPx - (float)targetPx) <= tolerancePx;
}

static void Task3_NotifyFinished(void)
{
    if (!s_finishNotified) {
        RobotCore_NotifyTaskFinished(2U);
        s_finishNotified = true;
    }
}

static void Task3_EnterTimeout(void)
{
    AppBallControl_RequestStop();
    s_state = T3_STATE_TIMEOUT;
    Task3_NotifyFinished();
}

static uint32_t Task3_GetElapsedMs(void)
{
    if (!s_timerStarted) {
        return 0U;
    }

    return (uint32_t)((xTaskGetTickCount() - s_runStartTick) *
                      (TickType_t)portTICK_PERIOD_MS);
}

const char *Task3_GetUiStatus(void)
{
    const char *phase;
    uint32_t tenths;
    uint32_t idx = 0U;

    switch (s_state) {
    case T3_STATE_WAIT_BALL_RELEASE:
        return "T3 RELEASE";
    case T3_STATE_START_FORWARD:
    case T3_STATE_WAIT_FIRST_CONTROL:
        return "T3 WAIT X";
    case T3_STATE_TO_FORWARD:
        phase = "T3 TO415 ";
        break;
    case T3_STATE_FORWARD_STABLE:
        phase = "T3 STABLE ";
        break;
    case T3_STATE_DONE:
        return "T3 DONE";
    case T3_STATE_TIMEOUT:
        return "T3 TIMEOUT";
    default:
        return "T3 ERROR";
    }

    while (phase[idx] != '\0') {
        s_uiStatusBuf[idx] = phase[idx];
        idx++;
    }

    tenths = Task3_GetElapsedMs() / 100U;
    s_uiStatusBuf[idx++] = (char)('0' + ((tenths / 10U) % 10U));
    s_uiStatusBuf[idx++] = '.';
    s_uiStatusBuf[idx++] = (char)('0' + (tenths % 10U));
    s_uiStatusBuf[idx++] = 's';
    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}

void Task3_OnEnter(void)
{
    /* 先由后台闭环完成急停、失能，避免两套控制同时向 ID1 下发命令。 */
    AppBallControl_RequestStop();
    s_state = T3_STATE_WAIT_BALL_RELEASE;
    s_runStartTick = 0U;
    s_stableStartTick = 0U;
    s_timerStarted = false;
    s_finishNotified = false;
}

void Task3_OnLoop(void)
{
    AppBallControlStatus_t status;
    TickType_t now = xTaskGetTickCount();

    AppBallControl_GetStatus(&status);

    switch (s_state) {
    case T3_STATE_WAIT_BALL_RELEASE:
        if (!AppBallControl_IsActive()) {
            s_state = T3_STATE_START_FORWARD;
        }
        break;

    case T3_STATE_START_FORWARD:
        if (AppBallControl_RequestTargetWithProfile(T3_STAGE1_TARGET_X_PX,
                                                     &s_task3ForwardProfile)) {
            s_state = T3_STATE_WAIT_FIRST_CONTROL;
        } else {
            Task3_EnterTimeout();
        }
        break;

    case T3_STATE_WAIT_FIRST_CONTROL:
        if (status.state == APP_BALL_CONTROL_FAULT_EDGE) {
            Task3_EnterTimeout();
        } else if (Task3_IsControlFrame(&status)) {
            /* 后台线程仅在第二帧有效视觉数据后真正下发首条位置命令。 */
            s_runStartTick = now;
            s_timerStarted = true;
            s_state = T3_STATE_TO_FORWARD;
        }
        break;

    case T3_STATE_TO_FORWARD:
        if (Task3_Elapsed(now, s_runStartTick, T3_STAGE1_MAX_RUN_MS)) {
            Task3_EnterTimeout();
        } else if (status.state == APP_BALL_CONTROL_FAULT_EDGE) {
            Task3_EnterTimeout();
        } else if (Task3_IsAtTarget(&status, T3_STAGE1_TARGET_X_PX,
                                     T3_STAGE1_TOLERANCE_PX) &&
                   (Task3_Abs(status.velocityPxPerSec) <= T3_STAGE1_HOLD_VELOCITY_PXPS)) {
            s_stableStartTick = now;
            s_state = T3_STATE_FORWARD_STABLE;
        }
        break;

    case T3_STATE_FORWARD_STABLE:
        if (Task3_Elapsed(now, s_runStartTick, T3_STAGE1_MAX_RUN_MS)) {
            Task3_EnterTimeout();
        } else if (status.state == APP_BALL_CONTROL_FAULT_EDGE) {
            Task3_EnterTimeout();
        } else if (!Task3_IsAtTarget(&status, T3_STAGE1_TARGET_X_PX,
                                      T3_STAGE1_TOLERANCE_PX) ||
                   (Task3_Abs(status.velocityPxPerSec) > T3_STAGE1_HOLD_VELOCITY_PXPS)) {
            s_stableStartTick = 0U;
            s_state = T3_STATE_TO_FORWARD;
        } else if (Task3_Elapsed(now, s_stableStartTick, T3_STAGE1_STABLE_MS)) {
            /* 第一阶段完成后保持 ID1 使能和 X=415 绝对目标，直到用户按 K4 退出。 */
            s_state = T3_STATE_DONE;
            Task3_NotifyFinished();
        }
        break;

    case T3_STATE_DONE:
    case T3_STATE_TIMEOUT:
        break;

    default:
        Task3_EnterTimeout();
        break;
    }
}

void Task3_OnExit(void)
{
    /* 通过执行器统一急停和失能 ID1，避免任务层与后台线程争用 UART1。 */
    AppBallControl_RequestStop();
    s_state = T3_STATE_WAIT_BALL_RELEASE;
    s_timerStarted = false;
    s_stableStartTick = 0U;
}
