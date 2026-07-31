#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "app_ball_control_task.h"
#include "app_vision_link.h"
#include "bsp_buzzer.h"
#include "bsp_home_switch.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 题目 6：手动指定钢珠目标点后循迹。
 *
 * 进入题目先对 ID1 执行“下降触发限位→抬升到水平”的归零流程，随后失能 ID1，
 * 供操作者手动调整摆杆并摆放钢珠。第一次 K3 抓取当前视觉 X 作为本次独立闭环
 * 目标；第二次 K3 后按题目五同款流程驱动 ID2/ID3 循迹到终点。
 * ================================================================== */

/* ---- 题目六私有循迹参数：初值按值复制自任务五，后续独立调节。 ---- */
#define T6_KP                              (2.4F)
#define T6_KI                              (0.25F)
#define T6_KD                              (0.7F)
#define T6_INTEGRAL_LIMIT                  (20.0F)
#define T6_MAX_STEER_RPM                   (100.0F)
#define T6_BASE_RPM                        (80.0F)
#define T6_MIN_WHEEL_RPM                   (5.0F)
#define T6_MAX_WHEEL_RPM                   (110.0F)
#define T6_MIN_BASE_RPM                    (10.0F)
#define T6_EMM_ACC                         (5U)
#define T6_RAMP_RPM_PER_SEC                (20.0F)
#define T6_STOP_EMM_ACC                    (80U)
#define T6_DT_SEC                          (0.03F)
#define T6_TICK_MS                         (30U)
#define T6_LINE_LOST_TICKS                 (20U)
#define T6_RESET_SETTLE_TICKS              (2U)
#define T6_ENABLE_SETTLE_TICKS             (6U)
#define T6_EMM_CMD_GAP_MS                  (6U)
#define T6_ERROR_FILTER_ALPHA              (0.5F)
#define T6_ERROR_DEADBAND                  (2.0F)
#define T6_CORNER_SLOWDOWN_GAIN            (0.65F)
#define T6_STEER_SLEW_RPM_PER_SEC          (250.0F)
#define T6_ARM_HIT_MAX                     (3U)
#define T6_ARM_TICKS                       (15U)
#define T6_STOP_LINE_HIT_MIN               (5U)
#define T6_FINISH_MIN_ELAPSED_MS           (3000U)
#define T6_AFTER_LINE_MS                   (1500U)
#define T6_BEEP_DELAY_MS                   (800U)
#define T6_STOPWATCH_CAL_SCALE             (0.897F)

/* ---- 题目六私有限位归零参数；不复用开机归零模块的可调参数。 ---- */
#define T6_HOME_SEEK_RPM                   (5)
#define T6_HOME_LIFT_RPM                   (5U)
#define T6_HOME_ACC                        (0U)
#define T6_HOME_LEVEL_OFFSET_PULSES        (210)
#define T6_HOME_RESET_SETTLE_TICKS         (2U)
#define T6_HOME_ENABLE_SETTLE_TICKS        (6U)
#define T6_HOME_STOP_SETTLE_TICKS          (1U)
#define T6_HOME_LIFT_WAIT_TICKS            (35U)

/* ---- 手动标定后的钢珠闭环 profile，参数完全归任务六所有。 ---- */
#define T6_BALL_FRICTION_FF_PULSE          (0.0F)
#define T6_BALL_LEVEL_TRIM_PULSE           (-50)
#define T6_BALL_FILTER_ALPHA               (0.9F)
#define T6_BALL_FILTER_BETA                (0.2F)
#define T6_BALL_OUTPUT_SIGN                (-1.0F)
#define T6_BALL_KX_PULSE_PER_PX            (1.13F)
#define T6_BALL_KV_PULSE_PER_PXPS          (0.5F)
#define T6_BALL_SETTLE_DEADBAND_PX         (4.0F)
#define T6_BALL_FF_VEL_BLEND_PXPS          (15.0F)
#define T6_BALL_POS_RPM                    (200U)
#define T6_BALL_POS_ACC                    (240U)
#define T6_BALL_MAX_PULSE_STEP             (0U)
#define T6_BALL_HOLD_POSITION_PX           (6.0F)
#define T6_BALL_HOLD_VELOCITY_PXPS         (10.0F)
#define T6_BALL_HOLD_TIME_MS               (500U)

/*
 * 起步加速度前馈：ID1 正脉冲为抬升。该偏置只在轮速爬升阶段临时叠加到
 * T6_BALL_LEVEL_TRIM_PULSE，不是巡线 PID 或钢珠位置 PD 的增益。
 */
#define T6_START_ACCEL_LIFT_PULSES         (30)
#define T6_START_ACCEL_LIFT_RAMP_IN_MS     (200U)
#define T6_START_ACCEL_LIFT_RAMP_OUT_MS    (400U)

typedef enum {
    T6_STATE_WAIT_BALL_RELEASE = 0,
    T6_STATE_HOME_DISABLE,
    T6_STATE_HOME_DISABLE_WAIT,
    T6_STATE_HOME_CLEAR_CLOG,
    T6_STATE_HOME_ENABLE,
    T6_STATE_HOME_ENABLE_WAIT,
    T6_STATE_HOME_SEEK,
    T6_STATE_HOME_STOP_WAIT,
    T6_STATE_HOME_LIFT,
    T6_STATE_HOME_LIFT_WAIT,
    T6_STATE_MANUAL_TARGET,
    T6_STATE_CAPTURE_ENABLE,
    T6_STATE_CAPTURE_ENABLE_WAIT,
    T6_STATE_CAPTURE_ZERO,
    T6_STATE_WAIT_BALL_CONTROL,
    T6_STATE_BALL_READY,
    T6_STATE_BALL_RECOVER_WAIT,
    T6_STATE_RESET_DISABLE,
    T6_STATE_RESET_WAIT,
    T6_STATE_ENABLE_LEFT,
    T6_STATE_ENABLE_LEFT_WAIT,
    T6_STATE_ENABLE_RIGHT,
    T6_STATE_ENABLE_RIGHT_WAIT,
    T6_STATE_RUN,
    T6_STATE_AFTER_LINE,
    T6_STATE_DECEL,
    T6_STATE_STOP_LEFT,
    T6_STATE_STOP_RIGHT,
    T6_STATE_FINISHED
} Task6State_t;

static const AppBallControlProfile_t s_task6BallProfile = {
    T6_BALL_FILTER_ALPHA, T6_BALL_FILTER_BETA, T6_BALL_OUTPUT_SIGN,
    T6_BALL_KX_PULSE_PER_PX, T6_BALL_KV_PULSE_PER_PXPS, T6_BALL_LEVEL_TRIM_PULSE,
    T6_BALL_SETTLE_DEADBAND_PX, T6_BALL_FRICTION_FF_PULSE, T6_BALL_FF_VEL_BLEND_PXPS,
    T6_BALL_POS_RPM, T6_BALL_POS_ACC, T6_BALL_MAX_PULSE_STEP,
    T6_BALL_HOLD_POSITION_PX, T6_BALL_HOLD_VELOCITY_PXPS, T6_BALL_HOLD_TIME_MS
};

static Task6State_t s_state;
static Pid_t s_pid;
static float s_lastSteerRpm;
static float s_appliedSteerRpm;
static float s_leftRpm;
static float s_rightRpm;
static float s_filteredError;
static float s_rampBaseRpm;
static float s_straightRpm;
static int32_t s_sentLeftRpm;
static int32_t s_sentRightRpm;
static uint32_t s_settleTicks;
static uint32_t s_lineLostTicks;
static uint32_t s_armTicks;
static uint32_t s_elapsedTicks;
static uint32_t s_afterLineElapsedMs;
static uint32_t s_startLiftRampInElapsedMs;
static uint32_t s_startLiftRampOutElapsedMs;
static int16_t s_targetX;
static int32_t s_startLiftOffsetPulse;
static bool s_sendLeftNext;
static bool s_finishArmed;
static bool s_lineStopped;
static bool s_captureRequested;
static bool s_startCarRequested;
static bool s_ballControlRequested;
static bool s_wheelsStarted;
static bool s_runCompleted;
static bool s_finishBeeped;
static bool s_startLiftRampOut;
static bool s_startLiftCompensationDone;
static char s_uiStatusBuf[24];
static char s_phaseStatusBuf[16];

static float Task6_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static int32_t Task6_ScaleStartLift(uint32_t elapsedMs, uint32_t durationMs)
{
    if ((durationMs == 0U) || (elapsedMs >= durationMs)) {
        return T6_START_ACCEL_LIFT_PULSES;
    }
    return (int32_t)(((uint32_t)T6_START_ACCEL_LIFT_PULSES * elapsedMs +
                      (durationMs / 2U)) / durationMs);
}

static void Task6_SetStartLiftOffset(int32_t offsetPulse)
{
    if (offsetPulse == s_startLiftOffsetPulse) {
        return;
    }
    s_startLiftOffsetPulse = offsetPulse;
    AppBallControl_SetLevelTrimOffset(offsetPulse);
}

static void Task6_ResetStartLiftCompensation(void)
{
    s_startLiftRampInElapsedMs = 0U;
    s_startLiftRampOutElapsedMs = 0U;
    s_startLiftRampOut = false;
    s_startLiftCompensationDone = false;
    Task6_SetStartLiftOffset(0);
}

static void Task6_CancelStartLiftCompensation(void)
{
    Task6_ResetStartLiftCompensation();
    s_startLiftCompensationDone = true;
}

static void Task6_UpdateStartLiftCompensation(void)
{
    int32_t offsetPulse;

    if (s_startLiftCompensationDone) {
        return;
    }

    if (s_startLiftRampOut) {
        s_startLiftRampOutElapsedMs += T6_TICK_MS;
        if (s_startLiftRampOutElapsedMs >= T6_START_ACCEL_LIFT_RAMP_OUT_MS) {
            Task6_SetStartLiftOffset(0);
            s_startLiftCompensationDone = true;
            return;
        }
        offsetPulse = T6_START_ACCEL_LIFT_PULSES -
                      Task6_ScaleStartLift(s_startLiftRampOutElapsedMs,
                                           T6_START_ACCEL_LIFT_RAMP_OUT_MS);
        Task6_SetStartLiftOffset(offsetPulse);
        return;
    }

    if (s_startLiftRampInElapsedMs < T6_START_ACCEL_LIFT_RAMP_IN_MS) {
        s_startLiftRampInElapsedMs += T6_TICK_MS;
    }
    offsetPulse = Task6_ScaleStartLift(s_startLiftRampInElapsedMs,
                                       T6_START_ACCEL_LIFT_RAMP_IN_MS);
    Task6_SetStartLiftOffset(offsetPulse);

    if (s_rampBaseRpm >= T6_BASE_RPM) {
        s_startLiftRampOut = true;
        s_startLiftRampOutElapsedMs = 0U;
    }
}

static void Task6_ClampWheelPair(float *leftRpm, float *rightRpm)
{
    float high = (*leftRpm > *rightRpm) ? *leftRpm : *rightRpm;
    float low;
    float shift;

    if (high > T6_MAX_WHEEL_RPM) {
        shift = high - T6_MAX_WHEEL_RPM;
        *leftRpm -= shift;
        *rightRpm -= shift;
    }
    low = (*leftRpm < *rightRpm) ? *leftRpm : *rightRpm;
    if (low < T6_MIN_WHEEL_RPM) {
        shift = T6_MIN_WHEEL_RPM - low;
        *leftRpm += shift;
        *rightRpm += shift;
    }
    *leftRpm = Task6_Clamp(*leftRpm, T6_MIN_WHEEL_RPM, T6_MAX_WHEEL_RPM);
    *rightRpm = Task6_Clamp(*rightRpm, T6_MIN_WHEEL_RPM, T6_MAX_WHEEL_RPM);
}

/* 每拍最多下发一个轮速帧，避免与 BALLCTRL 共享 UART1 时背靠背丢帧。 */
static void Task6_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t left = (int32_t)(leftRpm + 0.5F);
    int32_t right = (int32_t)(rightRpm + 0.5F);

    if (s_sendLeftNext) {
        if (left != s_sentLeftRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_L, (int16_t)left, T6_EMM_ACC);
            s_sentLeftRpm = left;
        }
    } else if (right != s_sentRightRpm) {
        Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_R, (int16_t)right, T6_EMM_ACC);
        s_sentRightRpm = right;
    }
    s_sendLeftNext = !s_sendLeftNext;
}

static bool Task6_GetLineError(float *error, uint32_t *hitCountOut)
{
    uint8_t bitmap = BspLine_ReadAll();
    int32_t weightedSum = 0;
    uint32_t hitCount = 0U;
    uint32_t physicalIdx;

    for (physicalIdx = 0U; physicalIdx < (uint32_t)BSP_LINE_COUNT; physicalIdx++) {
        uint32_t channel = (uint32_t)BSP_LINE_COUNT - 1U - physicalIdx;
        if ((bitmap & (uint8_t)(1U << channel)) != 0U) {
            weightedSum += (int32_t)(physicalIdx * 2U) - 7;
            hitCount++;
        }
    }
    *hitCountOut = hitCount;
    if (hitCount == 0U) return false;
    *error = (float)weightedSum / (float)hitCount;
    return true;
}

static void Task6_UpdateTracking(bool lineFound, float rawError)
{
    float delta;
    float maxDelta = T6_STEER_SLEW_RPM_PER_SEC * T6_DT_SEC;

    if (lineFound) {
        if (s_lineStopped) {
            s_filteredError = rawError;
            Pid_Reset(&s_pid);
        } else {
            s_filteredError += T6_ERROR_FILTER_ALPHA * (rawError - s_filteredError);
        }
        s_lineLostTicks = 0U;
        s_lineStopped = false;
        if ((s_filteredError > -T6_ERROR_DEADBAND) &&
            (s_filteredError < T6_ERROR_DEADBAND)) {
            s_lastSteerRpm = Pid_Update(&s_pid, 0.0F, T6_DT_SEC);
        } else {
            s_lastSteerRpm = Pid_Update(&s_pid, s_filteredError, T6_DT_SEC);
        }
    } else if (++s_lineLostTicks >= T6_LINE_LOST_TICKS) {
        s_lineStopped = true;
    }

    delta = s_lastSteerRpm - s_appliedSteerRpm;
    if (delta > maxDelta) delta = maxDelta;
    if (delta < -maxDelta) delta = -maxDelta;
    s_appliedSteerRpm += delta;
}

static void Task6_ApplyTracking(void)
{
    float steerAbs;
    float baseRpm;
    float left;
    float right;

    if (s_lineStopped) {
        Task6_CancelStartLiftCompensation();
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        Task6_ApplyWheelRpm(s_leftRpm, s_rightRpm);
        return;
    }
    steerAbs = (s_appliedSteerRpm >= 0.0F) ? s_appliedSteerRpm : -s_appliedSteerRpm;
    baseRpm = T6_BASE_RPM - T6_CORNER_SLOWDOWN_GAIN * steerAbs;
    baseRpm = Task6_Clamp(baseRpm, T6_MIN_BASE_RPM, T6_BASE_RPM);
    if (s_state == T6_STATE_RUN) {
        s_rampBaseRpm += T6_RAMP_RPM_PER_SEC * T6_DT_SEC;
        if (s_rampBaseRpm > T6_BASE_RPM) s_rampBaseRpm = T6_BASE_RPM;
        Task6_UpdateStartLiftCompensation();
    }
    if (baseRpm > s_rampBaseRpm) baseRpm = s_rampBaseRpm;
    left = baseRpm + s_appliedSteerRpm;
    right = baseRpm - s_appliedSteerRpm;
    Task6_ClampWheelPair(&left, &right);
    s_leftRpm = left;
    s_rightRpm = right;
    Task6_ApplyWheelRpm(left, right);
}

static uint32_t Task6_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T6_TICK_MS) * T6_STOPWATCH_CAL_SCALE);
}

static void Task6_FormatRemaining(const char *prefix, uint32_t elapsedMs)
{
    uint32_t remainingMs;
    uint32_t tenths;
    uint32_t idx = 0U;

    if (elapsedMs >= T6_AFTER_LINE_MS) {
        remainingMs = 0U;
    } else {
        remainingMs = T6_AFTER_LINE_MS - elapsedMs;
    }
    tenths = (remainingMs + 99U) / 100U;

    while (*prefix != '\0') {
        s_phaseStatusBuf[idx++] = *prefix++;
    }
    s_phaseStatusBuf[idx++] = (char)('0' + ((tenths / 10U) % 10U));
    s_phaseStatusBuf[idx++] = '.';
    s_phaseStatusBuf[idx++] = (char)('0' + (tenths % 10U));
    s_phaseStatusBuf[idx++] = 's';
    s_phaseStatusBuf[idx] = '\0';
}

static const char *Task6_GetPhaseStatus(void)
{
    switch (s_state) {
    case T6_STATE_RUN:
        if (s_lineStopped) {
            return "LOST";
        }
        return s_finishArmed ? "LINE" : "ARM";

    case T6_STATE_AFTER_LINE:
        Task6_FormatRemaining("GO:", s_afterLineElapsedMs);
        return s_phaseStatusBuf;

    case T6_STATE_DECEL:
        if (s_lineStopped) {
            return "LOST";
        }
        return "DEC";

    case T6_STATE_STOP_LEFT:
    case T6_STATE_STOP_RIGHT:
        return "STOP";

    case T6_STATE_FINISHED:
        return "DONE";

    default:
        return "INIT";
    }
}

void Task6_OnConfirm(void)
{
    if (s_state == T6_STATE_MANUAL_TARGET) s_captureRequested = true;
    else if (s_state == T6_STATE_BALL_READY) s_startCarRequested = true;
}

void Task6_OnEnter(void)
{
    Pid_Init(&s_pid, T6_KP, T6_KI, T6_KD, T6_INTEGRAL_LIMIT, T6_MAX_STEER_RPM);
    s_state = T6_STATE_WAIT_BALL_RELEASE;
    s_lastSteerRpm = s_appliedSteerRpm = s_leftRpm = s_rightRpm = 0.0F;
    s_filteredError = s_rampBaseRpm = s_straightRpm = 0.0F;
    s_sentLeftRpm = s_sentRightRpm = 0;
    s_settleTicks = s_lineLostTicks = s_armTicks = s_elapsedTicks = s_afterLineElapsedMs = 0U;
    s_startLiftRampInElapsedMs = s_startLiftRampOutElapsedMs = 0U;
    s_targetX = 0;
    s_startLiftOffsetPulse = 0;
    s_sendLeftNext = true;
    s_finishArmed = s_lineStopped = s_captureRequested = s_startCarRequested = false;
    s_ballControlRequested = s_wheelsStarted = s_runCompleted = s_finishBeeped = false;
    s_startLiftRampOut = s_startLiftCompensationDone = false;
}

void Task6_OnLoop(void)
{
    AppBallControlStatus_t ballStatus;
    AppVisionXSample_t sample;
    float rawError;
    bool lineFound;
    uint32_t hitCount;

    AppBallControl_GetStatus(&ballStatus);
    if ((s_state >= T6_STATE_RUN) && (s_state != T6_STATE_FINISHED)) s_elapsedTicks++;

    if (s_state == T6_STATE_WAIT_BALL_RELEASE) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) s_state = T6_STATE_HOME_DISABLE;
        return;
    }
    if (s_state == T6_STATE_HOME_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        s_settleTicks = T6_HOME_RESET_SETTLE_TICKS;
        s_state = T6_STATE_HOME_DISABLE_WAIT;
        return;
    }
    if (s_state == T6_STATE_HOME_DISABLE_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_HOME_CLEAR_CLOG;
        return;
    }
    if (s_state == T6_STATE_HOME_CLEAR_CLOG) {
        Emm42Robot_ClearClogProtection(EMM42_ROBOT_LIFT);
        s_state = T6_STATE_HOME_ENABLE;
        return;
    }
    if (s_state == T6_STATE_HOME_ENABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
        s_settleTicks = T6_HOME_ENABLE_SETTLE_TICKS;
        s_state = T6_STATE_HOME_ENABLE_WAIT;
        return;
    }
    if (s_state == T6_STATE_HOME_ENABLE_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_HOME_SEEK;
        return;
    }
    if (s_state == T6_STATE_HOME_SEEK) {
        if (BspHomeSwitch_IsPressed()) {
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            s_settleTicks = T6_HOME_STOP_SETTLE_TICKS;
            s_state = T6_STATE_HOME_STOP_WAIT;
        } else {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT, (int16_t)-T6_HOME_SEEK_RPM, T6_HOME_ACC);
        }
        return;
    }
    if (s_state == T6_STATE_HOME_STOP_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_HOME_LIFT;
        return;
    }
    if (s_state == T6_STATE_HOME_LIFT) {
        Emm42Robot_MoveRelative(EMM42_ROBOT_LIFT, T6_HOME_LEVEL_OFFSET_PULSES,
                                 T6_HOME_LIFT_RPM, T6_HOME_ACC);
        s_settleTicks = T6_HOME_LIFT_WAIT_TICKS;
        s_state = T6_STATE_HOME_LIFT_WAIT;
        return;
    }
    if (s_state == T6_STATE_HOME_LIFT_WAIT) {
        if (s_settleTicks-- > 0U) return;
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        s_state = T6_STATE_MANUAL_TARGET;
        return;
    }

    if ((s_state != T6_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T6_STATE_BALL_RECOVER_WAIT) &&
        (ballStatus.state == APP_BALL_CONTROL_FAULT_EDGE)) {
        Task6_CancelStartLiftCompensation();
        AppBallControl_RequestStop();
        s_ballControlRequested = false;
        s_state = T6_STATE_BALL_RECOVER_WAIT;
    }
    if (s_state == T6_STATE_BALL_RECOVER_WAIT) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) s_state = T6_STATE_WAIT_BALL_CONTROL;
        return;
    }

    if (s_state == T6_STATE_MANUAL_TARGET) {
        if (!s_captureRequested) return;
        AppVisionLink_GetLatestX(&sample);
        if (!sample.valid) return;
        s_targetX = sample.pixel;
        s_captureRequested = false;
        s_state = T6_STATE_CAPTURE_ENABLE;
        return;
    }
    if (s_state == T6_STATE_CAPTURE_ENABLE) {
        Emm42Robot_ClearClogProtection(EMM42_ROBOT_LIFT);
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
        s_settleTicks = T6_HOME_ENABLE_SETTLE_TICKS;
        s_state = T6_STATE_CAPTURE_ENABLE_WAIT;
        return;
    }
    if (s_state == T6_STATE_CAPTURE_ENABLE_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_CAPTURE_ZERO;
        return;
    }
    if (s_state == T6_STATE_CAPTURE_ZERO) {
        Emm42Robot_ResetPosToZero(EMM42_ROBOT_LIFT);
        s_state = T6_STATE_WAIT_BALL_CONTROL;
        return;
    }
    if (s_state == T6_STATE_WAIT_BALL_CONTROL) {
        if (!s_ballControlRequested) {
            s_ballControlRequested = AppBallControl_RequestTargetWithProfile(s_targetX, &s_task6BallProfile);
        }
        if (s_ballControlRequested && (ballStatus.targetPx == s_targetX) &&
            ((ballStatus.state == APP_BALL_CONTROL_RUNNING) ||
             (ballStatus.state == APP_BALL_CONTROL_HOLDING))) {
            if (s_runCompleted) s_state = T6_STATE_FINISHED;
            else if (s_wheelsStarted) s_state = T6_STATE_RUN;
            else s_state = T6_STATE_BALL_READY;
        }
        return;
    }
    if (s_state == T6_STATE_BALL_READY) {
        if (s_startCarRequested) {
            s_startCarRequested = false;
            s_wheelsStarted = true;
            s_rampBaseRpm = 0.0F;
            Task6_ResetStartLiftCompensation();
            RobotCore_NotifyTaskStarted(5U);
            s_state = T6_STATE_RESET_DISABLE;
        }
        return;
    }

    if (s_state == T6_STATE_RESET_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        vTaskDelay(pdMS_TO_TICKS(T6_EMM_CMD_GAP_MS));
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_settleTicks = T6_RESET_SETTLE_TICKS;
        s_state = T6_STATE_RESET_WAIT;
        return;
    }
    if (s_state == T6_STATE_RESET_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_ENABLE_LEFT;
        return;
    }
    if (s_state == T6_STATE_ENABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_settleTicks = T6_ENABLE_SETTLE_TICKS;
        s_state = T6_STATE_ENABLE_LEFT_WAIT;
        return;
    }
    if (s_state == T6_STATE_ENABLE_LEFT_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_ENABLE_RIGHT;
        return;
    }
    if (s_state == T6_STATE_ENABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_settleTicks = T6_ENABLE_SETTLE_TICKS;
        s_state = T6_STATE_ENABLE_RIGHT_WAIT;
        return;
    }
    if (s_state == T6_STATE_ENABLE_RIGHT_WAIT) {
        if (s_settleTicks-- > 0U) return;
        s_state = T6_STATE_RUN;
    }
    if (s_state == T6_STATE_STOP_LEFT) {
        Task6_CancelStartLiftCompensation();
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T6_STOP_EMM_ACC);
        s_state = T6_STATE_STOP_RIGHT;
        return;
    }
    if (s_state == T6_STATE_STOP_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T6_STOP_EMM_ACC);
        s_leftRpm = s_rightRpm = 0.0F;
        s_state = T6_STATE_FINISHED;
        return;
    }
    if (s_state == T6_STATE_FINISHED) return;

    if (s_state == T6_STATE_AFTER_LINE) {
        s_afterLineElapsedMs += T6_TICK_MS;
        s_leftRpm = s_rightRpm = s_straightRpm;
        Task6_ApplyWheelRpm(s_leftRpm, s_rightRpm);
        if (!s_finishBeeped && (s_afterLineElapsedMs >= T6_BEEP_DELAY_MS)) {
            s_finishBeeped = true;
            BspBuzzer_BeepShort();
            RobotCore_NotifyTaskFinished(5U);
        }
        if (s_afterLineElapsedMs >= T6_AFTER_LINE_MS) s_state = T6_STATE_DECEL;
        return;
    }
    if (s_state == T6_STATE_DECEL) {
        s_rampBaseRpm -= T6_RAMP_RPM_PER_SEC * T6_DT_SEC;
        if (s_rampBaseRpm <= 0.0F) {
            s_rampBaseRpm = 0.0F;
            s_state = T6_STATE_STOP_LEFT;
            return;
        }
        lineFound = Task6_GetLineError(&rawError, &hitCount);
        Task6_UpdateTracking(lineFound, rawError);
        Task6_ApplyTracking();
        return;
    }

    lineFound = Task6_GetLineError(&rawError, &hitCount);
    if (s_finishArmed && (hitCount >= T6_STOP_LINE_HIT_MIN) &&
        ((s_elapsedTicks * T6_TICK_MS) >= T6_FINISH_MIN_ELAPSED_MS)) {
        Task6_CancelStartLiftCompensation();
        s_straightRpm = (s_leftRpm + s_rightRpm) * 0.5F;
        s_afterLineElapsedMs = 0U;
        s_lastSteerRpm = s_appliedSteerRpm = 0.0F;
        s_runCompleted = true;
        s_state = T6_STATE_AFTER_LINE;
        return;
    }
    Task6_UpdateTracking(lineFound, rawError);
    if (!s_finishArmed) {
        if (hitCount <= T6_ARM_HIT_MAX) {
            if (++s_armTicks >= T6_ARM_TICKS) s_finishArmed = true;
        } else s_armTicks = 0U;
    }
    Task6_ApplyTracking();
}

void Task6_OnExit(void)
{
    Task6_CancelStartLiftCompensation();
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T6_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T6_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T6_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
    Emm42Robot_Stop(EMM42_ROBOT_LIFT);
    Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
    AppBallControl_RequestStop();
    Pid_Reset(&s_pid);
    s_state = T6_STATE_WAIT_BALL_RELEASE;
}

const char *Task6_GetUiStatus(void)
{
    const char *phase;
    uint32_t totalMs;
    uint32_t secWhole;
    uint32_t tenths;
    uint32_t idx = 0U;
    uint32_t n = 0U;
    uint32_t value;
    char digits[10];

    if (s_state <= T6_STATE_HOME_LIFT_WAIT) return "T6 HOME";
    if (s_state == T6_STATE_MANUAL_TARGET) return s_captureRequested ? "T6 NO BALL" : "T6 SET BALL";
    if ((s_state == T6_STATE_CAPTURE_ENABLE) || (s_state == T6_STATE_CAPTURE_ENABLE_WAIT) ||
         (s_state == T6_STATE_CAPTURE_ZERO) || (s_state == T6_STATE_WAIT_BALL_CONTROL)) return "T6 B WAIT";
    if (s_state == T6_STATE_BALL_READY) return "T6 K3=GO";
    if (s_state == T6_STATE_BALL_RECOVER_WAIT) return "T6 B RECOV";

    phase = Task6_GetPhaseStatus();
    totalMs = Task6_GetElapsedMs();
    secWhole = totalMs / 1000U;
    tenths = (totalMs / 100U) % 10U;
    value = secWhole;

    s_uiStatusBuf[idx++] = 'T';
    s_uiStatusBuf[idx++] = ':';
    if (value == 0U) {
        s_uiStatusBuf[idx++] = '0';
    } else {
        while (value > 0U) {
            digits[n++] = (char)('0' + (value % 10U));
            value /= 10U;
        }
        while (n > 0U) {
            s_uiStatusBuf[idx++] = digits[--n];
        }
    }
    s_uiStatusBuf[idx++] = '.';
    s_uiStatusBuf[idx++] = (char)('0' + tenths);
    s_uiStatusBuf[idx++] = 's';
    s_uiStatusBuf[idx++] = ' ';
    while ((*phase != '\0') && (idx < (sizeof(s_uiStatusBuf) - 1U))) {
        s_uiStatusBuf[idx++] = *phase++;
    }
    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}
