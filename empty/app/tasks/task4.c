#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 4 题：6.5 秒循迹 PID 缓停
 *
 * 本题循迹、入场、丢线保护、终点保护和定时减速逻辑均照搬第 2 题，参数也
 * 保持同值。唯一业务区别是：进入正常循迹后累计前进 6.5 秒，分别给左右轮发送
 * 速度模式 0 RPM 帧；该帧带 T4_STOP_EMM_ACC 加速度档位，驱动器按曲线缓停。
 * ================================================================== */

/* PID 增益、积分限幅与转向输出限幅。 */
#define T4_KP                              (5.0F)
#define T4_KI                              (0.15F)
#define T4_KD                              (0.2F)
#define T4_INTEGRAL_LIMIT                  (20.0F)
#define T4_MAX_STEER_RPM                   (100.0F)

/* 基础速度和单轮安全范围。 */
#define T4_BASE_RPM                        (110.0F)
#define T4_MIN_WHEEL_RPM                   (5.0F)
#define T4_MAX_WHEEL_RPM                   (230.0F)

/* 正常循迹速度模式加速度档位，参数与任务二一致。 */
#define T4_EMM_ACC                         (150U)

/*
 * 6.5 秒缓停参数：T4_STOP_AFTER_MS 固定本题开始缓停的时间；
 * T4_STOP_EMM_ACC 由使用者按实车需要传给速度模式 0 RPM 帧。
 * 数值越小减速越平缓，越大越接近立即停；可直接修改后重新烧录。
 */
#define T4_STOP_AFTER_MS                   (6500U)
#define T4_STOP_EMM_ACC                    (80U)

/* UIMENU 固定控制周期。 */
#define T4_DT_SEC                          (0.03F)
#define T4_TICK_MS                         (30U)

/* 丢线、入场和共享 UART1 总线时序参数。 */
#define T4_LINE_LOST_TICKS                 (20U)
#define T4_RESET_SETTLE_TICKS              (2U)
#define T4_ENABLE_SETTLE_TICKS             (6U)
#define T4_EMM_CMD_GAP_MS                  (5U)

/* 误差滤波、死区和转弯减速参数。 */
#define T4_ERROR_FILTER_ALPHA              (0.5F)
#define T4_ERROR_DEADBAND                  (1.0F)
#define T4_CORNER_SLOWDOWN_GAIN            (0.6F)
#define T4_MIN_BASE_RPM                    (10.0F)

/* 终点保护参数，与任务二一致。 */
#define T4_FINISH_ARM_HIT_MAX              (3U)
#define T4_FINISH_ARM_TICKS                (15U)
#define T4_FINISH_HIT_MIN                  (6U)
#define T4_FINISH_HIT_TICKS                (1U)

/* 秒表与定时减速参数，与任务二一致。 */
#define T4_STOPWATCH_CAL_SCALE             (0.897F)
#define T4_DECEL_START_MS                  (14500U)
#define T4_DECEL_GRADIENT_RPM_PER_SEC      (60.0F)
#define T4_DECEL_MIN_RPM                   (10.0F)

typedef enum {
    T4_STATE_RESET_DISABLE = 0,
    T4_STATE_RESET_WAIT,
    T4_STATE_ENABLE_LEFT,
    T4_STATE_ENABLE_LEFT_WAIT,
    T4_STATE_ENABLE_RIGHT,
    T4_STATE_ENABLE_RIGHT_WAIT,
    T4_STATE_RUN,
    T4_STATE_STOP,
    T4_STATE_TIME_STOP_LEFT,
    T4_STATE_TIME_STOP_RIGHT,
    T4_STATE_TIME_STOPPED,
    T4_STATE_FINISHED
} Task4State_t;

static Task4State_t s_state;
static Pid_t        s_pid;
static float        s_lastSteerRpm;
static float        s_leftRpm;
static float        s_rightRpm;
static float        s_filteredError;
static int32_t      s_sentLeftRpm;
static int32_t      s_sentRightRpm;
static uint32_t     s_lineLostTicks;
static uint32_t     s_enableSettleTicks;
static bool         s_sendLeftNext;
static bool         s_finishArmed;
static uint32_t     s_armTicks;
static uint32_t     s_finishHitTicks;
static uint32_t     s_elapsedTicks;
static uint32_t     s_runTicks;
static char         s_uiStatusBuf[16];

static float Task4_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

/* 整体平移限幅，保留左右轮差速，避免独立限幅压扁转向量。 */
static void Task4_ClampWheelPair(float *leftRpm, float *rightRpm)
{
    float hi = (*leftRpm > *rightRpm) ? *leftRpm : *rightRpm;
    float lo;
    float shift;

    if (hi > T4_MAX_WHEEL_RPM) {
        shift      = hi - T4_MAX_WHEEL_RPM;
        *leftRpm  -= shift;
        *rightRpm -= shift;
    }

    lo = (*leftRpm < *rightRpm) ? *leftRpm : *rightRpm;
    if (lo < T4_MIN_WHEEL_RPM) {
        shift      = T4_MIN_WHEEL_RPM - lo;
        *leftRpm  += shift;
        *rightRpm += shift;
    }

    *leftRpm  = Task4_Clamp(*leftRpm, T4_MIN_WHEEL_RPM, T4_MAX_WHEEL_RPM);
    *rightRpm = Task4_Clamp(*rightRpm, T4_MIN_WHEEL_RPM, T4_MAX_WHEEL_RPM);
}

/* 正常循迹每拍只发一帧，左右轮交替更新，避免共享总线背靠背丢帧。 */
static void Task4_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t leftInt = (leftRpm >= 0.0F) ? (int32_t)(leftRpm + 0.5F)
                                        : (int32_t)(leftRpm - 0.5F);
    int32_t rightInt = (rightRpm >= 0.0F) ? (int32_t)(rightRpm + 0.5F)
                                          : (int32_t)(rightRpm - 0.5F);

    if (s_sendLeftNext) {
        if (leftInt != s_sentLeftRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_L, (int16_t)leftInt, T4_EMM_ACC);
            s_sentLeftRpm = leftInt;
        }
    } else {
        if (rightInt != s_sentRightRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_R, (int16_t)rightInt, T4_EMM_ACC);
            s_sentRightRpm = rightInt;
        }
    }
    s_sendLeftNext = !s_sendLeftNext;
}

/* 按物理左→右的 LINE8→LINE1 顺序计算加权位置误差。 */
static bool Task4_GetLineError(float *error, uint32_t *hitCountOut)
{
    uint8_t  bitmap = BspLine_ReadAll();
    int32_t  weightedSum = 0;
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
    if (hitCount == 0U) {
        return false;
    }
    *error = (float)weightedSum / (float)hitCount;
    return true;
}

static uint32_t Task4_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T4_TICK_MS) * T4_STOPWATCH_CAL_SCALE);
}

const char *Task4_GetUiStatus(void)
{
    uint32_t totalMs = Task4_GetElapsedMs();
    uint32_t secWhole = totalMs / 1000U;
    uint32_t tenths = (totalMs / 100U) % 10U;
    uint32_t idx = 0U;
    char digits[10];
    uint32_t n = 0U;
    uint32_t value = secWhole;

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
    if ((s_state == T4_STATE_FINISHED) || (s_state == T4_STATE_TIME_STOPPED)) {
        s_uiStatusBuf[idx++] = ' ';
        s_uiStatusBuf[idx++] = 'D';
        s_uiStatusBuf[idx++] = 'O';
        s_uiStatusBuf[idx++] = 'N';
        s_uiStatusBuf[idx++] = 'E';
    }
    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}

void Task4_OnEnter(void)
{
    Pid_Init(&s_pid, T4_KP, T4_KI, T4_KD, T4_INTEGRAL_LIMIT, T4_MAX_STEER_RPM);
    s_state             = T4_STATE_RESET_DISABLE;
    s_lastSteerRpm      = 0.0F;
    s_leftRpm           = 0.0F;
    s_rightRpm          = 0.0F;
    s_filteredError     = 0.0F;
    s_sentLeftRpm       = 0;
    s_sentRightRpm      = 0;
    s_lineLostTicks     = 0U;
    s_enableSettleTicks = 0U;
    s_sendLeftNext      = true;
    s_finishArmed       = false;
    s_armTicks          = 0U;
    s_finishHitTicks    = 0U;
    s_elapsedTicks      = 0U;
    s_runTicks          = 0U;
}

void Task4_OnLoop(void)
{
    float rawError;
    float targetLeftRpm;
    float targetRightRpm;
    uint32_t hitCount;

    if ((s_state != T4_STATE_FINISHED) && (s_state != T4_STATE_TIME_STOPPED)) {
        s_elapsedTicks++;
    }

    if (s_state == T4_STATE_RESET_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_enableSettleTicks = T4_RESET_SETTLE_TICKS;
        s_state = T4_STATE_RESET_WAIT;
        return;
    }
    if (s_state == T4_STATE_RESET_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T4_STATE_ENABLE_LEFT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_enableSettleTicks = T4_ENABLE_SETTLE_TICKS;
        s_state = T4_STATE_ENABLE_LEFT_WAIT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_LEFT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T4_STATE_ENABLE_RIGHT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_enableSettleTicks = T4_ENABLE_SETTLE_TICKS;
        s_state = T4_STATE_ENABLE_RIGHT_WAIT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_RIGHT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T4_STATE_RUN;
    }

    /* 6.5 秒到：两拍分别给左右轮发送带加速度的速度模式 0 RPM，不能走急停接口。 */
    if (s_state == T4_STATE_TIME_STOP_LEFT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T4_STOP_EMM_ACC);
        s_sentLeftRpm = 0;
        s_state = T4_STATE_TIME_STOP_RIGHT;
        return;
    }
    if (s_state == T4_STATE_TIME_STOP_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T4_STOP_EMM_ACC);
        s_sentRightRpm = 0;
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        s_state = T4_STATE_TIME_STOPPED;
        RobotCore_NotifyTaskFinished(3U);
        return;
    }
    if (s_state == T4_STATE_FINISHED) {
        return;
    }

    /* 只统计正常循迹状态的前进时间；丢线停车期间不计入 6.5 秒。 */
    if (s_state == T4_STATE_RUN) {
        s_runTicks++;
        if ((s_runTicks * T4_TICK_MS) >= T4_STOP_AFTER_MS) {
            s_state = T4_STATE_TIME_STOP_LEFT;
            return;
        }
    }

    if (Task4_GetLineError(&rawError, &hitCount)) {
        if (s_state == T4_STATE_STOP) {
            s_filteredError = rawError;
        } else {
            s_filteredError += T4_ERROR_FILTER_ALPHA * (rawError - s_filteredError);
        }
        if (s_state == T4_STATE_STOP) {
            Pid_Reset(&s_pid);
        }
        /*
         * 0 RPM 缓停期间仍持续采样和更新 PID，避免状态机锁死后丢失循迹状态；
         * 但不能重新切回 RUN 并下发非零速度，否则会覆盖驱动器正在执行的 0 RPM 曲线。
         */
        if (s_state != T4_STATE_TIME_STOPPED) {
            s_state = T4_STATE_RUN;
        }
        s_lineLostTicks = 0U;
        {
            float pidError = s_filteredError;
            if ((pidError > -T4_ERROR_DEADBAND) && (pidError < T4_ERROR_DEADBAND)) {
                pidError = 0.0F;
            }
            s_lastSteerRpm = Pid_Update(&s_pid, pidError, T4_DT_SEC);
        }
    } else {
        s_lineLostTicks++;
        if (s_lineLostTicks >= T4_LINE_LOST_TICKS) {
            s_state = T4_STATE_STOP;
        }
    }

    if (!s_finishArmed) {
        if (hitCount <= T4_FINISH_ARM_HIT_MAX) {
            s_armTicks++;
            if (s_armTicks >= T4_FINISH_ARM_TICKS) {
                s_finishArmed = true;
            }
        } else {
            s_armTicks = 0U;
        }
    } else if (s_state != T4_STATE_FINISHED) {
        if (hitCount >= T4_FINISH_HIT_MIN) {
            s_finishHitTicks++;
            if (s_finishHitTicks >= T4_FINISH_HIT_TICKS) {
                s_state = T4_STATE_FINISHED;
            }
        } else {
            s_finishHitTicks = 0U;
        }
    }

    if (s_state == T4_STATE_FINISHED) {
        /* 终点保护沿用任务二：这是异常/终点保护，仍需要立即急停。 */
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
        vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        RobotCore_NotifyTaskFinished(3U);
        return;
    }

    if (s_state == T4_STATE_RUN) {
        float steerAbs = (s_lastSteerRpm >= 0.0F) ? s_lastSteerRpm : -s_lastSteerRpm;
        float dynBaseRpm = T4_BASE_RPM - T4_CORNER_SLOWDOWN_GAIN * steerAbs;
        dynBaseRpm = Task4_Clamp(dynBaseRpm, T4_MIN_BASE_RPM, T4_BASE_RPM);
        {
            uint32_t elapsedMs = Task4_GetElapsedMs();
            if (elapsedMs >= T4_DECEL_START_MS) {
                float decelSec = (float)(elapsedMs - T4_DECEL_START_MS) * 0.001F;
                float decelBaseRpm = T4_BASE_RPM - T4_DECEL_GRADIENT_RPM_PER_SEC * decelSec;
                decelBaseRpm = Task4_Clamp(decelBaseRpm, T4_DECEL_MIN_RPM, T4_BASE_RPM);
                if (decelBaseRpm < dynBaseRpm) {
                    dynBaseRpm = decelBaseRpm;
                }
            }
        }
        targetLeftRpm = dynBaseRpm + s_lastSteerRpm;
        targetRightRpm = dynBaseRpm - s_lastSteerRpm;
        Task4_ClampWheelPair(&targetLeftRpm, &targetRightRpm);
    } else {
        targetLeftRpm = 0.0F;
        targetRightRpm = 0.0F;
    }

    s_leftRpm = targetLeftRpm;
    s_rightRpm = targetRightRpm;
    if (s_state != T4_STATE_TIME_STOPPED) {
        Task4_ApplyWheelRpm(s_leftRpm, s_rightRpm);
    }
}

void Task4_OnExit(void)
{
    /* K4 退出保留任务二的硬安全收尾：急停后失能左右轮。 */
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);

    Pid_Reset(&s_pid);
    s_state = T4_STATE_STOP;
    s_finishArmed = false;
    s_armTicks = 0U;
    s_finishHitTicks = 0U;
    s_runTicks = 0U;
}
