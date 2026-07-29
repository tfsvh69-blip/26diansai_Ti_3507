#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "bsp_buzzer.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 5 题：横向停止线后 0.5 秒循迹并线性缓停
 *
 * 以题目二的 8 路灰度 PID 为基础，行驶 RPM 参数按 62% 缩放。完成细线
 * 武装后首次命中 >=6 路黑线时，横向黑胶带期间保持触发前的轮速，离开横带后
 * 继续循迹，累计 0.5 秒后用 2 秒软件线性减速到 0 RPM。
 * ================================================================== */

/* PID 参数沿用题目二；位置误差仍为 -7、-5、...、+7。 */
#define T5_KP                              (5.0F)
#define T5_KI                              (0.15F)
#define T5_KD                              (0.2F)
#define T5_INTEGRAL_LIMIT                  (20.0F)
#define T5_MAX_STEER_RPM                   (62.0F)

/* 题目二行驶 RPM 参数按 62% 缩放。 */
#define T5_BASE_RPM                        (80.6F)
#define T5_MIN_WHEEL_RPM                   (5.0F)
#define T5_MAX_WHEEL_RPM                   (142.6F)
#define T5_MIN_BASE_RPM                    (6.2F)

/* 保持题目二的驱动器加速度档位与控制节拍。 */
#define T5_EMM_ACC                         (150U)
#define T5_DT_SEC                          (0.03F)
#define T5_TICK_MS                         (30U)

/* 丢线保护、入场使能与共享 UART1 总线时序。 */
#define T5_LINE_LOST_TICKS                 (20U)
#define T5_RESET_SETTLE_TICKS              (2U)
#define T5_ENABLE_SETTLE_TICKS             (6U)
#define T5_EMM_CMD_GAP_MS                  (5U)

/* 抗抖与转弯减速参数沿用题目二。 */
#define T5_ERROR_FILTER_ALPHA              (0.5F)
#define T5_ERROR_DEADBAND                  (1.0F)
#define T5_CORNER_SLOWDOWN_GAIN            (0.6F)

/* 先连续识别细线，再允许宽横线触发后续流程。 */
#define T5_ARM_HIT_MAX                     (3U)
#define T5_ARM_TICKS                       (15U)
#define T5_STOP_LINE_HIT_MIN               (6U)

/* 首次压到横线后的正常循迹时间与软件线性减速时长。 */
#define T5_AFTER_LINE_MS                   (500U)
#define T5_DECEL_MS                        (2000U)

/* 与任务二采用同一实测初值；任务五可独立按实测比例继续修正。 */
#define T5_STOPWATCH_CAL_SCALE             (0.897F)

typedef enum {
    T5_STATE_RESET_DISABLE = 0,
    T5_STATE_RESET_WAIT,
    T5_STATE_ENABLE_LEFT,
    T5_STATE_ENABLE_LEFT_WAIT,
    T5_STATE_ENABLE_RIGHT,
    T5_STATE_ENABLE_RIGHT_WAIT,
    T5_STATE_RUN,
    T5_STATE_AFTER_LINE,
    T5_STATE_DECEL,
    T5_STATE_STOP_LEFT,
    T5_STATE_STOP_RIGHT,
    T5_STATE_FINISHED
} Task5State_t;

static Task5State_t s_state;
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
static bool         s_lineStopped;
static bool         s_holdingStopLine;
static uint32_t     s_afterLineElapsedMs;
static uint32_t     s_decelElapsedMs;
static uint32_t     s_elapsedTicks;
static char         s_uiStatusBuf[24];
static char         s_phaseStatusBuf[12];

static float Task5_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

/* 正常循迹保留差速的整体平移限幅。 */
static void Task5_ClampWheelPair(float *leftRpm, float *rightRpm)
{
    float hi = (*leftRpm > *rightRpm) ? *leftRpm : *rightRpm;
    float lo;
    float shift;

    if (hi > T5_MAX_WHEEL_RPM) {
        shift      = hi - T5_MAX_WHEEL_RPM;
        *leftRpm  -= shift;
        *rightRpm -= shift;
    }

    lo = (*leftRpm < *rightRpm) ? *leftRpm : *rightRpm;
    if (lo < T5_MIN_WHEEL_RPM) {
        shift      = T5_MIN_WHEEL_RPM - lo;
        *leftRpm  += shift;
        *rightRpm += shift;
    }

    *leftRpm  = Task5_Clamp(*leftRpm, T5_MIN_WHEEL_RPM, T5_MAX_WHEEL_RPM);
    *rightRpm = Task5_Clamp(*rightRpm, T5_MIN_WHEEL_RPM, T5_MAX_WHEEL_RPM);
}

/* 正常控制每拍最多发一帧，左右轮交替更新。 */
static void Task5_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t leftInt = (int32_t)(leftRpm + 0.5F);
    int32_t rightInt = (int32_t)(rightRpm + 0.5F);

    if (s_sendLeftNext) {
        if (leftInt != s_sentLeftRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_L, (int16_t)leftInt,
                                   T5_EMM_ACC);
            s_sentLeftRpm = leftInt;
        }
    } else {
        if (rightInt != s_sentRightRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_R, (int16_t)rightInt,
                                   T5_EMM_ACC);
            s_sentRightRpm = rightInt;
        }
    }
    s_sendLeftNext = !s_sendLeftNext;
}

/* 按物理左→右的 LINE8→LINE1 顺序计算线位置误差。 */
static bool Task5_GetLineError(float *error, uint32_t *hitCountOut)
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

/* 有效线数据才更新滤波和 PID；长期丢线时保留题目二的安全停车。 */
static void Task5_UpdateTracking(bool lineFound, float rawError)
{
    if (lineFound) {
        if (s_lineStopped) {
            s_filteredError = rawError;
            Pid_Reset(&s_pid);
        } else {
            s_filteredError += T5_ERROR_FILTER_ALPHA * (rawError - s_filteredError);
        }

        s_lineLostTicks = 0U;
        s_lineStopped = false;
        {
            float pidError = s_filteredError;
            if ((pidError > -T5_ERROR_DEADBAND) && (pidError < T5_ERROR_DEADBAND)) {
                pidError = 0.0F;
            }
            s_lastSteerRpm = Pid_Update(&s_pid, pidError, T5_DT_SEC);
        }
    } else {
        s_lineLostTicks++;
        if (s_lineLostTicks >= T5_LINE_LOST_TICKS) {
            s_lineStopped = true;
        }
    }
}

/* 正常循迹目标：转弯时降低基础速度，并保持差速整体限幅。 */
static void Task5_ApplyNormalTracking(void)
{
    float steerAbs;
    float dynBaseRpm;
    float targetLeftRpm;
    float targetRightRpm;

    if (s_lineStopped) {
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);
        return;
    }

    steerAbs = (s_lastSteerRpm >= 0.0F) ? s_lastSteerRpm : -s_lastSteerRpm;
    dynBaseRpm = T5_BASE_RPM - T5_CORNER_SLOWDOWN_GAIN * steerAbs;
    dynBaseRpm = Task5_Clamp(dynBaseRpm, T5_MIN_BASE_RPM, T5_BASE_RPM);
    targetLeftRpm = dynBaseRpm + s_lastSteerRpm;
    targetRightRpm = dynBaseRpm - s_lastSteerRpm;
    Task5_ClampWheelPair(&targetLeftRpm, &targetRightRpm);

    s_leftRpm = targetLeftRpm;
    s_rightRpm = targetRightRpm;
    Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);
}

/* 减速期允许的差速随基础速度同步收窄，保证两轮始终非负并能降到 0 RPM。 */
static void Task5_ApplyDecelTracking(void)
{
    float progress;
    float linearBaseRpm;
    float steerAbs;
    float dynBaseRpm;
    float steerLimit;
    float steerRpm;
    float targetLeftRpm;
    float targetRightRpm;

    progress = (float)s_decelElapsedMs / (float)T5_DECEL_MS;
    progress = Task5_Clamp(progress, 0.0F, 1.0F);
    linearBaseRpm = T5_BASE_RPM * (1.0F - progress);

    steerAbs = (s_lastSteerRpm >= 0.0F) ? s_lastSteerRpm : -s_lastSteerRpm;
    dynBaseRpm = linearBaseRpm - T5_CORNER_SLOWDOWN_GAIN * steerAbs;
    dynBaseRpm = Task5_Clamp(dynBaseRpm, 0.0F, linearBaseRpm);

    steerLimit = dynBaseRpm;
    if ((T5_MAX_WHEEL_RPM - dynBaseRpm) < steerLimit) {
        steerLimit = T5_MAX_WHEEL_RPM - dynBaseRpm;
    }
    steerLimit = Task5_Clamp(steerLimit, 0.0F, T5_MAX_STEER_RPM);
    steerRpm = Task5_Clamp(s_lastSteerRpm, -steerLimit, steerLimit);

    targetLeftRpm = dynBaseRpm + steerRpm;
    targetRightRpm = dynBaseRpm - steerRpm;
    s_leftRpm = Task5_Clamp(targetLeftRpm, 0.0F, T5_MAX_WHEEL_RPM);
    s_rightRpm = Task5_Clamp(targetRightRpm, 0.0F, T5_MAX_WHEEL_RPM);
    Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);
}

static void Task5_FormatRemaining(const char *prefix, uint32_t elapsedMs)
{
    uint32_t remainingMs;
    uint32_t tenths;
    uint32_t idx = 0U;
    uint32_t durationMs = (s_state == T5_STATE_AFTER_LINE) ? T5_AFTER_LINE_MS
                                                             : T5_DECEL_MS;

    if (elapsedMs >= durationMs) {
        remainingMs = 0U;
    } else {
        remainingMs = durationMs - elapsedMs;
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

/* 秒表校准后的累计毫秒数；与 OLED 显示使用同一套换算。 */
static uint32_t Task5_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T5_TICK_MS) * T5_STOPWATCH_CAL_SCALE);
}

static const char *Task5_GetPhaseStatus(void)
{
    switch (s_state) {
    case T5_STATE_RUN:
        if (s_lineStopped) {
            return "LOST";
        }
        return s_finishArmed ? "LINE" : "ARM";

    case T5_STATE_AFTER_LINE:
        if (s_holdingStopLine) {
            return "HOLD";
        }
        if (s_lineStopped) {
            return "LOST";
        }
        Task5_FormatRemaining("GO:", s_afterLineElapsedMs);
        return s_phaseStatusBuf;

    case T5_STATE_DECEL:
        if (s_holdingStopLine) {
            return "HOLD";
        }
        Task5_FormatRemaining("DEC:", s_decelElapsedMs);
        return s_phaseStatusBuf;

    case T5_STATE_STOP_LEFT:
    case T5_STATE_STOP_RIGHT:
        return "STOP";

    case T5_STATE_FINISHED:
        return "DONE";

    default:
        return "INIT";
    }
}

/* 秒表文本："T:12.3s GO:0.5s"，到停车完成后定格并追加 "DONE"。 */
const char *Task5_GetUiStatus(void)
{
    const char *phase = Task5_GetPhaseStatus();
    uint32_t totalMs = Task5_GetElapsedMs();
    uint32_t secWhole = totalMs / 1000U;
    uint32_t tenths = (totalMs / 100U) % 10U;
    uint32_t idx = 0U;
    uint32_t n = 0U;
    uint32_t value = secWhole;
    char digits[10];

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

void Task5_OnEnter(void)
{
    Pid_Init(&s_pid, T5_KP, T5_KI, T5_KD,
             T5_INTEGRAL_LIMIT, T5_MAX_STEER_RPM);
    s_state = T5_STATE_RESET_DISABLE;
    s_lastSteerRpm = 0.0F;
    s_leftRpm = 0.0F;
    s_rightRpm = 0.0F;
    s_filteredError = 0.0F;
    s_sentLeftRpm = 0;
    s_sentRightRpm = 0;
    s_lineLostTicks = 0U;
    s_enableSettleTicks = 0U;
    s_sendLeftNext = true;
    s_finishArmed = false;
    s_armTicks = 0U;
    s_lineStopped = false;
    s_holdingStopLine = false;
    s_afterLineElapsedMs = 0U;
    s_decelElapsedMs = 0U;
    s_elapsedTicks = 0U;
}

void Task5_OnLoop(void)
{
    float rawError;
    bool lineFound;
    uint32_t hitCount;

    /* 从进入本题开始计时，停车完成后定格。 */
    if (s_state != T5_STATE_FINISHED) {
        s_elapsedTicks++;
    }

    if (s_state == T5_STATE_RESET_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_enableSettleTicks = T5_RESET_SETTLE_TICKS;
        s_state = T5_STATE_RESET_WAIT;
        return;
    }
    if (s_state == T5_STATE_RESET_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T5_STATE_ENABLE_LEFT;
        return;
    }
    if (s_state == T5_STATE_ENABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_enableSettleTicks = T5_ENABLE_SETTLE_TICKS;
        s_state = T5_STATE_ENABLE_LEFT_WAIT;
        return;
    }
    if (s_state == T5_STATE_ENABLE_LEFT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T5_STATE_ENABLE_RIGHT;
        return;
    }
    if (s_state == T5_STATE_ENABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_enableSettleTicks = T5_ENABLE_SETTLE_TICKS;
        s_state = T5_STATE_ENABLE_RIGHT_WAIT;
        return;
    }
    if (s_state == T5_STATE_ENABLE_RIGHT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T5_STATE_RUN;
    }

    if (s_state == T5_STATE_STOP_LEFT) {
        /* 线性减速已到 0，分两拍保留速度模式 0 RPM 的停车语义。 */
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T5_EMM_ACC);
        s_sentLeftRpm = 0;
        s_state = T5_STATE_STOP_RIGHT;
        return;
    }
    if (s_state == T5_STATE_STOP_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T5_EMM_ACC);
        s_sentRightRpm = 0;
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        s_state = T5_STATE_FINISHED;
        /* 自动停车完成时与按键共用同一短促提示音。 */
        BspBuzzer_BeepShort();
        RobotCore_NotifyTaskFinished(4U);
        return;
    }
    if (s_state == T5_STATE_FINISHED) {
        return;
    }

    lineFound = Task5_GetLineError(&rawError, &hitCount);

    if (s_state == T5_STATE_RUN) {
        /* 宽横线首次触发时立即冻结最后的正常速度帧，不让横带误差进入 PID。 */
        if (s_finishArmed && (hitCount >= T5_STOP_LINE_HIT_MIN)) {
            s_state = T5_STATE_AFTER_LINE;
            s_holdingStopLine = true;
            s_afterLineElapsedMs = 0U;
            return;
        }

        s_holdingStopLine = false;
        Task5_UpdateTracking(lineFound, rawError);
        if (!s_finishArmed) {
            if (hitCount <= T5_ARM_HIT_MAX) {
                s_armTicks++;
                if (s_armTicks >= T5_ARM_TICKS) {
                    s_finishArmed = true;
                }
            } else {
                s_armTicks = 0U;
            }
        }
        Task5_ApplyNormalTracking();
        return;
    }

    if (s_state == T5_STATE_AFTER_LINE) {
        /* 横带出现后计时不中断；横带仍在传感器下方时保持最后轮速。 */
        s_holdingStopLine = (hitCount >= T5_STOP_LINE_HIT_MIN);
        s_afterLineElapsedMs += T5_TICK_MS;
        if (s_afterLineElapsedMs >= T5_AFTER_LINE_MS) {
            s_state = T5_STATE_DECEL;
            s_decelElapsedMs = 0U;
            return;
        }
        if (s_holdingStopLine) {
            return;
        }

        Task5_UpdateTracking(lineFound, rawError);
        Task5_ApplyNormalTracking();
        return;
    }

    /* 减速期再次压到横带时，冻结轮速并暂停减速计时，直到重新回到细线。 */
    s_holdingStopLine = (hitCount >= T5_STOP_LINE_HIT_MIN);
    if (s_holdingStopLine) {
        return;
    }

    Task5_UpdateTracking(lineFound, rawError);
    if (s_lineStopped) {
        Task5_ApplyNormalTracking();
    } else {
        Task5_ApplyDecelTracking();
    }

    s_decelElapsedMs += T5_TICK_MS;
    if (s_decelElapsedMs >= T5_DECEL_MS) {
        s_state = T5_STATE_STOP_LEFT;
    }
}

void Task5_OnExit(void)
{
    /* K4 退出保留题目二的硬安全收尾：急停后失能左右轮。 */
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);

    Pid_Reset(&s_pid);
    s_state = T5_STATE_RUN;
    s_finishArmed = false;
    s_armTicks = 0U;
    s_lineStopped = false;
    s_holdingStopLine = false;
    s_afterLineElapsedMs = 0U;
    s_decelElapsedMs = 0U;
    s_elapsedTicks = 0U;
}
