#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "app_ball_control_task.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 4 题：6.5 秒循迹 PID 缓停
 *
 * 本题循迹、入场、丢线保护、终点保护和定时减速逻辑均照搬第 2 题，参数也
 * 保持同值。唯一业务区别是：进入正常循迹后累计前进 6.5 秒，分别给左右轮发送
 * 速度模式 0 RPM 帧；该帧带 T4_STOP_EMM_ACC 加速度档位，驱动器按曲线缓停。
 * 车辆行驶期间通过 BALLCTRL 同时控制 ID1，使钢珠持续保持在 X=320。
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

/* 正常循迹速度模式加速度档位，任务四独立调低以减小小车起步冲击。 */
#define T4_EMM_ACC                         (120U)

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
#define T4_EMM_CMD_GAP_MS                  (6U)

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

/*
 * 任务四钢珠平衡参数。算法与任务三第一阶段相同，但参数只归任务四所有，后续调车时
 * 不会影响菜单 K4 或任务三。车辆必须等后台闭环已经按此 profile 进入实际控制后才起步。
 */
#define T4_BALL_TARGET_X_PX                 (320)
#define T4_BALL_FILTER_ALPHA                (0.20F)   /* 关键：降低以抗噪声，防误判到位 */
#define T4_BALL_FILTER_BETA                 (0.05F)   /* 配合低 α，速度估计也放缓 */
#define T4_BALL_OUTPUT_SIGN                 (1.0F)
/* ---- 第一步：纯 P，Kv=0，只验证方向+静摩擦 ---- */
#define T4_BALL_KX_PULSE_PER_PX             (20.0F)
#define T4_BALL_KV_PULSE_PER_PXPS           (4.8F)    /* 先关掉速度阻尼 */
#define T4_BALL_LEVEL_TRIM_PULSE            (30)
#define T4_BALL_SETTLE_DEADBAND_PX          (12.0F)   /* 放宽：配合低 α 确保不误判 */
/*
 * 以下三个夹紧参数原为 5000/12/80，比菜单档 s_menuProfile（app_ball_control_task.c，
 * 已实测调好）更激进：STUCK_TIME 太短、STUCK_VELOCITY 太高会把球正常减速路过的
 * 低速瞬间误判成"卡住"，进而满幅夹紧把球推走；STICTION_PULSE 超出题六阶梯测试
 * 实测的静摩擦阈值(3200~4000)上限，夹紧命令本身就是过量的猛踹。三者叠加会让
 * 摆杆在"踹一下→球刚动就被判定不卡→命令掉回小P值→球停→再判卡住"之间持续摆动，
 * 出不了原位置；过量的踹也可能把球推过安全边界触发 FAULT_EDGE 死锁。现改回菜单档
 * 已验证的数值，详见 docs/CONTROL_ALGORITHM.md §5.3/§9.2。
 */
#define T4_BALL_STICTION_PULSE              (4000.0F)
#define T4_BALL_STUCK_VELOCITY_PXPS         (6.0F)
#define T4_BALL_STUCK_TIME_MS               (150U)
#define T4_BALL_POS_RPM                     (100U)
#define T4_BALL_POS_ACC                     (0U)
#define T4_BALL_HOLD_POSITION_PX            (6.0F)  /* 与死区一致 */
#define T4_BALL_HOLD_VELOCITY_PXPS          (10.0F)
#define T4_BALL_HOLD_TIME_MS                (500U)   /* 延长判定，确认真正稳定 */

/*
 * 手动调参开关：置 1 时电机不启动，仅 BALLCTRL 保持钢球平衡；
 * 用手推拉小车模拟加减速扰动，调好参数后改回 0 即可恢复完整功能。
 */
#define T4_MOTORS_DISABLED_MANUAL_TEST      (1U)

typedef enum {
    T4_STATE_WAIT_BALL_CONTROL = 0,
    T4_STATE_BALL_RECOVER_WAIT, /* 钢珠闭环触发 FAULT_EDGE 后，等待其停止/释放 ID1 再重新请求 */
    T4_STATE_MANUAL_BALANCE,   /* 电机不启动，仅后台 BALLCTRL 保持钢球平衡 */
    T4_STATE_RESET_DISABLE,
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

static const AppBallControlProfile_t s_task4BallProfile = {
    T4_BALL_FILTER_ALPHA,
    T4_BALL_FILTER_BETA,
    T4_BALL_OUTPUT_SIGN,
    T4_BALL_KX_PULSE_PER_PX,
    T4_BALL_KV_PULSE_PER_PXPS,
    T4_BALL_LEVEL_TRIM_PULSE,
    T4_BALL_SETTLE_DEADBAND_PX,
    T4_BALL_STICTION_PULSE,
    T4_BALL_STUCK_VELOCITY_PXPS,
    T4_BALL_STUCK_TIME_MS,
    T4_BALL_POS_RPM,
    T4_BALL_POS_ACC,
    T4_BALL_HOLD_POSITION_PX,
    T4_BALL_HOLD_VELOCITY_PXPS,
    T4_BALL_HOLD_TIME_MS
};

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
static bool         s_ballControlRequested;
static bool         s_wheelsStarted;   /* 轮子是否已经完成过一次使能起步（钢珠故障恢复后据此跳过重复使能） */
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

    if (s_state == T4_STATE_WAIT_BALL_CONTROL) {
        return "T4 B WAIT";
    }
    if (s_state == T4_STATE_BALL_RECOVER_WAIT) {
        return "T4 B RECOV";
    }
#if T4_MOTORS_DISABLED_MANUAL_TEST
    if (s_state == T4_STATE_MANUAL_BALANCE) {
        return "T4 MANUAL";
    }
#endif

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
    s_state             = T4_STATE_WAIT_BALL_CONTROL;
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
    s_ballControlRequested = false;
    s_wheelsStarted     = false;
}

void Task4_OnLoop(void)
{
    float rawError;
    float targetLeftRpm;
    float targetRightRpm;
    uint32_t hitCount;
    AppBallControlStatus_t ballStatus;

    AppBallControl_GetStatus(&ballStatus);

    if ((s_state != T4_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T4_STATE_BALL_RECOVER_WAIT) &&
        (s_state != T4_STATE_MANUAL_BALANCE) &&
        (s_state != T4_STATE_FINISHED) && (s_state != T4_STATE_TIME_STOPPED)) {
        s_elapsedTicks++;
    }

    /*
     * 钢珠闭环触发 FAULT_EDGE（球触边）后会一直锁在回水平的位置，不会自己恢复，
     * 必须重新走一遍"停止→等待释放→重新请求"的握手才能恢复到 X=320；否则表现
     * 就是"进了任务四但钢珠不再被伺服"。这里主动检测并恢复，不需要用户手动
     * 退出重进。只要不是正在做这套握手本身，任何时候（含 RUN/STOP/已完成等待
     * K4 退出期间）检测到故障都立即触发恢复。
     */
    if ((s_state != T4_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T4_STATE_BALL_RECOVER_WAIT) &&
        (ballStatus.state == APP_BALL_CONTROL_FAULT_EDGE)) {
        AppBallControl_RequestStop();
        s_ballControlRequested = false;
        s_state = T4_STATE_BALL_RECOVER_WAIT;
    }

    /* 等 BALLCTRL 真正回到 OFF 才能重新请求，避免与刚发出的停止命令产生竞态。 */
    if (s_state == T4_STATE_BALL_RECOVER_WAIT) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) {
            s_state = T4_STATE_WAIT_BALL_CONTROL;
        }
        return;
    }

    /* 先确保 ID1 已按任务四专属参数开始闭环，随后才允许车辆使能和起步。 */
    if (s_state == T4_STATE_WAIT_BALL_CONTROL) {
        if (!s_ballControlRequested) {
            s_ballControlRequested = AppBallControl_RequestTargetWithProfile(
                T4_BALL_TARGET_X_PX, &s_task4BallProfile);
        }

        if (s_ballControlRequested &&
            (ballStatus.targetPx == T4_BALL_TARGET_X_PX) &&
            ((ballStatus.state == APP_BALL_CONTROL_RUNNING) ||
             (ballStatus.state == APP_BALL_CONTROL_HOLDING))) {
            if (s_wheelsStarted) {
                /* 钢珠故障恢复场景：轮子早已在跑，直接回到循迹，不重新走使能时序。 */
                s_state = T4_STATE_RUN;
            } else {
#if T4_MOTORS_DISABLED_MANUAL_TEST
                s_state = T4_STATE_MANUAL_BALANCE;
#else
                s_wheelsStarted = true;
                s_state = T4_STATE_RESET_DISABLE;
#endif
            }
        }
        return;
    }

#if T4_MOTORS_DISABLED_MANUAL_TEST
    /* 手动调参模式：电机不使能、不驱动，仅后台 BALLCTRL 保持钢球 X=320。*/
    if (s_state == T4_STATE_MANUAL_BALANCE) {
        return;
    }
#endif

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

    /* 任务四结束后才释放 ID1，运行和缓停期间保持钢珠 X=320 的位置目标。 */
    AppBallControl_RequestStop();

    Pid_Reset(&s_pid);
    s_state = T4_STATE_STOP;
    s_finishArmed = false;
    s_armTicks = 0U;
    s_finishHitTicks = 0U;
    s_runTicks = 0U;
    s_ballControlRequested = false;
    s_wheelsStarted = false;
}
