#include "app_ball_control_task.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "alpha_beta_filter.h"
#include "app_config.h"
#include "app_vision_link.h"
#include "emm42_robot.h"

/* ==================================================================
 * 钢球 X 位置后台闭环
 *
 * 数据流：视觉 X → α-β 位置/速度估计 → 局部 PID → ID1 速度模式。
 * 本线程只控制 EMM42_ROBOT_LIFT，不触碰 ID2/ID3；题目层通过
 * AppBallControl_RequestTarget() 传目标，不直接共享内部变量。
 *
 * I 项只在低速小误差阶段补偿静摩擦，并带独立输出限幅及清零条件。
 * 参数确认顺序应为输出方向 → Kp → Kd → Ki，I 不能替代摆杆角度反馈。
 * ================================================================== */

/*
 * 视觉约 15Hz，即名义帧间隔 66.7ms。130ms 未收到有效 X 先软降级，
 * 220ms（约 3.3 帧）仍无有效 X 才进入 LOST 并急停。
 */
#define BALL_CTRL_VISION_DEGRADE_MS       (130U)
#define BALL_CTRL_VISION_TIMEOUT_MS       (220U)
#define BALL_CTRL_NA_LOST_COUNT           (2U)
#define BALL_CTRL_RECOVERY_VALID_COUNT    (2U)
#define BALL_CTRL_DEGRADE_STEP_PERIOD_MS  (20U)
#define BALL_CTRL_DEGRADE_RPM_STEP        (2)
#define BALL_CTRL_DT_MIN_SEC              (0.030F)
#define BALL_CTRL_DT_MAX_SEC              (0.200F)

/* 沿用题目二/三实测稳定的 Emm42 复位与使能等待。 */
#define BALL_CTRL_RESET_SETTLE_MS         (60U)
#define BALL_CTRL_ENABLE_SETTLE_MS        (180U)

/* α-β 滤波：先压制像素抖动，同时保留足够的速度响应。 */
#define BALL_CTRL_FILTER_ALPHA            (0.70F)
#define BALL_CTRL_FILTER_BETA             (0.08F)

/*
 * 局部 PID 输出单位为 RPM：
 *   rpm = OUTPUT_SIGN × (Kp × errorForP - Kd × velocity + Ki × integralError)
 *
 * ID1 正 RPM 已标定为“连杆向下”，但连杆位于摆杆哪一端决定钢球 X 的响应方向。
 * 首次上车必须先用小偏差确认；若越控越远，只改 OUTPUT_SIGN 的正负。
 */
#define BALL_CTRL_OUTPUT_SIGN             (1.0F)
#define BALL_CTRL_KP_RPM_PER_PX           (0.1F)
#define BALL_CTRL_KD_RPM_PER_PXPS         (1.6F)
/* I 项只补偿低速小误差区的静摩擦，不能替代摆杆位置反馈。 */
#define BALL_CTRL_KI_RPM_PER_PXSEC         (0.5F)
#define BALL_CTRL_I_OUTPUT_LIMIT_RPM       (5.0F)
#define BALL_CTRL_I_DEADBAND_PX            (1.0F)
#define BALL_CTRL_I_ENABLE_ERROR_PX        (12.0F)
#define BALL_CTRL_I_ENABLE_VELOCITY_PXPS   (10.0F)
/* 速度模式加速度为 0，PID 输出直接生效，不叠加软件或驱动器速度斜坡。 */
#define BALL_CTRL_EMM_ACC                 (0U)

/* 中心保持判定与安全边界。 */
#define BALL_CTRL_POSITION_DEADBAND_PX    (4.0F)
#define BALL_CTRL_HOLD_POSITION_PX        (5.0F)
#define BALL_CTRL_HOLD_VELOCITY_PXPS      (10.0F)
#define BALL_CTRL_HOLD_TIME_MS            (500U)
#define BALL_CTRL_SAFE_X_MIN_PX           (20)
#define BALL_CTRL_SAFE_X_MAX_PX           (620)

/* 首次启动方向保护：同侧误差连续扩大到初值 +25px 时停止并提示 DIR。 */
#define BALL_CTRL_DIR_CHECK_MIN_ERROR_PX  (20.0F)
#define BALL_CTRL_DIR_CHECK_IMPROVE_PX    (10.0F)
#define BALL_CTRL_DIR_CHECK_GROW_PX       (25.0F)
#define BALL_CTRL_DIR_CHECK_BAD_SAMPLES   (3U)

typedef enum {
    BALL_CTRL_INTERNAL_OFF = 0,
    BALL_CTRL_INTERNAL_RESET_DISABLE,
    BALL_CTRL_INTERNAL_RESET_WAIT,
    BALL_CTRL_INTERNAL_ENABLE,
    BALL_CTRL_INTERNAL_ENABLE_WAIT,
    BALL_CTRL_INTERNAL_WAIT_VISION,
    BALL_CTRL_INTERNAL_ACTIVE,
    BALL_CTRL_INTERNAL_DEGRADED,
    BALL_CTRL_INTERNAL_LOST,
    BALL_CTRL_INTERNAL_STOP,
    BALL_CTRL_INTERNAL_DISABLE,
    BALL_CTRL_INTERNAL_FAULT_DIRECTION,
    BALL_CTRL_INTERNAL_FAULT_EDGE
} BallControlInternalState_t;

typedef struct {
    bool enable;
    int16_t targetPx;
} BallControlCommand_t;

static TaskHandle_t s_taskHandle;
static QueueHandle_t s_commandQueue;
static AppBallControlStatus_t s_publicStatus;

static float BallControl_Abs(float value)
{
    return (value >= 0.0F) ? value : -value;
}

static float BallControl_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static bool BallControl_Elapsed(TickType_t now, TickType_t then, uint32_t timeoutMs)
{
    return (TickType_t)(now - then) >= pdMS_TO_TICKS(timeoutMs);
}

static bool BallControl_SameSign(float a, float b)
{
    return ((a >= 0.0F) && (b >= 0.0F)) || ((a < 0.0F) && (b < 0.0F));
}

static bool BallControl_CrossedZero(float previous, float current)
{
    return ((previous > 0.0F) && (current <= 0.0F)) ||
           ((previous < 0.0F) && (current >= 0.0F));
}

static float BallControl_GetIntegralErrorLimit(void)
{
    if (BALL_CTRL_KI_RPM_PER_PXSEC <= 0.0F) {
        return 0.0F;
    }

    return BALL_CTRL_I_OUTPUT_LIMIT_RPM / BALL_CTRL_KI_RPM_PER_PXSEC;
}

static int16_t BallControl_RoundRpm(float value)
{
    if (value >= 0.0F) {
        return (int16_t)(value + 0.5F);
    }
    return (int16_t)(value - 0.5F);
}

static int16_t BallControl_StepRpmTowardZero(int16_t commandRpm)
{
    if (commandRpm > BALL_CTRL_DEGRADE_RPM_STEP) {
        return commandRpm - BALL_CTRL_DEGRADE_RPM_STEP;
    }
    if (commandRpm < -BALL_CTRL_DEGRADE_RPM_STEP) {
        return commandRpm + BALL_CTRL_DEGRADE_RPM_STEP;
    }
    return 0;
}

static AppBallControlState_t BallControl_ToPublicState(BallControlInternalState_t state,
                                                       bool holding)
{
    switch (state) {
    case BALL_CTRL_INTERNAL_OFF:
        return APP_BALL_CONTROL_OFF;
    case BALL_CTRL_INTERNAL_RESET_DISABLE:
    case BALL_CTRL_INTERNAL_RESET_WAIT:
    case BALL_CTRL_INTERNAL_ENABLE:
    case BALL_CTRL_INTERNAL_ENABLE_WAIT:
        return APP_BALL_CONTROL_STARTING;
    case BALL_CTRL_INTERNAL_WAIT_VISION:
        return APP_BALL_CONTROL_WAIT_VISION;
    case BALL_CTRL_INTERNAL_ACTIVE:
        return holding ? APP_BALL_CONTROL_HOLDING : APP_BALL_CONTROL_RUNNING;
    case BALL_CTRL_INTERNAL_DEGRADED:
        return APP_BALL_CONTROL_DEGRADED;
    case BALL_CTRL_INTERNAL_LOST:
        return APP_BALL_CONTROL_LOST;
    case BALL_CTRL_INTERNAL_FAULT_DIRECTION:
        return APP_BALL_CONTROL_FAULT_DIRECTION;
    case BALL_CTRL_INTERNAL_FAULT_EDGE:
        return APP_BALL_CONTROL_FAULT_EDGE;
    case BALL_CTRL_INTERNAL_STOP:
    case BALL_CTRL_INTERNAL_DISABLE:
    default:
        return APP_BALL_CONTROL_STARTING;
    }
}

static void BallControl_Publish(BallControlInternalState_t state, bool holding,
                                int16_t targetPx, int16_t measuredPx,
                                const AlphaBetaFilter_t *filter, int16_t commandRpm,
                                float integralRpm, uint32_t sampleSeq)
{
    taskENTER_CRITICAL();
    s_publicStatus.state = BallControl_ToPublicState(state, holding);
    s_publicStatus.targetPx = targetPx;
    s_publicStatus.measuredPx = measuredPx;
    s_publicStatus.filteredPx = filter->position;
    s_publicStatus.velocityPxPerSec = filter->velocity;
    s_publicStatus.commandRpm = commandRpm;
    s_publicStatus.integralRpm = integralRpm;
    s_publicStatus.sampleSeq = sampleSeq;
    taskEXIT_CRITICAL();
}

static void AppBallControlTask_Entry(void *argument)
{
    BallControlInternalState_t state = BALL_CTRL_INTERNAL_OFF;
    BallControlCommand_t command;
    AppVisionXSample_t vision;
    AlphaBetaFilter_t filter;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t stateStart = lastWake;
    TickType_t lastValidSampleTick = 0U;
    TickType_t holdStartTick = 0U;
    TickType_t degradeStepTick = 0U;
    uint32_t seenSampleSeq = 0U;
    uint32_t consecutiveNaCount = 0U;
    uint32_t recoveryValidCount = 0U;
    int16_t targetPx = APP_BALL_CONTROL_CENTER_X_PX;
    int16_t measuredPx = 0;
    int16_t commandRpm = 0;
    float integralErrorPxSec = 0.0F;
    float integralRpm = 0.0F;
    float previousRawError = 0.0F;
    float directionInitialError = 0.0F;
    uint32_t directionBadSamples = 0U;
    bool directionCheckActive = false;
    bool previousRawErrorValid = false;
    bool holding = false;

    (void)argument;
    AlphaBetaFilter_Init(&filter, BALL_CTRL_FILTER_ALPHA, BALL_CTRL_FILTER_BETA);
    BallControl_Publish(state, holding, targetPx, measuredPx, &filter,
                        commandRpm, integralRpm, seenSampleSeq);

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* 长度为 1 的覆盖队列只保留调用方最新意图。 */
        if (xQueueReceive(s_commandQueue, &command, 0U) == pdPASS) {
            targetPx = command.targetPx;
            holding = false;
            holdStartTick = 0U;
            /* 目标变更或 K4 启停后，旧目标的积分偏置不能保留。 */
            integralErrorPxSec = 0.0F;
            integralRpm = 0.0F;
            previousRawErrorValid = false;

            if (command.enable) {
                if (state == BALL_CTRL_INTERNAL_OFF) {
                    AlphaBetaFilter_Reset(&filter);
                    seenSampleSeq = 0U;
                    lastValidSampleTick = 0U;
                    measuredPx = 0;
                    commandRpm = 0;
                    integralErrorPxSec = 0.0F;
                    integralRpm = 0.0F;
                    consecutiveNaCount = 0U;
                    recoveryValidCount = 0U;
                    degradeStepTick = 0U;
                    directionCheckActive = false;
                    directionBadSamples = 0U;
                    state = BALL_CTRL_INTERNAL_RESET_DISABLE;
                }
                /* 已运行时更新 targetPx 即可，不重做电机使能时序。 */
            } else if (state != BALL_CTRL_INTERNAL_OFF) {
                state = BALL_CTRL_INTERNAL_STOP;
            }
        }

        switch (state) {
        case BALL_CTRL_INTERNAL_OFF:
            break;

        case BALL_CTRL_INTERNAL_RESET_DISABLE:
            Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
            stateStart = now;
            state = BALL_CTRL_INTERNAL_RESET_WAIT;
            break;

        case BALL_CTRL_INTERNAL_RESET_WAIT:
            if (BallControl_Elapsed(now, stateStart, BALL_CTRL_RESET_SETTLE_MS)) {
                state = BALL_CTRL_INTERNAL_ENABLE;
            }
            break;

        case BALL_CTRL_INTERNAL_ENABLE:
            Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
            stateStart = now;
            state = BALL_CTRL_INTERNAL_ENABLE_WAIT;
            break;

        case BALL_CTRL_INTERNAL_ENABLE_WAIT:
            if (BallControl_Elapsed(now, stateStart, BALL_CTRL_ENABLE_SETTLE_MS)) {
                state = BALL_CTRL_INTERNAL_WAIT_VISION;
            }
            break;

        case BALL_CTRL_INTERNAL_WAIT_VISION:
        case BALL_CTRL_INTERNAL_DEGRADED:
        case BALL_CTRL_INTERNAL_LOST:
        case BALL_CTRL_INTERNAL_ACTIVE:
            AppVisionLink_GetLatestX(&vision);

            if (vision.sequence != seenSampleSeq) {
                seenSampleSeq = vision.sequence;

                if (vision.na) {
                    consecutiveNaCount++;
                    recoveryValidCount = 0U;
                    holding = false;
                    holdStartTick = 0U;
                    /* 丢球后不允许旧积分继续推动摆杆。 */
                    integralErrorPxSec = 0.0F;
                    integralRpm = 0.0F;
                    previousRawErrorValid = false;
                    if (lastValidSampleTick == 0U) {
                        /*
                         * 启动后尚未收到过有效 X 时，以首个 NA 作为丢失计时起点，
                         * 防止只收到一帧 NA 后链路中断而永久停留在 DEG。
                         */
                        lastValidSampleTick = now;
                    }

                    if ((consecutiveNaCount >= BALL_CTRL_NA_LOST_COUNT) &&
                        (state != BALL_CTRL_INTERNAL_LOST)) {
                        /* 连续两帧 NA 才判定持续丢球，急停后保持 ID1 使能。 */
                        Emm42Robot_Stop(EMM42_ROBOT_LIFT);
                        commandRpm = 0;
                        AlphaBetaFilter_Reset(&filter);
                        state = BALL_CTRL_INTERNAL_LOST;
                    } else if (state != BALL_CTRL_INTERNAL_LOST) {
                        /*
                         * 单次 NA 只进入软降级，保留滤波状态并逐步把速度降到零，
                         * 避免偶发漏检造成频繁急停，也不继续沿用非零速度。
                         */
                        state = BALL_CTRL_INTERNAL_DEGRADED;
                        degradeStepTick = now;
                    }
                } else if (vision.valid) {
                    float dtSec;
                    float rawError;
                    float errorForP;
                    float output;
                    float absError;
                    float integralLimit;
                    bool resetIntegral;

                    measuredPx = vision.pixel;
                    if ((measuredPx <= BALL_CTRL_SAFE_X_MIN_PX) ||
                        (measuredPx >= BALL_CTRL_SAFE_X_MAX_PX)) {
                        Emm42Robot_Stop(EMM42_ROBOT_LIFT);
                        commandRpm = 0;
                        integralErrorPxSec = 0.0F;
                        integralRpm = 0.0F;
                        previousRawErrorValid = false;
                        state = BALL_CTRL_INTERNAL_FAULT_EDGE;
                        break;
                    }

                    consecutiveNaCount = 0U;

                    if ((state == BALL_CTRL_INTERNAL_WAIT_VISION) ||
                        (state == BALL_CTRL_INTERNAL_LOST) ||
                        (state == BALL_CTRL_INTERNAL_DEGRADED)) {
                        integralErrorPxSec = 0.0F;
                        integralRpm = 0.0F;
                        previousRawErrorValid = false;
                        /*
                         * 启动或视觉恢复的第一帧只重建位置并把速度置零；
                         * 第二帧才有可信 dt 和速度，可重新产生控制输出。
                         */
                        if (recoveryValidCount == 0U) {
                            AlphaBetaFilter_Reset(&filter);
                            AlphaBetaFilter_Update(&filter, (float)measuredPx,
                                                   1.0F / 15.0F);
                            lastValidSampleTick = now;
                            recoveryValidCount = 1U;
                            holding = false;
                            holdStartTick = 0U;
                            if (state == BALL_CTRL_INTERNAL_LOST) {
                                state = BALL_CTRL_INTERNAL_WAIT_VISION;
                            }
                            break;
                        }
                        recoveryValidCount++;
                        if (recoveryValidCount <
                            BALL_CTRL_RECOVERY_VALID_COUNT) {
                            break;
                        }
                    }

                    if (lastValidSampleTick == 0U) {
                        dtSec = 1.0F / 15.0F;
                    } else {
                        dtSec = (float)(TickType_t)(now - lastValidSampleTick) /
                                (float)configTICK_RATE_HZ;
                        dtSec = BallControl_Clamp(dtSec, BALL_CTRL_DT_MIN_SEC,
                                                 BALL_CTRL_DT_MAX_SEC);
                    }
                    lastValidSampleTick = now;
                    AlphaBetaFilter_Update(&filter, (float)measuredPx, dtSec);
                    recoveryValidCount = BALL_CTRL_RECOVERY_VALID_COUNT;

                    rawError = (float)targetPx - filter.position;
                    absError = BallControl_Abs(rawError);
                    errorForP = rawError;
                    if (absError <= BALL_CTRL_POSITION_DEADBAND_PX) {
                        errorForP = 0.0F;
                    }

                    /*
                     * I 项只在低速、小误差、连续有效视觉下补偿静摩擦。越过目标、
                     * 速度升高或离开补偿区时立即清零，避免积分变成持续倾角。
                     */
                    resetIntegral = (state != BALL_CTRL_INTERNAL_ACTIVE) ||
                                    (BALL_CTRL_KI_RPM_PER_PXSEC <= 0.0F) ||
                                    (absError <= BALL_CTRL_I_DEADBAND_PX) ||
                                    (absError > BALL_CTRL_I_ENABLE_ERROR_PX) ||
                                    (BallControl_Abs(filter.velocity) >
                                     BALL_CTRL_I_ENABLE_VELOCITY_PXPS) ||
                                    (previousRawErrorValid &&
                                     BallControl_CrossedZero(previousRawError,
                                                              rawError));
                    if (resetIntegral) {
                        integralErrorPxSec = 0.0F;
                    } else {
                        integralLimit = BallControl_GetIntegralErrorLimit();
                        integralErrorPxSec += rawError * dtSec;
                        integralErrorPxSec = BallControl_Clamp(integralErrorPxSec,
                                                               -integralLimit,
                                                               integralLimit);
                    }
                    integralRpm = BALL_CTRL_KI_RPM_PER_PXSEC * integralErrorPxSec;
                    previousRawError = rawError;
                    previousRawErrorValid = true;

                    output = BALL_CTRL_OUTPUT_SIGN *
                             (BALL_CTRL_KP_RPM_PER_PX * errorForP -
                              BALL_CTRL_KD_RPM_PER_PXPS * filter.velocity +
                              integralRpm);
                    commandRpm = BallControl_RoundRpm(output);
                    Emm42Robot_VelControl(EMM42_ROBOT_LIFT, commandRpm,
                                          BALL_CTRL_EMM_ACC);
                    state = BALL_CTRL_INTERNAL_ACTIVE;

                    /* 首次大偏差只做一次方向自检，确认误差在向中心收敛。 */
                    if (!directionCheckActive && (directionInitialError == 0.0F) &&
                        (absError >= BALL_CTRL_DIR_CHECK_MIN_ERROR_PX)) {
                        directionInitialError = rawError;
                        directionCheckActive = true;
                    }
                    if (directionCheckActive) {
                        float initialAbs = BallControl_Abs(directionInitialError);

                        if (!BallControl_SameSign(rawError, directionInitialError) ||
                            (absError <= initialAbs - BALL_CTRL_DIR_CHECK_IMPROVE_PX)) {
                            directionCheckActive = false;
                        } else if (absError >= initialAbs + BALL_CTRL_DIR_CHECK_GROW_PX) {
                            directionBadSamples++;
                            if (directionBadSamples >= BALL_CTRL_DIR_CHECK_BAD_SAMPLES) {
                                Emm42Robot_Stop(EMM42_ROBOT_LIFT);
                                commandRpm = 0;
                                integralErrorPxSec = 0.0F;
                                integralRpm = 0.0F;
                                previousRawErrorValid = false;
                                state = BALL_CTRL_INTERNAL_FAULT_DIRECTION;
                            }
                        } else {
                            directionBadSamples = 0U;
                        }
                    }

                    if ((state == BALL_CTRL_INTERNAL_ACTIVE) &&
                        (absError <= BALL_CTRL_HOLD_POSITION_PX) &&
                        (BallControl_Abs(filter.velocity) <=
                         BALL_CTRL_HOLD_VELOCITY_PXPS)) {
                        if (holdStartTick == 0U) {
                            holdStartTick = now;
                        } else if (BallControl_Elapsed(now, holdStartTick,
                                                       BALL_CTRL_HOLD_TIME_MS)) {
                            holding = true;
                        }
                    } else {
                        holdStartTick = 0U;
                        holding = false;
                    }
                }
            }

            if (((state == BALL_CTRL_INTERNAL_ACTIVE) ||
                 (state == BALL_CTRL_INTERNAL_WAIT_VISION)) &&
                (lastValidSampleTick != 0U) &&
                BallControl_Elapsed(now, lastValidSampleTick,
                                    BALL_CTRL_VISION_DEGRADE_MS)) {
                state = BALL_CTRL_INTERNAL_DEGRADED;
                degradeStepTick = now;
                recoveryValidCount = 0U;
                holding = false;
                holdStartTick = 0U;
                integralErrorPxSec = 0.0F;
                integralRpm = 0.0F;
                previousRawErrorValid = false;
            }

            if ((state == BALL_CTRL_INTERNAL_DEGRADED) &&
                BallControl_Elapsed(now, degradeStepTick,
                                    BALL_CTRL_DEGRADE_STEP_PERIOD_MS)) {
                int16_t reducedRpm = BallControl_StepRpmTowardZero(commandRpm);

                degradeStepTick = now;
                if (reducedRpm != commandRpm) {
                    commandRpm = reducedRpm;
                    Emm42Robot_VelControl(EMM42_ROBOT_LIFT, commandRpm,
                                          BALL_CTRL_EMM_ACC);
                }
            }

            if (((state == BALL_CTRL_INTERNAL_ACTIVE) ||
                 (state == BALL_CTRL_INTERNAL_DEGRADED) ||
                 (state == BALL_CTRL_INTERNAL_WAIT_VISION)) &&
                (lastValidSampleTick != 0U) &&
                BallControl_Elapsed(now, lastValidSampleTick,
                                    BALL_CTRL_VISION_TIMEOUT_MS)) {
                Emm42Robot_Stop(EMM42_ROBOT_LIFT);
                commandRpm = 0;
                AlphaBetaFilter_Reset(&filter);
                state = BALL_CTRL_INTERNAL_LOST;
                consecutiveNaCount = BALL_CTRL_NA_LOST_COUNT;
                recoveryValidCount = 0U;
                holding = false;
                holdStartTick = 0U;
                integralErrorPxSec = 0.0F;
                integralRpm = 0.0F;
                previousRawErrorValid = false;
            }
            break;

        case BALL_CTRL_INTERNAL_STOP:
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            commandRpm = 0;
            integralErrorPxSec = 0.0F;
            integralRpm = 0.0F;
            previousRawErrorValid = false;
            state = BALL_CTRL_INTERNAL_DISABLE;
            break;

        case BALL_CTRL_INTERNAL_DISABLE:
            Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
            AlphaBetaFilter_Reset(&filter);
            consecutiveNaCount = 0U;
            recoveryValidCount = 0U;
            degradeStepTick = 0U;
            directionInitialError = 0.0F;
            directionBadSamples = 0U;
            directionCheckActive = false;
            integralErrorPxSec = 0.0F;
            integralRpm = 0.0F;
            previousRawErrorValid = false;
            holding = false;
            state = BALL_CTRL_INTERNAL_OFF;
            break;

        case BALL_CTRL_INTERNAL_FAULT_DIRECTION:
        case BALL_CTRL_INTERNAL_FAULT_EDGE:
            /* 故障态保持电机使能和当前位置，等待 K4 请求安全停止。 */
            break;

        default:
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            commandRpm = 0;
            integralErrorPxSec = 0.0F;
            integralRpm = 0.0F;
            previousRawErrorValid = false;
            state = BALL_CTRL_INTERNAL_STOP;
            break;
        }

        BallControl_Publish(state, holding, targetPx, measuredPx, &filter,
                            commandRpm, integralRpm, seenSampleSeq);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(APP_BALL_CONTROL_PERIOD_MS));
    }
}

void AppBallControlTask_Init(void)
{
    BaseType_t ret;

    s_commandQueue = xQueueCreate(1U, sizeof(BallControlCommand_t));
    configASSERT(s_commandQueue != NULL);

    s_publicStatus.state = APP_BALL_CONTROL_OFF;
    s_publicStatus.targetPx = APP_BALL_CONTROL_CENTER_X_PX;
    s_publicStatus.measuredPx = 0;
    s_publicStatus.filteredPx = 0.0F;
    s_publicStatus.velocityPxPerSec = 0.0F;
    s_publicStatus.commandRpm = 0;
    s_publicStatus.integralRpm = 0.0F;
    s_publicStatus.sampleSeq = 0U;

    ret = xTaskCreate(AppBallControlTask_Entry,
                      "BALLCTRL",
                      APP_BALL_CONTROL_TASK_STACK_WORDS,
                      NULL,
                      APP_BALL_CONTROL_TASK_PRIORITY,
                      &s_taskHandle);
    configASSERT(ret == pdPASS);
}

bool AppBallControl_RequestTarget(int16_t targetPx)
{
    BallControlCommand_t command;

    if ((s_commandQueue == NULL) ||
        (targetPx <= BALL_CTRL_SAFE_X_MIN_PX) ||
        (targetPx >= BALL_CTRL_SAFE_X_MAX_PX)) {
        return false;
    }

    command.enable = true;
    command.targetPx = targetPx;
    return xQueueOverwrite(s_commandQueue, &command) == pdPASS;
}

void AppBallControl_RequestStop(void)
{
    BallControlCommand_t command;

    if (s_commandQueue == NULL) {
        return;
    }

    command.enable = false;
    command.targetPx = APP_BALL_CONTROL_CENTER_X_PX;
    (void)xQueueOverwrite(s_commandQueue, &command);
}

void AppBallControl_GetStatus(AppBallControlStatus_t *out)
{
    if (out == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_publicStatus;
    taskEXIT_CRITICAL();
}

bool AppBallControl_IsActive(void)
{
    AppBallControlStatus_t status;

    AppBallControl_GetStatus(&status);
    return status.state != APP_BALL_CONTROL_OFF;
}
