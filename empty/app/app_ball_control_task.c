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
 * 钢球 X 位置后台闭环 —— 位置模式版本
 *
 * 数据流：视觉 X → α-β 位置/速度估计 → 局部 PD → ID1 绝对位置模式。
 * 本线程只控制 EMM42_ROBOT_LIFT，不触碰 ID2/ID3；题目层通过
 * AppBallControl_RequestTarget() 传目标，不直接共享内部变量。
 *
 * 2026-07-31 由速度模式换成位置模式：速度模式下 PID 输出的 RPM 对摆杆角度
 * 是一次积分、球位置对摆杆角度又是二次积分，整链三阶，纯 PID 极难镇定；
 * 且为了防止 RPM 失控，此前专门加了软降速(DEGRADED)、连续丢帧硬急停(LOST)、
 * 方向发散急停(FAULT_DIRECTION) 等保护——一旦被正常调参过程中偶发的视觉
 * 丢帧触发，就会打断本来平滑的运动，导致"移动过程中突然停止"。
 *
 * 位置模式每帧下发的是"目标该停在哪个绝对脉冲位置"，没有新命令时电机保持
 * 在原地（结构上自带安全，不会像速度模式那样一直转下去），因此上述保护
 * 大部分不再需要：本版本只保留两处真正需要主动出手的情形——边缘保护（球
 * 真的快滚出摆杆）和用户主动 K4 停止；其余情况（单帧 NA、暂时丢球）只更新
 * 显示状态，不打断已下发的绝对目标。详见 docs/BALL_CONTROL.md §9。
 * ================================================================== */

/*
 * 视觉约 30fps（原设计假设 15Hz，现上位机已提升帧率），名义帧间隔约 33ms。
 * 连续两帧 NA 或 220ms 无有效帧才判定 LOST——现在只影响 OLED 显示，不再
 * 主动下发停止命令。
 */
#define BALL_CTRL_NA_LOST_COUNT           (2U)
#define BALL_CTRL_VISION_TIMEOUT_MS       (220U)
#define BALL_CTRL_RECOVERY_VALID_COUNT    (2U)
/*
 * 2026-08-01 实测上位机帧率 40~60fps（真实帧间隔 16.7~25ms），故下限必须低于
 * 16.7ms：原值 0.02(20ms) 会把 60fps 的帧间隔【往上夹】，使 dt 被高估、速度估计
 * 系统性偏小约 20%，Kv 阻尼跟着失真。取 0.010 留一倍余量。
 */
#define BALL_CTRL_DT_MIN_SEC              (0.010F)
#define BALL_CTRL_DT_MAX_SEC              (0.200F)
/* 尚无双帧间隔时的默认 dt：按实测 40~60fps 取中间的 50fps。 */
#define BALL_CTRL_NOMINAL_DT_SEC          (1.0F / 50.0F)

/* 沿用题目二/三/六实测稳定的 Emm42 复位与使能等待。 */
#define BALL_CTRL_RESET_SETTLE_MS         (60U)
#define BALL_CTRL_ENABLE_SETTLE_MS        (180U)

/* α-β 滤波：先压制像素抖动，同时保留足够的速度响应。 */
#define BALL_CTRL_FILTER_ALPHA            (0.70F)
#define BALL_CTRL_FILTER_BETA             (0.08F)

/*
 * 位置模式控制律（纯 PD，无 I 项——绝对位置命令本身就能顶住静摩擦把丝杆
 * 定在目标位置，不需要像速度模式那样额外补偿）：
 *   targetPulse = LEVEL_TRIM_PULSE + SIGN * (Kx*errorPx - Kv*velocityPxPerSec)
 *
 * ID1 正 RPM 已标定为"连杆向下"，但连杆位于摆杆哪一端决定钢球 X 的响应
 * 方向。首次上车必须先用小增益确认；若越控越远，只改本文件的 OUTPUT_SIGN，
 * 不要改 emm42_robot.c 里的角色方向标定表（那是全局标定，会连带影响其它
 * 题目）。
 *
 * v1 按要求不加任何输出限幅/深度保护。Kx=30 已实测确认方向正确（球靠近
 * 目标时丝杆确实收回水平）；纯 P 在目标附近有轻微振荡（球带惯性冲过头，
 * P 项才反向修正，标准欠阻尼表现），从这个小值开始加 Kv 压振荡，观察后
 * 再翻倍或减半调整，见 docs/BALL_CONTROL.md 阶段 E。
 *
 * 2026-07-31 题目六阶梯测试实测出钢球静摩擦阈值约 3200~4000 脉冲，而
 * Kx=30 时中等误差（几十~一百多像素）算出来的脉冲量远够不到这个阈值——
 * 丝杆会抬到一个不痛不痒的高度就停住，球既不前进也不回落，卡死不动。
 * 这不是"稳态误差"该用 I 项解决的问题（I 项需要时间慢慢积分才能顶过
 * 阈值，且冲破瞬间会因为多余的积分量造成明显过冲），而是静摩擦这类
 * 阈值型非线性该用的经典处理：球基本没在动时，只要算出来的量级不到
 * 阈值，直接把幅度顶到阈值（方向不变），一步打破僵局；球已经在动、
 * 有速度时不做这个夹紧，让 Kv 正常接管减速，不然会打断已经在收敛的运动。
 */
#define BALL_CTRL_OUTPUT_SIGN             (1.0F)
#define BALL_CTRL_KX_PULSE_PER_PX         (25.0F)
#define BALL_CTRL_KV_PULSE_PER_PXPS       (7.5F)
#define BALL_CTRL_LEVEL_TRIM_PULSE        (0)

/*
 * 到位保持精度与 OLED 的 HOLD 判定一致。进入该范围即回水平并禁止静摩擦夹紧，
 * 避免静摩擦补偿的较大倾角把已满足精度的钢球再次推出目标区。
 */
#define BALL_CTRL_SETTLE_DEADBAND_PX      (6.0F)

/* 题目六阶梯测试实测的钢球静摩擦阈值（3200~4000 脉冲），取中间值，可调。 */
#define BALL_CTRL_STICTION_PULSE          (3600.0F)

/* 判定"球基本没在动"的速度门限，低于此值才允许静摩擦夹紧介入。 */
#define BALL_CTRL_STUCK_VELOCITY_PXPS     (6.0F)

/*
 * 低速状态要持续这么久才算"真的卡住"，不是正常减速路过低速的一瞬间。
 * 没有这个时间门限时，球快到目标前的正常减速（Kv 在正常刹车）也会有
 * 瞬间低速，会被误判成"卡住"进而满幅夹紧，把已经快停稳的球重新推走，
 * 形成"以为卡住→满幅踹一脚→冲过头→减速→又被当成卡住"的持续振荡
 * （2026-07-31 实测：目标附近 50px 内来回抖动、到不了）。
 */
#define BALL_CTRL_STUCK_TIME_MS           (150U)

/*
 * 位置模式运行参数。30RPM 是题目六"转10圈慢慢量距离"标定测试用的保守值，
 * 直接套到实时闭环上太慢（30RPM≈4mm/s 丝杆线速度，见 docs/BALL_CONTROL.md），
 * 提到 200RPM 起步测试（协议层上限 3000RPM，驱动器额定 3000RPM+，尚未在本
 * 机构上实测过这个速度，先观察有无异响/失步再决定要不要再提高）。
 *
 * acc 必须用 0（瞬时），不能像题目二轮子那样用非 0 曲线档位：手册公式
 * "每升1RPM需要(256-acc)*50us"，acc=150 时爬到 200RPM 要 1 秒多，但
 * BALLCTRL 每 33ms 就刷新一次新的绝对目标，曲线还没爬起来就被下一帧打断
 * 重新规划，实际能达到的速度被摁在 33ms/(256-acc)/50us 这个RPM量级——
 * 比 acc=0 时瞬时给到目标 RPM 反而慢得多（2026-07-31 实测验证）。
 * 题目二 acc=180 有效是因为它是持续巡航、前后两帧目标转速差很小，不需要
 * 从 0 重新爬升，跟这里"每帧一次全新绝对目标"的场景不通用，不能照搬。
 */
#define BALL_CTRL_POS_RPM                 (200U)
#define BALL_CTRL_POS_ACC                 (0U)

/* 中心保持判定，仅用于 OLED 显示 HOLDING/RUNNING，不影响任何控制动作。 */
#define BALL_CTRL_HOLD_POSITION_PX        (5.0F)
#define BALL_CTRL_HOLD_VELOCITY_PXPS      (10.0F)
#define BALL_CTRL_HOLD_TIME_MS            (500U)

/* 安全边界：球越过这个像素范围视为快滚出摆杆，命令回水平并锁定报警。 */
#define BALL_CTRL_SAFE_X_MIN_PX           (20)
#define BALL_CTRL_SAFE_X_MAX_PX           (620)

/* 菜单 K4 闭环的默认参数；任务三通过独立 profile 覆盖这些控制量。 */
static const AppBallControlProfile_t s_menuProfile = {
    BALL_CTRL_FILTER_ALPHA,
    BALL_CTRL_FILTER_BETA,
    BALL_CTRL_OUTPUT_SIGN,
    BALL_CTRL_KX_PULSE_PER_PX,
    BALL_CTRL_KV_PULSE_PER_PXPS,
    BALL_CTRL_LEVEL_TRIM_PULSE,
    BALL_CTRL_SETTLE_DEADBAND_PX,
    BALL_CTRL_STICTION_PULSE,
    BALL_CTRL_STUCK_VELOCITY_PXPS,
    BALL_CTRL_STUCK_TIME_MS,
    BALL_CTRL_POS_RPM,
    BALL_CTRL_POS_ACC,
    BALL_CTRL_HOLD_POSITION_PX,
    BALL_CTRL_HOLD_VELOCITY_PXPS,
    BALL_CTRL_HOLD_TIME_MS
};

typedef enum {
    BALL_CTRL_INTERNAL_OFF = 0,
    BALL_CTRL_INTERNAL_RESET_DISABLE,
    BALL_CTRL_INTERNAL_RESET_WAIT,
    BALL_CTRL_INTERNAL_ENABLE,
    BALL_CTRL_INTERNAL_ENABLE_WAIT,
    BALL_CTRL_INTERNAL_ZERO,
    BALL_CTRL_INTERNAL_WAIT_VISION,
    BALL_CTRL_INTERNAL_ACTIVE,
    BALL_CTRL_INTERNAL_LOST,
    BALL_CTRL_INTERNAL_STOP,
    BALL_CTRL_INTERNAL_DISABLE,
    BALL_CTRL_INTERNAL_FAULT_EDGE
} BallControlInternalState_t;

typedef struct {
    bool enable;
    int16_t targetPx;
    AppBallControlProfile_t profile;
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

static int32_t BallControl_RoundToInt32(float value)
{
    if (value >= 0.0F) {
        return (int32_t)(value + 0.5F);
    }
    return (int32_t)(value - 0.5F);
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
    case BALL_CTRL_INTERNAL_ZERO:
        return APP_BALL_CONTROL_STARTING;
    case BALL_CTRL_INTERNAL_WAIT_VISION:
        return APP_BALL_CONTROL_WAIT_VISION;
    case BALL_CTRL_INTERNAL_ACTIVE:
        return holding ? APP_BALL_CONTROL_HOLDING : APP_BALL_CONTROL_RUNNING;
    case BALL_CTRL_INTERNAL_LOST:
        return APP_BALL_CONTROL_LOST;
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
                                const AlphaBetaFilter_t *filter, int32_t commandPulse,
                                uint32_t sampleSeq)
{
    taskENTER_CRITICAL();
    s_publicStatus.state = BallControl_ToPublicState(state, holding);
    s_publicStatus.targetPx = targetPx;
    s_publicStatus.measuredPx = measuredPx;
    s_publicStatus.filteredPx = filter->position;
    s_publicStatus.velocityPxPerSec = filter->velocity;
    s_publicStatus.commandPulse = commandPulse;
    s_publicStatus.sampleSeq = sampleSeq;
    taskEXIT_CRITICAL();
}

static void AppBallControlTask_Entry(void *argument)
{
    /*
     * 只在本次上电后第一次启动时清零：把上电前人工摸平的位置定义为原点。
     * 之后反复用 K4 停/启调参不会重新清零，沿用第一次建立的原点，避免中途
     * 摆杆停在某个倾角时被误当成新零点、跨轮次累积误差。函数级 static 在
     * 任务生命周期内持续存在，只有 MCU 重新上电才会复位。
     */
    static bool hasZeroedSinceBoot = false;

    BallControlInternalState_t state = BALL_CTRL_INTERNAL_OFF;
    BallControlCommand_t command;
    AppVisionXSample_t vision;
    AlphaBetaFilter_t filter;
    AppBallControlProfile_t activeProfile = s_menuProfile;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t stateStart = lastWake;
    TickType_t lastValidSampleTick = 0U;
    TickType_t holdStartTick = 0U;
    TickType_t stuckStartTick = 0U;
    uint32_t seenSampleSeq = 0U;
    uint32_t consecutiveNaCount = 0U;
    uint32_t recoveryValidCount = 0U;
    int16_t targetPx = APP_BALL_CONTROL_CENTER_X_PX;
    int16_t measuredPx = 0;
    int32_t commandPulse = 0;
    bool holding = false;

    (void)argument;
    AlphaBetaFilter_Init(&filter, activeProfile.filterAlpha, activeProfile.filterBeta);
    BallControl_Publish(state, holding, targetPx, measuredPx, &filter, commandPulse,
                        seenSampleSeq);

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* 长度为 1 的覆盖队列只保留调用方最新意图。 */
        if (xQueueReceive(s_commandQueue, &command, 0U) == pdPASS) {
            targetPx = command.targetPx;
            activeProfile = command.profile;
            holding = false;
            holdStartTick = 0U;
            stuckStartTick = 0U;

            if (command.enable) {
                if (state == BALL_CTRL_INTERNAL_OFF) {
                    AlphaBetaFilter_Init(&filter, activeProfile.filterAlpha,
                                         activeProfile.filterBeta);
                    AlphaBetaFilter_Reset(&filter);
                    seenSampleSeq = 0U;
                    lastValidSampleTick = 0U;
                    measuredPx = 0;
                    commandPulse = 0;
                    consecutiveNaCount = 0U;
                    recoveryValidCount = 0U;
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
                state = BALL_CTRL_INTERNAL_ZERO;
            }
            break;

        case BALL_CTRL_INTERNAL_ZERO:
            if (!hasZeroedSinceBoot) {
                Emm42Robot_ResetPosToZero(EMM42_ROBOT_LIFT);
                hasZeroedSinceBoot = true;
            }
            state = BALL_CTRL_INTERNAL_WAIT_VISION;
            break;

        case BALL_CTRL_INTERNAL_WAIT_VISION:
        case BALL_CTRL_INTERNAL_ACTIVE:
        case BALL_CTRL_INTERNAL_LOST:
            AppVisionLink_GetLatestX(&vision);

            if (vision.sequence != seenSampleSeq) {
                seenSampleSeq = vision.sequence;

                if (vision.na) {
                    consecutiveNaCount++;
                    if ((consecutiveNaCount >= BALL_CTRL_NA_LOST_COUNT) &&
                        (state != BALL_CTRL_INTERNAL_LOST)) {
                        /*
                         * 连续两帧 NA 才判定持续丢球；只标记状态供 OLED 显示，
                         * 不再急停——摆杆已经停在最后一次有效目标上，本身就安全。
                         */
                        AlphaBetaFilter_Reset(&filter);
                        recoveryValidCount = 0U;
                        holding = false;
                        holdStartTick = 0U;
                        stuckStartTick = 0U;
                        state = BALL_CTRL_INTERNAL_LOST;
                    }
                } else if (vision.valid) {
                    float dtSec;
                    float rawError;
                    float pulseDelta;
                    float output;

                    measuredPx = vision.pixel;
                    if ((measuredPx <= BALL_CTRL_SAFE_X_MIN_PX) ||
                        (measuredPx >= BALL_CTRL_SAFE_X_MAX_PX)) {
                        /* 球快滚出摆杆：命令回水平，标记故障并锁定，等 K4 处理。 */
                        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT,
                                               activeProfile.levelTrimPulse,
                                               activeProfile.positionRpm,
                                               activeProfile.positionAcc);
                        commandPulse = activeProfile.levelTrimPulse;
                        state = BALL_CTRL_INTERNAL_FAULT_EDGE;
                        break;
                    }

                    consecutiveNaCount = 0U;

                    if ((state == BALL_CTRL_INTERNAL_WAIT_VISION) ||
                        (state == BALL_CTRL_INTERNAL_LOST)) {
                        /*
                         * 启动或视觉恢复的第一帧只重建位置并把速度置零；
                         * 第二帧才有可信 dt 和速度，可重新产生控制输出。
                         */
                        if (recoveryValidCount == 0U) {
                            AlphaBetaFilter_Reset(&filter);
                            AlphaBetaFilter_Update(&filter, (float)measuredPx,
                                                   BALL_CTRL_NOMINAL_DT_SEC);
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
                        if (recoveryValidCount < BALL_CTRL_RECOVERY_VALID_COUNT) {
                            break;
                        }
                    }

                    if (lastValidSampleTick == 0U) {
                        dtSec = BALL_CTRL_NOMINAL_DT_SEC;
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

                    /* 纯 PD，无 I 项。 */
                    pulseDelta = (activeProfile.kxPulsePerPx * rawError) -
                                 (activeProfile.kvPulsePerPxps * filter.velocity);

                    if (BallControl_Abs(rawError) <= activeProfile.settleDeadbandPx) {
                        /* 到位保持区内不再修正，也不把静摩擦夹紧误判为卡住。 */
                        pulseDelta = 0.0F;
                        stuckStartTick = 0U;
                    } else if (BallControl_Abs(filter.velocity) <=
                               activeProfile.stuckVelocityPxps) {
                        /*
                         * 只有低速状态【持续够久】才算真的卡住——球正常减速接近目标
                         * 时也会有瞬间低速，不能一测到低速就立刻满幅夹紧，否则会把
                         * 快停稳的球重新推走，变成持续振荡。
                         */
                        if (stuckStartTick == 0U) {
                            stuckStartTick = now;
                        } else if (BallControl_Elapsed(now, stuckStartTick,
                                                       activeProfile.stuckTimeMs) &&
                                   (BallControl_Abs(pulseDelta) < activeProfile.stictionPulse)) {
                            /* 确认卡住：直接把幅度顶到实测阈值（保留方向）打破僵局。 */
                            pulseDelta = (pulseDelta >= 0.0F) ? activeProfile.stictionPulse
                                                               : -activeProfile.stictionPulse;
                        }
                    } else {
                        /* 球在正常移动，没有卡住，重置计时。 */
                        stuckStartTick = 0U;
                    }

                    output = (float)activeProfile.levelTrimPulse +
                             (activeProfile.outputSign * pulseDelta);
                    commandPulse = BallControl_RoundToInt32(output);

                    Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, commandPulse,
                                           activeProfile.positionRpm,
                                           activeProfile.positionAcc);
                    state = BALL_CTRL_INTERNAL_ACTIVE;

                    if ((BallControl_Abs(rawError) <= activeProfile.holdPositionPx) &&
                        (BallControl_Abs(filter.velocity) <= activeProfile.holdVelocityPxps)) {
                        if (holdStartTick == 0U) {
                            holdStartTick = now;
                        } else if (BallControl_Elapsed(now, holdStartTick,
                                                       activeProfile.holdTimeMs)) {
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
                BallControl_Elapsed(now, lastValidSampleTick, BALL_CTRL_VISION_TIMEOUT_MS)) {
                /* 视觉链路彻底没数据（不只是偶发 NA），同样只标记 LOST，不主动停车。 */
                AlphaBetaFilter_Reset(&filter);
                state = BALL_CTRL_INTERNAL_LOST;
                consecutiveNaCount = BALL_CTRL_NA_LOST_COUNT;
                recoveryValidCount = 0U;
                holding = false;
                holdStartTick = 0U;
                stuckStartTick = 0U;
            }
            break;

        case BALL_CTRL_INTERNAL_STOP:
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            commandPulse = 0;
            state = BALL_CTRL_INTERNAL_DISABLE;
            break;

        case BALL_CTRL_INTERNAL_DISABLE:
            Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
            AlphaBetaFilter_Reset(&filter);
            consecutiveNaCount = 0U;
            recoveryValidCount = 0U;
            holding = false;
            state = BALL_CTRL_INTERNAL_OFF;
            break;

        case BALL_CTRL_INTERNAL_FAULT_EDGE:
            /* 已把摆杆命令回水平，保持使能和当前位置，等待 K4 请求安全停止。 */
            break;

        default:
            /* 状态异常时只对本模块的 ID1 做安全收尾。 */
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            commandPulse = 0;
            state = BALL_CTRL_INTERNAL_STOP;
            break;
        }

        BallControl_Publish(state, holding, targetPx, measuredPx, &filter, commandPulse,
                            seenSampleSeq);
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
    s_publicStatus.commandPulse = 0;
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
    return AppBallControl_RequestTargetWithProfile(targetPx, &s_menuProfile);
}

bool AppBallControl_RequestTargetWithProfile(int16_t targetPx,
                                             const AppBallControlProfile_t *profile)
{
    BallControlCommand_t command;

    if ((s_commandQueue == NULL) || (profile == NULL) ||
        (targetPx <= BALL_CTRL_SAFE_X_MIN_PX) ||
        (targetPx >= BALL_CTRL_SAFE_X_MAX_PX) ||
        (profile->positionRpm == 0U)) {
        return false;
    }

    command.enable = true;
    command.targetPx = targetPx;
    command.profile = *profile;
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
    command.profile = s_menuProfile;
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
