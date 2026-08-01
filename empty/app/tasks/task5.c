#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "app_ball_control_task.h"
#include "bsp_buzzer.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 5 题：两段式 K3 启动（钢珠居中→发车）+ 循迹到终点线后直行保持 +
 *          对称软件斜坡线性缓停
 *
 * 2026-08 按题目四整体框架重做：进题目先只让 BALLCTRL 把钢珠伺服到
 * T5_BALL_TARGET_X_PX（第一次 K3），确认稳住后第二次 K3 才使能 ID2/ID3
 * 发车循迹，录像同步从第二次 K3 开始；行驶期间 ID1 全程保持钢珠闭环。
 * 加速度/顶速沿用题目四当前值（起步用软件斜坡 T5_RAMP_RPM_PER_SEC 压制
 * 驱动器 acc 曲线本身压不住的阶跃感）。
 *
 * 终点判定：武装后（先确认已离开出发线）首次读到 >=5 路命中黑线即判定
 * 到达终点——立即把左右轮命令改成同一个值（进入本阶段那一刻的平均转速，
 * 即"保持当前速度直直地前进"，不再有转向修正），维持 T5_AFTER_LINE_MS
 * (1500ms) 后才进入缓停。蜂鸣器第二次响与通知视觉端结束录像不等这 1500ms
 * 走完，而是在其中的 T5_BEEP_DELAY_MS(800ms) 时刻先触发——录像/计分只覆盖
 * 到终点线+800ms，之后车辆仍会再直行 700ms 才开始减速，给机械留够裕量。
 * 1500ms 结束后用与起步同一根 T5_RAMP_RPM_PER_SEC 斜坡反向线性降速到 0
 * （期间继续循迹 PID 修正，尽量平稳停住），最后两拍发送速度模式 0 RPM 收尾。
 * ================================================================== */

/*
 * PID 增益、积分限幅与转向输出限幅：初值与题目四当前值一致。
 * 2026-08 实车反馈"直线巡线左右摆动得比较厉害、过弯也有震动"后第一轮retune：
 * Kp 5.0→3.0（原值对离散 8 路灰度这种台阶式误差偏敏感，小误差就打出较大转向量，
 * 容易在直道上来回过修正形成等幅摆动）；Kd 0.2→0.35（加大阻尼，压制摆动本身，
 * 不像单纯降 Kp 那样牺牲太多响应速度）。这是新一轮起点值，不是已验证的最终值，
 * 需要上车看：摆动明显减轻但过弯跟不上（切内角/冲出）→ Kp 适当调回大一点；
 * 摆动还在→ Kp 继续往下、或 Kd 继续往上。
 */
#define T5_KP                              (2.2F)
#define T5_KI                              (0.32F)
#define T5_KD                              (0.7F)
#define T5_INTEGRAL_LIMIT                  (20.0F)
#define T5_MAX_STEER_RPM                   (100.0F)

/* 基础速度、单轮安全范围：按用户要求"加速度和极速都参考第四题"，直接复制题目四当前值。 */
#define T5_BASE_RPM                        (80.0F)
#define T5_MIN_WHEEL_RPM                   (5.0F)
#define T5_MAX_WHEEL_RPM                   (110.0F)
#define T5_MIN_BASE_RPM                    (10.0F)

/* 驱动器内部曲线加速度档位：与题目四一致，取较慢的小值，真正的起步平缓度交给下面的软件斜坡。 */
#define T5_EMM_ACC                         (5U)

/*
 * 起步/停车共用同一根软件速度斜坡（RPM/秒）：直接复制题目四当前值。
 * 启动时从 0 每拍向上抬到 T5_BASE_RPM；到达终点线直行保持 T5_AFTER_LINE_MS
 * 结束后，从当前值原样每拍向下压到 0——用户明确要求"跟小车从零加速一样的加速度
 * 来减速"，因此加速、减速共用同一个速率，不再另开一个缓停参数。
 */
#define T5_RAMP_RPM_PER_SEC                (20.0F)

/* 末段硬停用的驱动器曲线档位：此时软件斜坡已经把速度压到接近 0，这里只是让驱动器内部目标也归零，数值本身影响很小。 */
#define T5_STOP_EMM_ACC                    (80U)

/* UIMENU 固定控制周期。 */
#define T5_DT_SEC                          (0.03F)
#define T5_TICK_MS                         (30U)

/* 丢线、入场和共享 UART1 总线时序参数：与题目二/四一致。 */
#define T5_LINE_LOST_TICKS                 (20U)
#define T5_RESET_SETTLE_TICKS              (2U)
#define T5_ENABLE_SETTLE_TICKS             (6U)
#define T5_EMM_CMD_GAP_MS                  (6U)

/* 误差滤波、死区和转弯减速参数：初值与题目二/四一致。 */
#define T5_ERROR_FILTER_ALPHA              (0.5F)
/*
 * 死区 1.0→2.0：8 路灰度加权误差最小非零台阶就是 ±1（单路偏移一档），原死区
 * 1.0 卡在"等于 1 不算小"的边界上，这个最小台阶完全不会被过滤掉，直道上车身
 * 只要有一丁点没对准，就会被当成需要修正的误差持续打转向量，是"直线左右
 * 摆动"的另一个来源（跟上面 Kp 偏大是两个独立成因，一起改）。调到 2.0 后至少
 * 单路最小偏移会被吃掉，仍然对 2 档以上的真实偏移保持响应。
 */
#define T5_ERROR_DEADBAND                  (2.0F)
#define T5_CORNER_SLOWDOWN_GAIN            (0.65F)

/*
 * 转向输出软件限速（RPM/秒）：题目五独有，题目二/四没有这个环节。
 *
 * 现象：过弯时 PID 算出的转向量一拍内就能从很小跳到接近 T5_MAX_STEER_RPM，
 * 车身瞬间大幅横摆，把摆杆上的钢珠带得一起晃——这是"巡线不够细腻"的真正
 * 根因，不只是 Kp 大小的问题：即使 Kp 合适，离散灰度传感器命中路数切换
 * 本身就是台阶式的，PID 输出天然带阶跃。
 *
 * 做法与起步斜坡 T5_RAMP_RPM_PER_SEC 同一思路：PID 算出的原始转向量只作为
 * "目标"，实际下发给轮子差速的转向量 s_appliedSteerRpm 每拍最多向目标靠近
 * (本值×T5_DT_SEC)，把阶跃摊成渐变，car 最终仍会转到 PID 要求的量，只是
 * 不再一拍到位。数值越小越平滑但过弯响应越滞后，太小会明显"跟不上弯道"；
 * 400 是新增功能第一版的起点，需要在实车上配合"横摆是否消失"/"过弯会不会
 * 反应不过来"现场调大调小，不代表已验证的最终值。
 */
#define T5_STEER_SLEW_RPM_PER_SEC          (250.0F)

/*
 * 终点线判定：先连续 T5_ARM_TICKS 拍命中 <=T5_ARM_HIT_MAX 路细线，确认已经
 * 离开出发线才"武装"，避免出发瞬间压在宽起始线上被立即误判成终点。
 * 武装后单拍命中 >=T5_STOP_LINE_HIT_MIN（8 路里的 5 路）即判定到达终点，
 * 这是用户本次明确给出的新阈值（原先题目二/四系的终点判定用的是 6 路）。
 */
#define T5_ARM_HIT_MAX                     (3U)
#define T5_ARM_TICKS                       (15U)
#define T5_STOP_LINE_HIT_MIN               (5U)

/*
 * 终点线时间下限：赛道是环形，起跑线和终点线是同一根线。仅凭"武装+命中路数"
 * 不足以保证车辆真的跑完了一整圈——如果赛道较小或武装判定得比较快，发车后
 * 短时间内就可能再次经过这根线（或其他满足命中条件的黑色区域）被误判成
 * 终点。加一层时间下限：即使武装、命中路数都满足，也要发车（第二次 K3）
 * 后累计前进时间 >= 本值才真正判定到达终点，用原始 tick 计数（不叠加 OLED
 * 显示用的 T5_STOPWATCH_CAL_SCALE 校准系数）。
 */
#define T5_FINISH_MIN_ELAPSED_MS           (3000U)

/*
 * 到达终点线后"保持当前速度直直地前进"的总时长，1500ms 后才转入缓停。
 * 蜂鸣器第二次响、通知视觉端结束录像不是在这 1500ms 走完时触发，而是在
 * 其中更早的 T5_BEEP_DELAY_MS(800ms) 时刻——两者都由用户本次明确给出。
 */
#define T5_AFTER_LINE_MS                   (1500U)
#define T5_BEEP_DELAY_MS                   (800U)

/* 秒表校准系数：与任务二/四采用同一实测初值，任务五可按实测继续独立修正。 */
#define T5_STOPWATCH_CAL_SCALE             (0.897F)

/*
 * 任务五钢珠平衡 profile：数值按值复制自题目四当前 T4_BALL_* 参数组作为起点
 * （同一新曲柄摇杆机构、同一 ID1，理论上可直接复用），此后两题各调各的，
 * 互不影响，符合仓库【控制参数隔离规则】。
 */
#define T5_BALL_TARGET_X_PX                 (310)

#define T5_BALL_FRICTION_FF_PULSE           (0.0F)
#define T5_BALL_LEVEL_TRIM_PULSE            (-54)
#define T5_BALL_FILTER_ALPHA                (0.9F)
#define T5_BALL_FILTER_BETA                 (0.2F)
#define T5_BALL_OUTPUT_SIGN                 (-1.0F)
#define T5_BALL_KX_PULSE_PER_PX             (1.066F)
#define T5_BALL_KV_PULSE_PER_PXPS           (0.504F)
#define T5_BALL_SETTLE_DEADBAND_PX          (4.0F)
#define T5_BALL_FF_VEL_BLEND_PXPS           (15.0F)
#define T5_BALL_POS_RPM                     (200U)
#define T5_BALL_POS_ACC                     (240U)
/*
 * 软件限速：每帧最多允许下发的绝对目标变化量（脉冲），0=不限速。
 *
 * 2026-08 之前是 0（首帧不限速直跳到 PD 算出的目标，进入闭环时曲柄摇杆
 * "冲"那一下很猛）。现在给非 0 值让目标从 0（ZERO 阶段的零点）开始按
 * 每帧最多这么多脉冲逐步逼近，等效于限制起步阶段的平均加速度。
 *
 * 参考开机归零的 LIFT_HOMING_SEEK_RPM=5：在约 50fps（视觉帧间隔）下
 * 等效为 5*3200/60/50 ≈ 5.3 脉冲/帧。这里取 10 留一倍余量——起步时
 * 每帧目标只挪 10 脉冲，驱动器配合 T5_BALL_POS_ACC 在内部做曲线平滑，
 * 整体观感跟开机归零慢速下降一致；到目标后会自然定住。调小会更慢、
 * 调大能更快跟上球位置变化，视实车反馈现场调。
 */
#define T5_BALL_MAX_PULSE_STEP              (10U)
#define T5_BALL_HOLD_POSITION_PX            (6.0F)
#define T5_BALL_HOLD_VELOCITY_PXPS          (10.0F)
#define T5_BALL_HOLD_TIME_MS                (500U)

/*
 * 手动调参开关：置 1 时轮子完全不使能，第二次 K3 也不会发车，只让 BALLCTRL
 * 保持钢球平衡，供用手推拉小车模拟加减速扰动、专调 T5_BALL_* 参数。
 * 默认 0，执行完整的"两段式启动 + 循迹到终点线 + 对称软件斜坡缓停"流程。
 */
#define T5_MOTORS_DISABLED_MANUAL_TEST      (0U)

typedef enum {
    T5_STATE_IDLE = 0,          /* 刚进题目：什么都不动，等第一次 K3 启动球杆平衡 */
    T5_STATE_WAIT_BALL_CONTROL, /* 已请求球杆闭环，等后台真正接管 ID1 */
    T5_STATE_BALL_READY,        /* 球杆闭环已工作、小车待发，等第二次 K3 */
    T5_STATE_BALL_RECOVER_WAIT, /* 钢珠闭环触发 FAULT_EDGE 后，等待其停止/释放 ID1 再重新请求 */
#if T5_MOTORS_DISABLED_MANUAL_TEST
    T5_STATE_MANUAL_BALANCE,    /* 电机不启动，仅后台 BALLCTRL 保持钢球平衡 */
#endif
    T5_STATE_RESET_DISABLE,
    T5_STATE_RESET_WAIT,
    T5_STATE_ENABLE_LEFT,
    T5_STATE_ENABLE_LEFT_WAIT,
    T5_STATE_ENABLE_RIGHT,
    T5_STATE_ENABLE_RIGHT_WAIT,
    T5_STATE_RUN,
    T5_STATE_AFTER_LINE,        /* 已判定到达终点线，原样匀速保持 T5_AFTER_LINE_MS */
    T5_STATE_DECEL,             /* 直行保持结束，软件斜坡对称降速到 0，期间继续循迹修正 */
    T5_STATE_STOP_LEFT,
    T5_STATE_STOP_RIGHT,
    T5_STATE_FINISHED
} Task5State_t;

static const AppBallControlProfile_t s_task5BallProfile = {
    T5_BALL_FILTER_ALPHA,
    T5_BALL_FILTER_BETA,
    T5_BALL_OUTPUT_SIGN,
    T5_BALL_KX_PULSE_PER_PX,
    T5_BALL_KV_PULSE_PER_PXPS,
    T5_BALL_LEVEL_TRIM_PULSE,
    T5_BALL_SETTLE_DEADBAND_PX,
    T5_BALL_FRICTION_FF_PULSE,
    T5_BALL_FF_VEL_BLEND_PXPS,
    T5_BALL_POS_RPM,
    T5_BALL_POS_ACC,
    T5_BALL_MAX_PULSE_STEP,
    T5_BALL_HOLD_POSITION_PX,
    T5_BALL_HOLD_VELOCITY_PXPS,
    T5_BALL_HOLD_TIME_MS
};

static Task5State_t s_state;
static Pid_t        s_pid;
static float        s_lastSteerRpm;
static float        s_appliedSteerRpm; /* PID 转向量经 T5_STEER_SLEW_RPM_PER_SEC 限速后实际下发的值 */
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
static uint32_t     s_afterLineElapsedMs;
static float        s_straightRpm;     /* 判定到达终点线那一刻的平均转速，AFTER_LINE 阶段两轮都按此值直行 */
static float        s_rampBaseRpm;     /* 起步/缓停共用的软件斜坡当前基础速度 */
static uint32_t     s_elapsedTicks;
static bool         s_ballControlRequested;
static bool         s_wheelsStarted;   /* 轮子是否已经完成过一次使能起步（钢珠故障恢复后据此跳过重复使能） */
static bool         s_runCompleted;    /* 本次任务是否已经判定到达终点（用于 FAULT_EDGE 恢复握手，含义同题目四） */
static bool         s_startBallRequested;
static bool         s_startCarRequested;
static bool         s_finishBeeped;    /* 终点提示音只响一次 */
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

/* 整体平移限幅，保留左右轮差速，避免独立限幅压扁转向量。 */
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

/* 正常控制每拍最多发一帧，左右轮交替更新，避免共享总线背靠背丢帧。 */
static void Task5_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t leftInt = (leftRpm >= 0.0F) ? (int32_t)(leftRpm + 0.5F)
                                        : (int32_t)(leftRpm - 0.5F);
    int32_t rightInt = (rightRpm >= 0.0F) ? (int32_t)(rightRpm + 0.5F)
                                          : (int32_t)(rightRpm - 0.5F);

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

/* 有效线数据才更新滤波和 PID；长期丢线时置位 s_lineStopped 作安全兜底。 */
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

    /*
     * 转向输出限速：s_appliedSteerRpm 每拍最多向 s_lastSteerRpm 靠近
     * (T5_STEER_SLEW_RPM_PER_SEC × T5_DT_SEC)，把 PID 阶跃摊成渐变，
     * 抑制过弯瞬间大幅横摆把钢珠带得晃动；最终仍会跟上 PID 要求的转向量。
     */
    {
        float maxDelta = T5_STEER_SLEW_RPM_PER_SEC * T5_DT_SEC;
        float delta = s_lastSteerRpm - s_appliedSteerRpm;
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        s_appliedSteerRpm += delta;
    }
}

/*
 * 正常循迹目标：转弯时降低基础速度，起步阶段叠加软件斜坡（与题目四同一套
 * 做法）压住驱动器 acc 曲线本身压不住的起步阶跃，斜坡爬满 T5_BASE_RPM 后
 * 自然失效，不影响后续转弯减速。
 */
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

    steerAbs = (s_appliedSteerRpm >= 0.0F) ? s_appliedSteerRpm : -s_appliedSteerRpm;
    dynBaseRpm = T5_BASE_RPM - T5_CORNER_SLOWDOWN_GAIN * steerAbs;
    dynBaseRpm = Task5_Clamp(dynBaseRpm, T5_MIN_BASE_RPM, T5_BASE_RPM);

    s_rampBaseRpm += T5_RAMP_RPM_PER_SEC * T5_DT_SEC;
    if (s_rampBaseRpm > T5_BASE_RPM) {
        s_rampBaseRpm = T5_BASE_RPM;
    }
    if (dynBaseRpm > s_rampBaseRpm) {
        dynBaseRpm = s_rampBaseRpm;
    }

    targetLeftRpm = dynBaseRpm + s_appliedSteerRpm;
    targetRightRpm = dynBaseRpm - s_appliedSteerRpm;
    Task5_ClampWheelPair(&targetLeftRpm, &targetRightRpm);

    s_leftRpm = targetLeftRpm;
    s_rightRpm = targetRightRpm;
    Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);
}

/*
 * 缓停目标：基础速度上限用 s_rampBaseRpm（由调用方每拍先减量），差速限幅
 * 随之同步收窄，保证两轮始终非负并能平滑降到 0 RPM，同时继续做转弯修正。
 */
static void Task5_ApplyDecelTracking(void)
{
    float steerAbs;
    float dynBaseRpm;
    float steerLimit;
    float steerRpm;
    float targetLeftRpm;
    float targetRightRpm;

    steerAbs = (s_appliedSteerRpm >= 0.0F) ? s_appliedSteerRpm : -s_appliedSteerRpm;
    dynBaseRpm = s_rampBaseRpm - T5_CORNER_SLOWDOWN_GAIN * steerAbs;
    dynBaseRpm = Task5_Clamp(dynBaseRpm, 0.0F, s_rampBaseRpm);

    steerLimit = dynBaseRpm;
    if ((T5_MAX_WHEEL_RPM - dynBaseRpm) < steerLimit) {
        steerLimit = T5_MAX_WHEEL_RPM - dynBaseRpm;
    }
    steerLimit = Task5_Clamp(steerLimit, 0.0F, T5_MAX_STEER_RPM);
    steerRpm = Task5_Clamp(s_appliedSteerRpm, -steerLimit, steerLimit);

    targetLeftRpm = dynBaseRpm + steerRpm;
    targetRightRpm = dynBaseRpm - steerRpm;
    s_leftRpm = Task5_Clamp(targetLeftRpm, 0.0F, T5_MAX_WHEEL_RPM);
    s_rightRpm = Task5_Clamp(targetRightRpm, 0.0F, T5_MAX_WHEEL_RPM);
    Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);
}

/* 秒表校准后的累计毫秒数；与 OLED 显示使用同一套换算。 */
static uint32_t Task5_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T5_TICK_MS) * T5_STOPWATCH_CAL_SCALE);
}

/* "GO:0.3s" 形式的剩余时间文本，仅用于 T5_STATE_AFTER_LINE 阶段。 */
static void Task5_FormatRemaining(const char *prefix, uint32_t elapsedMs)
{
    uint32_t remainingMs;
    uint32_t tenths;
    uint32_t idx = 0U;

    if (elapsedMs >= T5_AFTER_LINE_MS) {
        remainingMs = 0U;
    } else {
        remainingMs = T5_AFTER_LINE_MS - elapsedMs;
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

static const char *Task5_GetPhaseStatus(void)
{
    switch (s_state) {
    case T5_STATE_RUN:
        if (s_lineStopped) {
            return "LOST";
        }
        return s_finishArmed ? "LINE" : "ARM";

    case T5_STATE_AFTER_LINE:
        Task5_FormatRemaining("GO:", s_afterLineElapsedMs);
        return s_phaseStatusBuf;

    case T5_STATE_DECEL:
        if (s_lineStopped) {
            return "LOST";
        }
        return "DEC";

    case T5_STATE_STOP_LEFT:
    case T5_STATE_STOP_RIGHT:
        return "STOP";

    case T5_STATE_FINISHED:
        return "DONE";

    default:
        return "INIT";
    }
}

/* 秒表文本："T:12.3s LINE" 等，停车完成后追加 "DONE"；两段式启动的等待态各给一行提示。 */
const char *Task5_GetUiStatus(void)
{
    const char *phase;
    uint32_t totalMs;
    uint32_t secWhole;
    uint32_t tenths;
    uint32_t idx = 0U;
    uint32_t n = 0U;
    uint32_t value;
    char digits[10];

    if (s_state == T5_STATE_IDLE) {
        return "T5 K3=BALL";
    }
    if (s_state == T5_STATE_WAIT_BALL_CONTROL) {
        return "T5 B WAIT";
    }
    if (s_state == T5_STATE_BALL_READY) {
        return "T5 K3=GO";
    }
    if (s_state == T5_STATE_BALL_RECOVER_WAIT) {
        return "T5 B RECOV";
    }
#if T5_MOTORS_DISABLED_MANUAL_TEST
    if (s_state == T5_STATE_MANUAL_BALANCE) {
        return "T5 MANUAL";
    }
#endif

    phase = Task5_GetPhaseStatus();
    totalMs = Task5_GetElapsedMs();
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

/*
 * 运行态 K3 按下沿（UIMENU → RobotCore_ConfirmTask 转发）。
 * 只置标志、不在这里操作电机，实际动作统一交给同一拍随后运行的 OnLoop
 * 状态机。第一次按启动球杆平衡，第二次按发车；其余状态下按 K3 无效。
 */
void Task5_OnConfirm(void)
{
    if (s_state == T5_STATE_IDLE) {
        s_startBallRequested = true;
    } else if (s_state == T5_STATE_BALL_READY) {
        s_startCarRequested = true;
    }
}

void Task5_OnEnter(void)
{
    Pid_Init(&s_pid, T5_KP, T5_KI, T5_KD, T5_INTEGRAL_LIMIT, T5_MAX_STEER_RPM);
    /* 进题目只做复位，什么都不动，等第一次 K3。 */
    s_state              = T5_STATE_IDLE;
    s_lastSteerRpm       = 0.0F;
    s_appliedSteerRpm    = 0.0F;
    s_leftRpm            = 0.0F;
    s_rightRpm           = 0.0F;
    s_filteredError      = 0.0F;
    s_sentLeftRpm        = 0;
    s_sentRightRpm       = 0;
    s_lineLostTicks      = 0U;
    s_enableSettleTicks  = 0U;
    s_sendLeftNext       = true;
    s_finishArmed        = false;
    s_armTicks           = 0U;
    s_lineStopped        = false;
    s_afterLineElapsedMs = 0U;
    s_straightRpm        = 0.0F;
    s_rampBaseRpm        = 0.0F;
    s_elapsedTicks       = 0U;
    s_ballControlRequested = false;
    s_wheelsStarted      = false;
    s_runCompleted       = false;
    s_startBallRequested = false;
    s_startCarRequested  = false;
    s_finishBeeped       = false;
}

void Task5_OnLoop(void)
{
    float rawError;
    bool lineFound;
    uint32_t hitCount;
    AppBallControlStatus_t ballStatus;

    AppBallControl_GetStatus(&ballStatus);

    /* 秒表只统计"小车已发车之后"的时间：三个等待态、故障恢复态和已完成态都不计时。 */
    if ((s_state != T5_STATE_IDLE) &&
        (s_state != T5_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T5_STATE_BALL_READY) &&
        (s_state != T5_STATE_BALL_RECOVER_WAIT) &&
#if T5_MOTORS_DISABLED_MANUAL_TEST
        (s_state != T5_STATE_MANUAL_BALANCE) &&
#endif
        (s_state != T5_STATE_FINISHED)) {
        s_elapsedTicks++;
    }

    /* 第一次 K3 之前什么都不做：ID1 不闭环、轮子不使能，摆杆保持归零后的水平位置。 */
    if (s_state == T5_STATE_IDLE) {
        if (s_startBallRequested) {
            s_startBallRequested = false;
            s_state = T5_STATE_WAIT_BALL_CONTROL;
        }
        return;
    }

    /*
     * 钢珠闭环触发 FAULT_EDGE（球触边）后会一直锁在回水平的位置，需要主动走一遍
     * "停止→等待释放→重新请求"的握手才能恢复，做法与题目四完全一致。
     */
    if ((s_state != T5_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T5_STATE_BALL_RECOVER_WAIT) &&
        (ballStatus.state == APP_BALL_CONTROL_FAULT_EDGE)) {
        AppBallControl_RequestStop();
        s_ballControlRequested = false;
        s_state = T5_STATE_BALL_RECOVER_WAIT;
    }

    if (s_state == T5_STATE_BALL_RECOVER_WAIT) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) {
            s_state = T5_STATE_WAIT_BALL_CONTROL;
        }
        return;
    }

    /* 第一次 K3 后：请求球杆闭环，等后台真正接管 ID1。 */
    if (s_state == T5_STATE_WAIT_BALL_CONTROL) {
        if (!s_ballControlRequested) {
            s_ballControlRequested = AppBallControl_RequestTargetWithProfile(
                T5_BALL_TARGET_X_PX, &s_task5BallProfile);
        }

        if (s_ballControlRequested &&
            (ballStatus.targetPx == T5_BALL_TARGET_X_PX) &&
            ((ballStatus.state == APP_BALL_CONTROL_RUNNING) ||
             (ballStatus.state == APP_BALL_CONTROL_HOLDING))) {
            if (s_runCompleted) {
                /* 本次任务已经跑完过一次，恢复握手不应把状态机复活成 RUN。 */
                s_state = T5_STATE_FINISHED;
            } else if (s_wheelsStarted) {
                /* 钢珠故障恢复场景：轮子早已在跑，直接回到循迹，不重新走使能时序。 */
                s_state = T5_STATE_RUN;
            } else {
#if T5_MOTORS_DISABLED_MANUAL_TEST
                s_state = T5_STATE_MANUAL_BALANCE;
#else
                s_state = T5_STATE_BALL_READY;
#endif
            }
        }
        return;
    }

    /* 球杆平衡已工作、小车待发：持续保持钢珠伺服，等第二次 K3 才走轮子使能时序。 */
    if (s_state == T5_STATE_BALL_READY) {
        if (s_startCarRequested) {
            s_startCarRequested = false;
            s_wheelsStarted = true;
            /* 起步斜坡从 0 重新爬，保证每次发车都是缓慢加速。 */
            s_rampBaseRpm = 0.0F;
            /*
             * 本题 deferVideoStart=true（app_robot_core.c 题目表），进题目时不会
             * 自动开始录像；真正发车这一刻才通知视觉端开始录像。
             */
            RobotCore_NotifyTaskStarted(4U);
            s_state = T5_STATE_RESET_DISABLE;
        }
        return;
    }

#if T5_MOTORS_DISABLED_MANUAL_TEST
    /* 手动调参模式：电机不使能、不驱动，仅后台 BALLCTRL 保持钢球在目标位置。 */
    if (s_state == T5_STATE_MANUAL_BALANCE) {
        return;
    }
#endif

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
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T5_STOP_EMM_ACC);
        s_sentLeftRpm = 0;
        s_state = T5_STATE_STOP_RIGHT;
        return;
    }
    if (s_state == T5_STATE_STOP_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T5_STOP_EMM_ACC);
        s_sentRightRpm = 0;
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        s_state = T5_STATE_FINISHED;
        return;
    }
    if (s_state == T5_STATE_FINISHED) {
        return;
    }

    if (s_state == T5_STATE_AFTER_LINE) {
        /*
         * 判定到达终点后：不重新读线、不做任何转向修正，两轮都按 s_straightRpm
         * （进入本阶段那一刻的平均转速）直行，即"保持当前速度直直地前进"。
         */
        s_afterLineElapsedMs += T5_TICK_MS;
        s_leftRpm = s_straightRpm;
        s_rightRpm = s_straightRpm;
        Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);

        /* 蜂鸣器第二次响、结束录像在 1500ms 走完前的 800ms 时刻先触发，只响一次。 */
        if ((!s_finishBeeped) && (s_afterLineElapsedMs >= T5_BEEP_DELAY_MS)) {
            s_finishBeeped = true;
            BspBuzzer_BeepShort();
            RobotCore_NotifyTaskFinished(4U);
        }

        if (s_afterLineElapsedMs >= T5_AFTER_LINE_MS) {
            s_state = T5_STATE_DECEL;
        }
        return;
    }

    if (s_state == T5_STATE_DECEL) {
        /* 与起步同一根斜坡反向线性降速，只压这里的下降，不影响别处。 */
        s_rampBaseRpm -= T5_RAMP_RPM_PER_SEC * T5_DT_SEC;
        if (s_rampBaseRpm <= 0.0F) {
            s_rampBaseRpm = 0.0F;
        }

        lineFound = Task5_GetLineError(&rawError, &hitCount);
        Task5_UpdateTracking(lineFound, rawError);

        if (s_lineStopped) {
            s_leftRpm = 0.0F;
            s_rightRpm = 0.0F;
            Task5_ApplyWheelRpm(s_leftRpm, s_rightRpm);
        } else {
            Task5_ApplyDecelTracking();
        }

        if (s_rampBaseRpm <= 0.0F) {
            s_state = T5_STATE_STOP_LEFT;
        }
        return;
    }

    /*
     * T5_STATE_RUN：正常循迹，武装后首次命中 >=5 路、且发车后累计时间已
     * >= T5_FINISH_MIN_ELAPSED_MS 才判定到达终点（环形赛道起跑线=终点线，
     * 时间下限防止刚发车不久就在同一根线上被误判）。
     */
    lineFound = Task5_GetLineError(&rawError, &hitCount);

    if (s_finishArmed && (hitCount >= T5_STOP_LINE_HIT_MIN) &&
        ((s_elapsedTicks * T5_TICK_MS) >= T5_FINISH_MIN_ELAPSED_MS)) {
        s_state = T5_STATE_AFTER_LINE;
        s_afterLineElapsedMs = 0U;
        /* 记录当前平均转速，AFTER_LINE 阶段两轮都按此值直行，不再有转向修正。 */
        s_straightRpm = (s_leftRpm + s_rightRpm) * 0.5F;
        /* 直行期间不跑转向控制；转入 DECEL 重新循迹时从 0 开始平滑爬升，不带旧转向残留。 */
        s_lastSteerRpm = 0.0F;
        s_appliedSteerRpm = 0.0F;
        s_runCompleted = true;
        return;
    }

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
}

void Task5_OnExit(void)
{
    /* K4 退出保留硬安全收尾：急停后失能左右轮。 */
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T5_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);

    /* 任务五结束后才释放 ID1，运行、终点保持和缓停期间保持钢珠目标位置。 */
    AppBallControl_RequestStop();

    Pid_Reset(&s_pid);
    /* 回到未启动态：下次进题目仍需重新按两次 K3。 */
    s_state              = T5_STATE_IDLE;
    s_lastSteerRpm       = 0.0F;
    s_appliedSteerRpm    = 0.0F;
    s_finishArmed        = false;
    s_armTicks           = 0U;
    s_lineStopped        = false;
    s_afterLineElapsedMs = 0U;
    s_straightRpm        = 0.0F;
    s_rampBaseRpm        = 0.0F;
    s_elapsedTicks       = 0U;
    s_ballControlRequested = false;
    s_wheelsStarted      = false;
    s_runCompleted       = false;
    s_startBallRequested = false;
    s_startCarRequested  = false;
    s_finishBeeped       = false;
}
