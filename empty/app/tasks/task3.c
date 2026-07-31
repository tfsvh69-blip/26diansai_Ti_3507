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
 * 第 3 题：钢珠平衡 + 循迹框架（2026-08 由第 4 题整体移植）
 *
 * 本题当前是第 4 题框架的完整副本：循迹、入场、丢线保护、终点保护、定时减速
 * 与钢珠闭环流程全部一致，行驶期间通过 BALLCTRL 同时控制 ID1 把钢珠保持在
 * T3_BALL_TARGET_X_PX。
 *
 * ⚠️ 按仓库的【控制参数隔离规则】，下面所有参数都是本题私有的 T3_* 副本：
 * 初值虽然复制自第 4 题，但此后两题各调各的，改这里【不会】影响第 4 题，
 * 反之亦然。后续本题分化出自己的业务逻辑时，直接改本文件即可。
 * ================================================================== */

/* PID 增益、积分限幅与转向输出限幅。 */
#define T3_KP                              (5.0F)
#define T3_KI                              (0.15F)
#define T3_KD                              (0.2F)
#define T3_INTEGRAL_LIMIT                  (20.0F)
#define T3_MAX_STEER_RPM                   (100.0F)

/* 基础速度和单轮安全范围。 */
#define T3_BASE_RPM                        (110.0F)
#define T3_MIN_WHEEL_RPM                   (5.0F)
#define T3_MAX_WHEEL_RPM                   (230.0F)

/* 正常循迹速度模式加速度档位。 */
#define T3_EMM_ACC                         (120U)

/*
 * 缓停参数：T3_STOP_AFTER_MS 固定本题开始缓停的时间；
 * T3_STOP_EMM_ACC 由使用者按实车需要传给速度模式 0 RPM 帧。
 * 数值越小减速越平缓，越大越接近立即停；可直接修改后重新烧录。
 */
#define T3_STOP_AFTER_MS                   (6500U)
#define T3_STOP_EMM_ACC                    (80U)

/* UIMENU 固定控制周期。 */
#define T3_DT_SEC                          (0.03F)
#define T3_TICK_MS                         (30U)

/* 丢线、入场和共享 UART1 总线时序参数。 */
#define T3_LINE_LOST_TICKS                 (20U)
#define T3_RESET_SETTLE_TICKS              (2U)
#define T3_ENABLE_SETTLE_TICKS             (6U)
#define T3_EMM_CMD_GAP_MS                  (6U)

/* 误差滤波、死区和转弯减速参数。 */
#define T3_ERROR_FILTER_ALPHA              (0.5F)
#define T3_ERROR_DEADBAND                  (1.0F)
#define T3_CORNER_SLOWDOWN_GAIN            (0.6F)
#define T3_MIN_BASE_RPM                    (10.0F)

/* 终点保护参数。 */
#define T3_FINISH_ARM_HIT_MAX              (3U)
#define T3_FINISH_ARM_TICKS                (15U)
#define T3_FINISH_HIT_MIN                  (6U)
#define T3_FINISH_HIT_TICKS                (1U)

/* 秒表与定时减速参数。 */
#define T3_STOPWATCH_CAL_SCALE             (0.897F)
#define T3_DECEL_START_MS                  (14500U)
#define T3_DECEL_GRADIENT_RPM_PER_SEC      (60.0F)
#define T3_DECEL_MIN_RPM                   (10.0F)

/*
 * 本题钢珠平衡参数。车辆必须等后台闭环已经按此 profile 进入实际控制后才起步。
 */
#define T3_BALL_TARGET_X_PX                 (350)      /* 2026-08 新曲柄摇杆机构实测中心点 */

/* ---- A. 机械/视觉实测常量：由标定得出，不是调参旋钮，换硬件才重测 ---- */
/*
 * 2026-08-01 现场标定结果（视觉：菜单页读 X；机械：题目六 T6_RAMP_TEST 斜坡），
 * 对应旧"丝杆升降+连杆"机构：
 *   视觉比例  520px / 213mm = 2.441 px/mm（球心可达范围 X∈[66, 586]）
 *   静止噪声  ±2px（±0.8mm）→ 决定死区下限，也说明不需要重滤波
 *   脱离阈值  B = 1810 脉冲（4.53mm 升程 / 1.04° 倾角）
 *   真实水平  L = -70 脉冲（≈0.04°，机械基本是正的）
 *
 * 2026-08 机构变更为"电机直驱摇臂（±90°内旋转）"后的重新标定哲学：
 *   旧丝杆+螺母本身摩擦极大，占满了整个可用命令范围，必须靠 B 硬顶过去
 *   （见 docs/BALL_CONTROL.md §5）；直驱摇臂去掉了丝杆螺母这一大摩擦源，
 *   剩下的只是轴承转动摩擦和钢珠滚动摩擦，预期小得多。因此 B/L 先不做
 *   独立标定脚本测量，直接从 0 开始跑纯 PD（B=0 时下面的库仑摩擦前馈项
 *   在公式里自动归零，退化为纯 PD，不用改任何控制逻辑）：
 *   - 若实测出现"球离目标一截距离就完全僵住不动"，现场把 B 从 0 增量
 *     试大，观察消失即可，不再靠独立斜坡测试固定一个阈值——静摩擦本身
 *     随接触点/磨损/装配变化，固定常数覆盖全程不如现场调参鲁棒；
 *   - L 同理从 0 开始，靠"球停在偏目标固定像素、误差稳定不变"这个现象
 *     现场微调，不需要专门标定步骤。
 * 视觉比例、静止噪声与传动机构无关，继续沿用旧值。
 */
#define T3_BALL_FRICTION_FF_PULSE           (0.0F)     /* = B，先按 0 跑纯 PD，见上方说明 */
#define T3_BALL_LEVEL_TRIM_PULSE            (0)        /* = L，字段是 int32_t，脉冲数必须写整数 */
#define T3_BALL_FILTER_ALPHA                (0.10F)    /* 噪声仅 ±2px，无需压到 0.2 换来 165ms 滞后 */
#define T3_BALL_FILTER_BETA                 (0.10F)
/*
 * 2026-08 新曲柄摇杆机构方向实测（任务六 900 脉冲方向测试）：
 *   正脉冲 = 抬升摇杆；摇杆抬得越高，钢珠越往 X 变小方向移动。
 * 控制律 output = LEVEL_TRIM + SIGN×(Kx×error − Kv×velocity)，error = targetPx − 球位置。
 * error>0（目标在球右边）需要球向 +X 移动 → 必须降低摇杆 → output 须为负；
 * Kx>0 时 error>0 令 Kx×error>0，要让 output 为负必须 SIGN=-1。这是实测
 * 结论，不是猜测；如果实车验证方向反了，只改这一个数，不要改 emm42_robot.c
 * 的全局标定表。
 */
#define T3_BALL_OUTPUT_SIGN                 (-1.0F)

/* ---- B. 真正的调参旋钮 ---- */
/*
 * 旧丝杆机构曾有物理依据的系数 0.171（= (5/7)g / 400脉冲每mm / 250mm摆杆 *
 * 2.441px每mm）依赖丝杆导程换算，直驱摇臂后完全失效，需要新摇臂/连杆的实际
 * 几何尺寸才能重新推导，本次不臆测新系数。
 *
 * 调参顺序照 docs/CONTROL_ALGORITHM.md §10.2/§10.3 现象表：
 *   球对误差反应很弱/很慢 → 加大 Kx；
 *   目标附近来回轻微振荡、幅度不变或缓慢衰减 → 停止加 Kx，固定住，转去从 0
 *     开始加 Kv 压振荡；
 *   球剧烈振荡/越振越猛/摆杆动作剧烈有异响 → Kx 过大，退回更小值重新找临界点；
 *   Kv 加到很大仍压不住振荡 → 大概率是 Kx 选大了，回去减小 Kx 而不是无限加 Kv；
 *   摆杆走一下停一下、球一段段前进（粘滑）→ 查 FRICTION_FF 与 Kx，见 §10.4。
 */
#define T3_BALL_KX_PULSE_PER_PX             (0.38F)
#define T3_BALL_KV_PULSE_PER_PXPS           (0.6F)
/*
 * 到位死区，同时就是静态精度上限：4px ≈ 1.6mm，取实测噪声 ±2px 的两倍裕度。
 * 误差进此范围【且球基本停住】才回真实水平点、停止驱动。
 */
#define T3_BALL_SETTLE_DEADBAND_PX          (4.0F)
/*
 * 判定钢珠"在运动"的速度门限：高于它按 sign(v) 补动摩擦，低于它过渡到按
 * sign(pd) 补静摩擦；同时是上面"到位保持"的速度条件。
 * 15px/s ≈ 6mm/s，明显高于速度估计噪声量级(±10px/s 的一半)。
 */
#define T3_BALL_FF_VEL_BLEND_PXPS           (15.0F)

/* ---- C. 执行器与显示 ---- */
/*
 * 摆杆每帧（20ms）行程能力 = POS_RPM * 3200/60 * 0.02 = POS_RPM * 1.067 脉冲。
 * 直驱摇臂下同样 RPM 对应的角速度远大于旧丝杆机构（没有导程折算这层缓冲），
 * 具体数值按现象判断——命令发出但摆杆没走到位/响应打折扣→调大，
 * 过冲很猛或有异响/失步→调小。
 */
#define T3_BALL_POS_RPM                     (400U)
/*
 * 位置模式加速度档位。协议公式（手册§6.3.1，emm42_v5.h）：每升 1RPM 需要
 * (256−acc)×50us，【acc 越大爬升越快】，acc=0 是特例——不走曲线、直接瞬间给到
 * 目标速度（最快，也最抖），不是"最慢"，这一点容易搞反。
 *
 * ⚠️ 外环约 17~25ms 就刷新一条全新目标，曲线基本不可能走完就被打断重新规划，
 * 实际能达到的转速远低于 POS_RPM：acc=120 时 20ms 只能爬到约 2.9RPM（约 1.6
 * 脉冲/帧），acc=230 时约 15RPM（约 8.2 脉冲/帧）。
 * 调参方向：还抖 → 减小 acc；感觉跟不上球、响应发软 → 增大 acc。
 */
#define T3_BALL_POS_ACC                     (170U)
/*
 * 软件平滑：每帧实际下发的绝对目标相对上一帧最多变化多少脉冲，0=不限速。
 * 把 PD 输出的目标突变摊到连续多帧上，抑制目标跳变造成的机械冲击；小车行驶
 * 中球被持续扰动时尤其有用。调小 → 摆杆动作更柔和但跟踪更迟钝；调大 →
 * 越接近不限速的原始手感；0 = 完全关闭。
 *
 * ⚠️ 它【解决不了】"摆杆走一下停一下"的分段感，原因见 docs/CONTROL_ALGORITHM.md
 * §10.4：限速只会让每帧增量更小、更容易走完，反而加重。分段感要查静摩擦
 * （FRICTION_FF）和 Kx。
 */
#define T3_BALL_MAX_PULSE_STEP              (0U)
#define T3_BALL_HOLD_POSITION_PX            (6.0F)   /* 以下三项仅影响 OLED 的 B:HOLD 显示 */
#define T3_BALL_HOLD_VELOCITY_PXPS          (10.0F)
#define T3_BALL_HOLD_TIME_MS                (500U)

/*
 * 手动调参开关：置 1 时电机不启动，仅 BALLCTRL 保持钢球平衡；
 * 用手推拉小车模拟加减速扰动，调好参数后改回 0 即可恢复完整功能。
 */
#define T3_MOTORS_DISABLED_MANUAL_TEST      (1U)

/* 本题在题目表 s_robotTasks[] 中的下标（第 3 题 = 索引 2）。 */
#define T3_TASK_INDEX                       (2U)

typedef enum {
    T3_STATE_WAIT_BALL_CONTROL = 0,
    T3_STATE_BALL_RECOVER_WAIT, /* 钢珠闭环触发 FAULT_EDGE 后，等待其停止/释放 ID1 再重新请求 */
    T3_STATE_MANUAL_BALANCE,   /* 电机不启动，仅后台 BALLCTRL 保持钢球平衡 */
    T3_STATE_RESET_DISABLE,
    T3_STATE_RESET_WAIT,
    T3_STATE_ENABLE_LEFT,
    T3_STATE_ENABLE_LEFT_WAIT,
    T3_STATE_ENABLE_RIGHT,
    T3_STATE_ENABLE_RIGHT_WAIT,
    T3_STATE_RUN,
    T3_STATE_STOP,
    T3_STATE_TIME_STOP_LEFT,
    T3_STATE_TIME_STOP_RIGHT,
    T3_STATE_TIME_STOPPED,
    T3_STATE_FINISHED
} Task3State_t;

static const AppBallControlProfile_t s_task3BallProfile = {
    T3_BALL_FILTER_ALPHA,
    T3_BALL_FILTER_BETA,
    T3_BALL_OUTPUT_SIGN,
    T3_BALL_KX_PULSE_PER_PX,
    T3_BALL_KV_PULSE_PER_PXPS,
    T3_BALL_LEVEL_TRIM_PULSE,
    T3_BALL_SETTLE_DEADBAND_PX,
    T3_BALL_FRICTION_FF_PULSE,
    T3_BALL_FF_VEL_BLEND_PXPS,
    T3_BALL_POS_RPM,
    T3_BALL_POS_ACC,
    T3_BALL_MAX_PULSE_STEP,
    T3_BALL_HOLD_POSITION_PX,
    T3_BALL_HOLD_VELOCITY_PXPS,
    T3_BALL_HOLD_TIME_MS
};

static Task3State_t s_state;
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

static float Task3_Clamp(float value, float minValue, float maxValue)
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
static void Task3_ClampWheelPair(float *leftRpm, float *rightRpm)
{
    float hi = (*leftRpm > *rightRpm) ? *leftRpm : *rightRpm;
    float lo;
    float shift;

    if (hi > T3_MAX_WHEEL_RPM) {
        shift      = hi - T3_MAX_WHEEL_RPM;
        *leftRpm  -= shift;
        *rightRpm -= shift;
    }

    lo = (*leftRpm < *rightRpm) ? *leftRpm : *rightRpm;
    if (lo < T3_MIN_WHEEL_RPM) {
        shift      = T3_MIN_WHEEL_RPM - lo;
        *leftRpm  += shift;
        *rightRpm += shift;
    }

    *leftRpm  = Task3_Clamp(*leftRpm, T3_MIN_WHEEL_RPM, T3_MAX_WHEEL_RPM);
    *rightRpm = Task3_Clamp(*rightRpm, T3_MIN_WHEEL_RPM, T3_MAX_WHEEL_RPM);
}

/* 正常循迹每拍只发一帧，左右轮交替更新，避免共享总线背靠背丢帧。 */
static void Task3_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t leftInt = (leftRpm >= 0.0F) ? (int32_t)(leftRpm + 0.5F)
                                        : (int32_t)(leftRpm - 0.5F);
    int32_t rightInt = (rightRpm >= 0.0F) ? (int32_t)(rightRpm + 0.5F)
                                          : (int32_t)(rightRpm - 0.5F);

    if (s_sendLeftNext) {
        if (leftInt != s_sentLeftRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_L, (int16_t)leftInt, T3_EMM_ACC);
            s_sentLeftRpm = leftInt;
        }
    } else {
        if (rightInt != s_sentRightRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_R, (int16_t)rightInt, T3_EMM_ACC);
            s_sentRightRpm = rightInt;
        }
    }
    s_sendLeftNext = !s_sendLeftNext;
}

/* 按物理左→右的 LINE8→LINE1 顺序计算加权位置误差。 */
static bool Task3_GetLineError(float *error, uint32_t *hitCountOut)
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

static uint32_t Task3_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T3_TICK_MS) * T3_STOPWATCH_CAL_SCALE);
}

const char *Task3_GetUiStatus(void)
{
    uint32_t totalMs = Task3_GetElapsedMs();
    uint32_t secWhole = totalMs / 1000U;
    uint32_t tenths = (totalMs / 100U) % 10U;
    uint32_t idx = 0U;
    char digits[10];
    uint32_t n = 0U;
    uint32_t value = secWhole;

    if (s_state == T3_STATE_WAIT_BALL_CONTROL) {
        return "T3 B WAIT";
    }
    if (s_state == T3_STATE_BALL_RECOVER_WAIT) {
        return "T3 B RECOV";
    }
#if T3_MOTORS_DISABLED_MANUAL_TEST
    if (s_state == T3_STATE_MANUAL_BALANCE) {
        return "T3 MANUAL";
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
    if ((s_state == T3_STATE_FINISHED) || (s_state == T3_STATE_TIME_STOPPED)) {
        s_uiStatusBuf[idx++] = ' ';
        s_uiStatusBuf[idx++] = 'D';
        s_uiStatusBuf[idx++] = 'O';
        s_uiStatusBuf[idx++] = 'N';
        s_uiStatusBuf[idx++] = 'E';
    }
    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}

void Task3_OnEnter(void)
{
    Pid_Init(&s_pid, T3_KP, T3_KI, T3_KD, T3_INTEGRAL_LIMIT, T3_MAX_STEER_RPM);
    s_state             = T3_STATE_WAIT_BALL_CONTROL;
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

void Task3_OnLoop(void)
{
    float rawError;
    float targetLeftRpm;
    float targetRightRpm;
    uint32_t hitCount;
    AppBallControlStatus_t ballStatus;

    AppBallControl_GetStatus(&ballStatus);

    if ((s_state != T3_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T3_STATE_BALL_RECOVER_WAIT) &&
        (s_state != T3_STATE_MANUAL_BALANCE) &&
        (s_state != T3_STATE_FINISHED) && (s_state != T3_STATE_TIME_STOPPED)) {
        s_elapsedTicks++;
    }

    /*
     * 钢珠闭环触发 FAULT_EDGE（球触边）后会一直锁在回水平的位置，不会自己恢复，
     * 必须重新走一遍"停止→等待释放→重新请求"的握手才能恢复到目标位置；否则表现
     * 就是"进了本题但钢珠不再被伺服"。这里主动检测并恢复，不需要用户手动
     * 退出重进。只要不是正在做这套握手本身，任何时候（含 RUN/STOP/已完成等待
     * K4 退出期间）检测到故障都立即触发恢复。
     */
    if ((s_state != T3_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T3_STATE_BALL_RECOVER_WAIT) &&
        (ballStatus.state == APP_BALL_CONTROL_FAULT_EDGE)) {
        AppBallControl_RequestStop();
        s_ballControlRequested = false;
        s_state = T3_STATE_BALL_RECOVER_WAIT;
    }

    /* 等 BALLCTRL 真正回到 OFF 才能重新请求，避免与刚发出的停止命令产生竞态。 */
    if (s_state == T3_STATE_BALL_RECOVER_WAIT) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) {
            s_state = T3_STATE_WAIT_BALL_CONTROL;
        }
        return;
    }

    /* 先确保 ID1 已按本题专属参数开始闭环，随后才允许车辆使能和起步。 */
    if (s_state == T3_STATE_WAIT_BALL_CONTROL) {
        if (!s_ballControlRequested) {
            s_ballControlRequested = AppBallControl_RequestTargetWithProfile(
                T3_BALL_TARGET_X_PX, &s_task3BallProfile);
        }

        if (s_ballControlRequested &&
            (ballStatus.targetPx == T3_BALL_TARGET_X_PX) &&
            ((ballStatus.state == APP_BALL_CONTROL_RUNNING) ||
             (ballStatus.state == APP_BALL_CONTROL_HOLDING))) {
            if (s_wheelsStarted) {
                /* 钢珠故障恢复场景：轮子早已在跑，直接回到循迹，不重新走使能时序。 */
                s_state = T3_STATE_RUN;
            } else {
#if T3_MOTORS_DISABLED_MANUAL_TEST
                s_state = T3_STATE_MANUAL_BALANCE;
#else
                s_wheelsStarted = true;
                s_state = T3_STATE_RESET_DISABLE;
#endif
            }
        }
        return;
    }

#if T3_MOTORS_DISABLED_MANUAL_TEST
    /* 手动调参模式：电机不使能、不驱动，仅后台 BALLCTRL 保持钢球在目标位置。*/
    if (s_state == T3_STATE_MANUAL_BALANCE) {
        return;
    }
#endif

    if (s_state == T3_STATE_RESET_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_enableSettleTicks = T3_RESET_SETTLE_TICKS;
        s_state = T3_STATE_RESET_WAIT;
        return;
    }
    if (s_state == T3_STATE_RESET_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T3_STATE_ENABLE_LEFT;
        return;
    }
    if (s_state == T3_STATE_ENABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_enableSettleTicks = T3_ENABLE_SETTLE_TICKS;
        s_state = T3_STATE_ENABLE_LEFT_WAIT;
        return;
    }
    if (s_state == T3_STATE_ENABLE_LEFT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T3_STATE_ENABLE_RIGHT;
        return;
    }
    if (s_state == T3_STATE_ENABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_enableSettleTicks = T3_ENABLE_SETTLE_TICKS;
        s_state = T3_STATE_ENABLE_RIGHT_WAIT;
        return;
    }
    if (s_state == T3_STATE_ENABLE_RIGHT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T3_STATE_RUN;
    }

    /* 计时到：两拍分别给左右轮发送带加速度的速度模式 0 RPM，不能走急停接口。 */
    if (s_state == T3_STATE_TIME_STOP_LEFT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T3_STOP_EMM_ACC);
        s_sentLeftRpm = 0;
        s_state = T3_STATE_TIME_STOP_RIGHT;
        return;
    }
    if (s_state == T3_STATE_TIME_STOP_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T3_STOP_EMM_ACC);
        s_sentRightRpm = 0;
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        s_state = T3_STATE_TIME_STOPPED;
        RobotCore_NotifyTaskFinished(T3_TASK_INDEX);
        return;
    }
    if (s_state == T3_STATE_FINISHED) {
        return;
    }

    /* 只统计正常循迹状态的前进时间；丢线停车期间不计入。 */
    if (s_state == T3_STATE_RUN) {
        s_runTicks++;
        if ((s_runTicks * T3_TICK_MS) >= T3_STOP_AFTER_MS) {
            s_state = T3_STATE_TIME_STOP_LEFT;
            return;
        }
    }

    if (Task3_GetLineError(&rawError, &hitCount)) {
        if (s_state == T3_STATE_STOP) {
            s_filteredError = rawError;
        } else {
            s_filteredError += T3_ERROR_FILTER_ALPHA * (rawError - s_filteredError);
        }
        if (s_state == T3_STATE_STOP) {
            Pid_Reset(&s_pid);
        }
        /*
         * 0 RPM 缓停期间仍持续采样和更新 PID，避免状态机锁死后丢失循迹状态；
         * 但不能重新切回 RUN 并下发非零速度，否则会覆盖驱动器正在执行的 0 RPM 曲线。
         */
        if (s_state != T3_STATE_TIME_STOPPED) {
            s_state = T3_STATE_RUN;
        }
        s_lineLostTicks = 0U;
        {
            float pidError = s_filteredError;
            if ((pidError > -T3_ERROR_DEADBAND) && (pidError < T3_ERROR_DEADBAND)) {
                pidError = 0.0F;
            }
            s_lastSteerRpm = Pid_Update(&s_pid, pidError, T3_DT_SEC);
        }
    } else {
        s_lineLostTicks++;
        if (s_lineLostTicks >= T3_LINE_LOST_TICKS) {
            s_state = T3_STATE_STOP;
        }
    }

    if (!s_finishArmed) {
        if (hitCount <= T3_FINISH_ARM_HIT_MAX) {
            s_armTicks++;
            if (s_armTicks >= T3_FINISH_ARM_TICKS) {
                s_finishArmed = true;
            }
        } else {
            s_armTicks = 0U;
        }
    } else if (s_state != T3_STATE_FINISHED) {
        if (hitCount >= T3_FINISH_HIT_MIN) {
            s_finishHitTicks++;
            if (s_finishHitTicks >= T3_FINISH_HIT_TICKS) {
                s_state = T3_STATE_FINISHED;
            }
        } else {
            s_finishHitTicks = 0U;
        }
    }

    if (s_state == T3_STATE_FINISHED) {
        /* 终点保护：这是异常/终点保护，仍需要立即急停。 */
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
        vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        RobotCore_NotifyTaskFinished(T3_TASK_INDEX);
        return;
    }

    if (s_state == T3_STATE_RUN) {
        float steerAbs = (s_lastSteerRpm >= 0.0F) ? s_lastSteerRpm : -s_lastSteerRpm;
        float dynBaseRpm = T3_BASE_RPM - T3_CORNER_SLOWDOWN_GAIN * steerAbs;
        dynBaseRpm = Task3_Clamp(dynBaseRpm, T3_MIN_BASE_RPM, T3_BASE_RPM);
        {
            uint32_t elapsedMs = Task3_GetElapsedMs();
            if (elapsedMs >= T3_DECEL_START_MS) {
                float decelSec = (float)(elapsedMs - T3_DECEL_START_MS) * 0.001F;
                float decelBaseRpm = T3_BASE_RPM - T3_DECEL_GRADIENT_RPM_PER_SEC * decelSec;
                decelBaseRpm = Task3_Clamp(decelBaseRpm, T3_DECEL_MIN_RPM, T3_BASE_RPM);
                if (decelBaseRpm < dynBaseRpm) {
                    dynBaseRpm = decelBaseRpm;
                }
            }
        }
        targetLeftRpm = dynBaseRpm + s_lastSteerRpm;
        targetRightRpm = dynBaseRpm - s_lastSteerRpm;
        Task3_ClampWheelPair(&targetLeftRpm, &targetRightRpm);
    } else {
        targetLeftRpm = 0.0F;
        targetRightRpm = 0.0F;
    }

    s_leftRpm = targetLeftRpm;
    s_rightRpm = targetRightRpm;
    if (s_state != T3_STATE_TIME_STOPPED) {
        Task3_ApplyWheelRpm(s_leftRpm, s_rightRpm);
    }
}

void Task3_OnExit(void)
{
    /* K4 退出保留硬安全收尾：急停后失能左右轮。 */
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);

    /* 本题结束后才释放 ID1，运行和缓停期间保持钢珠位置目标。 */
    AppBallControl_RequestStop();

    Pid_Reset(&s_pid);
    s_state = T3_STATE_STOP;
    s_finishArmed = false;
    s_armTicks = 0U;
    s_finishHitTicks = 0U;
    s_runTicks = 0U;
    s_ballControlRequested = false;
    s_wheelsStarted = false;
}
