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
#define T4_BALL_TARGET_X_PX                 (350)      /* 2026-08 新曲柄摇杆机构实测中心点，替换旧值 320 */

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
#define T4_BALL_FRICTION_FF_PULSE           (25.0F)     /* 2026-08 隔离测试：50 时目标点附近持续等幅抖动，
                                                            疑似前馈踹一脚→速度超阈值→踹一脚的自激循环，
                                                            先退回 0 验证是不是它，见下方说明 */
#define T4_BALL_LEVEL_TRIM_PULSE            (0)        /* = L，字段是 int32_t，脉冲数必须写整数；
                                                            反推算出 0.15（截断前）已跟视觉噪声同量级，
                                                            视为已收敛，不用再抠更小的小数 */
#define T4_BALL_FILTER_ALPHA                (0.10F)    /* 噪声仅 ±2px，无需压到 0.2 换来 165ms 滞后 */
#define T4_BALL_FILTER_BETA                 (0.10F)
/*
 * 2026-08 新曲柄摇杆机构方向实测（任务六 900 脉冲方向测试）：
 *   正脉冲 = 抬升摇杆；摇杆抬得越高，钢珠越往 X 变小方向移动。
 * 控制律 output = LEVEL_TRIM + SIGN×(Kx×error − Kv×velocity)，error = targetPx − 球位置。
 * error>0（目标在球右边）需要球向 +X 移动 → 必须降低摇杆 → output 须为负；
 * Kx>0 时 error>0 令 Kx×error>0，要让 output 为负必须 SIGN=-1。这是本次实测
 * 结论，不是猜测；如果实车验证方向反了，只改这一个数，不要改 emm42_robot.c
 * 的全局标定表。
 */
#define T4_BALL_OUTPUT_SIGN                 (-1.0F)

/* ---- B. 真正的调参旋钮 ---- */
/*
 * 旧丝杆机构曾有物理依据的系数 0.171（= (5/7)g / 400脉冲每mm / 250mm摆杆 *
 * 2.441px每mm）依赖丝杆导程换算，直驱摇臂后完全失效，需要新摇臂/连杆的实际
 * 几何尺寸才能重新推导，本次不臆测新系数。
 *
 * 2026-08 新机构第一次真实闭环测试，按 docs/CONTROL_ALGORITHM.md §10.2 方法论
 * 从纯 P 开始：Kv（抑制/阻尼项）先关掉、Kx 从保守小值起步，不做任何旋钮之外的
 * 限幅。调参顺序照 §10.3 现象表：
 *   球对误差反应很弱/很慢 → 加大 Kx；
 *   目标附近来回轻微振荡、幅度不变或缓慢衰减 → 停止加 Kx，固定住，转去从 0
 *     开始加 Kv 压振荡；
 *   球剧烈振荡/越振越猛/摆杆动作剧烈有异响 → Kx 过大，退回更小值重新找临界点；
 *   Kv 加到很大仍压不住振荡 → 大概率是 Kx 选大了，回去减小 Kx 而不是无限加 Kv。
 */
#define T4_BALL_KX_PULSE_PER_PX             (0.5F)     /* 新机构保守起点，按上面现象表逐步增大 */
#define T4_BALL_KV_PULSE_PER_PXPS           (0.52F)     /* 先关掉抑制，只调纯 P */
/*
 * 到位死区，同时就是静态精度上限：4px ≈ 1.6mm，取实测噪声 ±2px 的两倍裕度。
 * 误差进此范围【且球基本停住】才回真实水平点、停止驱动。
 */
#define T4_BALL_SETTLE_DEADBAND_PX          (4.0F)
/*
 * 判定钢珠"在运动"的速度门限：高于它按 sign(v) 补动摩擦，低于它过渡到按
 * sign(pd) 补静摩擦；同时是上面"到位保持"的速度条件。
 * 15px/s ≈ 6mm/s，明显高于速度估计噪声量级(±10px/s 的一半)。
 */
#define T4_BALL_FF_VEL_BLEND_PXPS           (15.0F)

/* ---- C. 执行器与显示 ---- */
/*
 * 摆杆每帧（20ms）行程能力 = POS_RPM * 3200/60 * 0.02 = POS_RPM * 1.067 脉冲。
 * 400RPM → 427 脉冲/帧，能容纳 Kv=18 带来的 180 脉冲/帧噪声并留出控制余量。
 * 这是上面 Kv 上限的来源，两者必须一起看。
 * ⚠️ 本机构尚未实测过 400RPM 的上限，上车先听有无异响/失步。
 *
 * 2026-08 机构变更为直驱摇臂后：同样 RPM 对应的角速度远大于丝杆机构
 * （旧机构靠丝杆导程+连杆大幅折算，直驱没有这层折算），预期需要调小，
 * 具体数值按现象判断——命令发出但摆杆没走到位/响应打折扣→调大，
 * 过冲很猛或有异响/失步→调小。
 */
#define T4_BALL_POS_RPM                     (400U)
/*
 * 2026-08 新机构实测：acc=0（瞬时到设定速度）在直驱摇臂上抖动很明显——BALLCTRL
 * 外环每约 17~25ms（视觉帧间隔）就下发一条全新绝对目标，acc=0 时每次都是速度
 * 阶跃，旧丝杆机构靠大幅减速比把这个阶跃吸收掉了，直驱摇臂没有这层缓冲，阶跃
 * 直接体现成机械抖动。现在改成可调旋钮，由使用者按下面方向自己试。
 *
 * 协议公式（手册§6.3.1，emm42_v5.h）：每升 1RPM 需要 (256−acc)×50us，
 * 【acc 越大爬升越快】，acc=0 是特例——不走曲线、直接瞬间给到目标速度（最快，
 * 也最抖），不是"最慢"，这一点容易搞反。例如 acc=180 时每 1RPM 耗时
 * (256−180)×50us=3.8ms，爬满 200RPM 约 760ms。
 *
 * ⚠️ 关键约束：外环约 17~25ms 就刷新一条全新目标，曲线基本不可能走完就被
 * 打断重新规划，实际能达到的转速 ≈ 帧间隔 / 每RPM耗时，远低于 POS_RPM。这是
 * 有意的效果（正是要靠"来不及冲到全速"来消除阶跃抖动），不是 bug；只是意味着
 * acc 越小（越接近但不等于 0），每帧实际能走的脉冲数越少、响应越绵软，acc 越
 * 大（越接近 255）越接近原来 acc=0 的阶跃手感。
 * 调参方向：还抖 → 减小 acc；感觉跟不上球、响应发软 → 增大 acc（增大后如果又
 * 开始抖，说明已经接近临界，退回上一档）。180 是题目二轮子已验证能跑的档位，
 * 仅作起点参考，不代表适合本机构，需要现场重新试。
 *
 * 2026-08 实测反馈"响应慢"：换算成 20ms 内实际能走的脉冲数就看出来了——
 * acc=120 时 20ms 只能爬到约 2.9RPM，实际走约 1.6 脉冲/帧，跟 Kx×error 算出来
 * 要求走多远完全无关，是 acc 卡死了行程上限，加 Kx 救不了。acc=230 时约
 * 15RPM、8.2 脉冲/帧，先提到这个值试；感觉还发软可以继续往 240~250 冲，
 * 代价是重新接近 acc=0 的阶跃手感，抖动可能回来，找中间平衡点。
 */
#define T4_BALL_POS_ACC                     (170U)
/*
 * 软件平滑：每帧实际下发的绝对目标相对上一帧最多变化多少脉冲，0=不限速。
 * 把 PD 输出的目标突变摊到连续多帧上，抑制目标跳变造成的机械冲击；小车行驶
 * 中球被持续扰动时尤其有用。调小 → 摆杆动作更柔和但跟踪更迟钝；调大 →
 * 越接近不限速的原始手感；0 = 完全关闭。
 *
 * ⚠️ 它【解决不了】"摆杆走一下停一下"的分段感：那是驱动器把每条绝对位置命令
 * 都规划成"加速→减速→精确停住"的点到点运动，帧间隔内走得完就会停一下；限速
 * 只会让每帧增量更小、更容易走完，反而加重。分段感要查静摩擦（FRICTION_FF）
 * 和 Kx，不要指望这个参数。
 */
#define T4_BALL_MAX_PULSE_STEP              (0U)
#define T4_BALL_HOLD_POSITION_PX            (6.0F)   /* 以下三项仅影响 OLED 的 B:HOLD 显示 */
#define T4_BALL_HOLD_VELOCITY_PXPS          (10.0F)
#define T4_BALL_HOLD_TIME_MS                (500U)

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
    T4_BALL_FRICTION_FF_PULSE,
    T4_BALL_FF_VEL_BLEND_PXPS,
    T4_BALL_POS_RPM,
    T4_BALL_POS_ACC,
    T4_BALL_MAX_PULSE_STEP,
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
