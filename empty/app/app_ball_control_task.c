#include "app_ball_control_task.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "alpha_beta_filter.h"
#include "app_config.h"
#include "app_vision_link.h"
#include "bsp_uart.h"
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

/*
 * α-β 滤波。2026-08-01 实测视觉静止噪声仅 ±2px（≈±0.8mm），属于很干净的信号，
 * 因此不需要重度滤波——α 压得越低相位滞后越大（α=0.2 在 50fps 下约 165ms），
 * 而球杆系统恰恰要靠提前预判来刹车，滞后是直接的失稳来源。
 */
#define BALL_CTRL_FILTER_ALPHA            (0.50F)
#define BALL_CTRL_FILTER_BETA             (0.10F)

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
 * 2026-08-01 改为【库仑摩擦前馈】，替换掉此前的"卡住检测 + 满幅夹紧"：
 *
 *   pd  = Kx*误差 − Kv*速度      （希望作用在钢珠上的净驱动力）
 *   cmd = pd + B*sign(v)         （补掉反抗运动的摩擦，使净力恰好等于 pd）
 *
 * 为什么需要前馈：命令幅值低于脱离阈值 B 时钢珠物理上纹丝不动，所以【整个
 * 线性 PD 区间原本都是无效的】——旧参数 Kv=4.8 在 60px/s 时只算出 288 脉冲，
 * 远低于 B=1810，Kv 从来没产生过任何制动作用，调它等于没调。
 *
 * 【关键：前馈跟 sign(速度)，不能跟 sign(pd)】摩擦永远反抗【运动】，所以补偿
 * 方向由速度决定。这样净力恰好等于 pd，而且命令只在钢珠真正掉头时才翻转
 * ——那一刻速度接近 0，有充足时间完成摆杆行程。
 *   ⚠️ 2026-08-01 中途一度写成 sign(pd)，实测直接发散（"越冲越大、刹不住"）：
 *   pd 在接近目标途中会反复过零，每过零一次命令就要跳 2*B=3620 脉冲；摆杆在
 *   POS_RPM=100 下每帧只能走 107 脉冲，这一跳要 0.68 秒，于是摆杆永远在执行
 *   0.68 秒前该给的倾角，构成正反馈。切回 sign(v) 后命令幅值也大幅下降：
 *   球以 +v 运动、pd=-840 时命令只是 1810-840=970，摆杆略微回一点即可，
 *   不必满幅反倾。
 *
 * 球接近静止时没有"运动方向"可言，此时线性过渡到按 sign(pd) 补偿，用来打破
 * 静摩擦（过渡带宽度 = ffVelBlendPxps）。
 *
 * 被替换掉的旧方案（卡住检测 + 夹紧到阈值）的根本缺陷：夹紧条件是"速度低于
 * 门限"，球一旦动起来（下一帧，约 20ms 后）夹紧立即撤销、命令掉回无效的小值，
 * 于是球刚脱离静摩擦就失去驱动、走几个像素又粘住，表现为"在目标附近反复挣扎
 * 却过不去"（2026-08-01 实测：停在 300 到不了 320）。
 *
 * 【选增益的物理依据】把命令换算成钢珠加速度：
 *   a[px/s²] = (5/7)*g * (脉冲/400/250) * 2.441px/mm ≈ 0.171 * 脉冲
 * 代入 ẍ = 0.171*pd 得二阶系统 ω_n = sqrt(0.171*Kx)、ζ = 0.171*Kv/(2*ω_n)。
 * 另有两条硬约束：
 *   摆杆每帧行程 = POS_RPM * 3200/60 * 0.02 = POS_RPM * 1.067 脉冲；
 *   Kv 放大速度估计噪声 ≈ Kv * 10px/s 脉冲/帧（β=0.1、位置噪声±2px 实测）。
 * 后者必须明显小于前者，否则摆杆全部行程都用来追噪声（Kv=36 + POS_RPM=100
 * 时噪声 360 > 能力 107，实测直接失控）。
 */
/*
 * Kx=12 → ω_n = sqrt(0.171*12) = 1.43 rad/s；
 * Kv=18 → ζ = 0.171*18/(2*1.43) = 1.08（略过阻尼），噪声占用 18*10=180 脉冲/帧，
 * 在 POS_RPM=400 的 427 脉冲/帧能力之内。
 */
#define BALL_CTRL_OUTPUT_SIGN             (1.0F)
#define BALL_CTRL_KX_PULSE_PER_PX         (12.0F)
#define BALL_CTRL_KV_PULSE_PER_PXPS       (18.0F)

/* 真实水平点：2026-08-01 题目六标定斜坡实测 L = -70 脉冲（≈0.04°，机械基本是正的）。 */
#define BALL_CTRL_LEVEL_TRIM_PULSE        (-70)

/*
 * 到位保持区：误差进入该范围即回真实水平点、不再驱动，所以它同时就是静态
 * 精度上限。取 4px（≈1.6mm），依据是实测视觉噪声仅 ±2px，留一倍裕度。
 */
#define BALL_CTRL_SETTLE_DEADBAND_PX      (4.0F)

/*
 * 摩擦前馈量 = 钢珠脱离阈值，2026-08-01 题目六标定斜坡实测 B = 1810 脉冲
 * （4.53mm 升程 / 1.04° 倾角，换更光滑导轨后比旧值 3600~4000 低了一半多）。
 * 换导轨、换钢珠或改摆杆几何后必须用题目六重测，不要沿用。
 */
#define BALL_CTRL_FRICTION_FF_PULSE       (1810.0F)

/* 判定钢珠"在运动"的速度门限：15px/s ≈ 6mm/s，明显高于速度估计噪声(±10px/s 量级)。 */
#define BALL_CTRL_FF_VEL_BLEND_PXPS       (15.0F)

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

/*
 * 菜单默认闭环不启用软件平滑（0=不限速），保持历史行为不变；各题目按需在
 * 自己的 profile 里给非 0 值。含义与注意事项见 app_ball_control_task.h。
 */
#define BALL_CTRL_MAX_PULSE_STEP          (0U)

/* 中心保持判定，仅用于 OLED 显示 HOLDING/RUNNING，不影响任何控制动作。 */
#define BALL_CTRL_HOLD_POSITION_PX        (5.0F)
#define BALL_CTRL_HOLD_VELOCITY_PXPS      (10.0F)
#define BALL_CTRL_HOLD_TIME_MS            (500U)

/* 安全边界：球越过这个像素范围视为快滚出摆杆，命令回水平并锁定报警。 */
#define BALL_CTRL_SAFE_X_MIN_PX           (20)
#define BALL_CTRL_SAFE_X_MAX_PX           (620)

/*
 * 2026-08 临时调试开关：把每次实际算出命令的关键量打到 UART0（TX 空闲，只有
 * 视觉占 RX，互不冲突，用法与 task6 标定斜坡的日志一致）。用来分层排查"链路通、
 * 电机通，但球还是稳不住"这类问题——能直接看到目标/测量/滤波位置/速度/命令
 * 脉冲/状态，不用再靠猜。排查完建议改回 0，长期开着会占一部分 CPU 和串口带宽。
 * DIVIDER 用来降频：每隔这么多次"有新有效样本"才打印一行，避免刷屏
 * （视觉约 15~60fps，DIVIDER=5 时输出约 3~12 行/秒，人眼能跟得上）。
 */
#define BALL_CTRL_DEBUG_LOG_ENABLE        (1U)
#define BALL_CTRL_DEBUG_LOG_DIVIDER       (5U)

/* 模块内置默认参数（AppBallControl_RequestTarget 用）；各题目通过独立 profile 覆盖。 */
static const AppBallControlProfile_t s_menuProfile = {
    BALL_CTRL_FILTER_ALPHA,
    BALL_CTRL_FILTER_BETA,
    BALL_CTRL_OUTPUT_SIGN,
    BALL_CTRL_KX_PULSE_PER_PX,
    BALL_CTRL_KV_PULSE_PER_PXPS,
    BALL_CTRL_LEVEL_TRIM_PULSE,
    BALL_CTRL_SETTLE_DEADBAND_PX,
    BALL_CTRL_FRICTION_FF_PULSE,
    BALL_CTRL_FF_VEL_BLEND_PXPS,
    BALL_CTRL_POS_RPM,
    BALL_CTRL_POS_ACC,
    BALL_CTRL_MAX_PULSE_STEP,
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

#if (BALL_CTRL_DEBUG_LOG_ENABLE != 0U)
/*
 * 无锁输出带符号整数（调用方已经用 BspUart0_Lock 包住整行拼接）；浮点量先四舍
 * 五入成整数再打，串口带宽有限，小数位对排查没有额外价值。
 */
static void BallControl_LogSignedInt(int32_t value)
{
    if (value < 0) {
        BspUart0_SendByte((uint8_t)'-');
        BspUart0_SendUint((uint32_t)(-value));
    } else {
        BspUart0_SendUint((uint32_t)value);
    }
}

static void BallControl_LogSignedFloat(float value)
{
    BallControl_LogSignedInt(BallControl_RoundToInt32(value));
}

/*
 * 打一行完整诊断：新样本序号、目标/测量/滤波位置、滤波速度、误差、最终下发的
 * 绝对脉冲、当前状态。放在“确实算出了一条新命令”的地方调用，配合
 * BALL_CTRL_DEBUG_LOG_DIVIDER 降频，不在每一帧都打。
 */
static void BallControl_LogSample(uint32_t seq, int16_t targetPx, int16_t measuredPx,
                                  const AlphaBetaFilter_t *filter, float rawError,
                                  int32_t commandPulse, const char *stateTag)
{
    BspUart0_Lock();
    BspUart0_SendString("BC seq=");
    BspUart0_SendUint(seq);
    BspUart0_SendString(" tgt=");
    BspUart0_SendUint((uint32_t)targetPx);
    BspUart0_SendString(" meas=");
    BspUart0_SendUint((uint32_t)measuredPx);
    BspUart0_SendString(" filt=");
    BallControl_LogSignedFloat(filter->position);
    BspUart0_SendString(" v=");
    BallControl_LogSignedFloat(filter->velocity);
    BspUart0_SendString(" err=");
    BallControl_LogSignedFloat(rawError);
    BspUart0_SendString(" cmd=");
    BallControl_LogSignedInt(commandPulse);
    BspUart0_SendString(" st=");
    BspUart0_SendString(stateTag);
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}
#endif /* BALL_CTRL_DEBUG_LOG_ENABLE */

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
    uint32_t seenSampleSeq = 0U;
    uint32_t consecutiveNaCount = 0U;
    uint32_t recoveryValidCount = 0U;
    int16_t targetPx = APP_BALL_CONTROL_CENTER_X_PX;
    int16_t measuredPx = 0;
    int32_t commandPulse = 0;
    bool holding = false;
    /*
     * 软件平滑状态：commandPulse 是本帧实际下发给驱动器的目标（已限速），
     * smoothPrimed 表示它是否已经有过一个有效起点。首帧不限速直接跳到 PD 输出，
     * 否则会从一个与电机真实位置无关的起点（如上次退出时的位置）慢慢爬。
     */
    bool smoothPrimed = false;
#if (BALL_CTRL_DEBUG_LOG_ENABLE != 0U)
    uint32_t debugLogCounter = 0U;
#endif

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
                    smoothPrimed = false;
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
                        state = BALL_CTRL_INTERNAL_LOST;
#if (BALL_CTRL_DEBUG_LOG_ENABLE != 0U)
                        /* LOST 转换不受降频限制：这种边界事件本来就少，全打出来更有用。 */
                        BspUart0_Lock();
                        BspUart0_SendString("BC seq=");
                        BspUart0_SendUint(seenSampleSeq);
                        BspUart0_SendString(" st=LOST(consecutive NA)\r\n");
                        BspUart0_Unlock();
#endif
                    }
                } else if (vision.valid) {
                    float dtSec;
                    float rawError;
                    float pulseDelta;
                    float output;

                    measuredPx = vision.pixel;
                    if ((measuredPx <= BALL_CTRL_SAFE_X_MIN_PX) ||
                        (measuredPx >= BALL_CTRL_SAFE_X_MAX_PX)) {
                        /*
                         * 球快滚出摆杆：命令回水平，标记故障并锁定，等 K4 处理。
                         * 安全动作【不走软件平滑限速】，必须立即到位。
                         */
                        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT,
                                               activeProfile.levelTrimPulse,
                                               activeProfile.positionRpm,
                                               activeProfile.positionAcc);
                        commandPulse = activeProfile.levelTrimPulse;
                        smoothPrimed = true;
                        state = BALL_CTRL_INTERNAL_FAULT_EDGE;
#if (BALL_CTRL_DEBUG_LOG_ENABLE != 0U)
                        BspUart0_Lock();
                        BspUart0_SendString("BC seq=");
                        BspUart0_SendUint(seenSampleSeq);
                        BspUart0_SendString(" meas=");
                        BspUart0_SendUint((uint32_t)measuredPx);
                        BspUart0_SendString(" st=FAULT_EDGE\r\n");
                        BspUart0_Unlock();
#endif
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

                    if ((BallControl_Abs(rawError) <= activeProfile.settleDeadbandPx) &&
                        (BallControl_Abs(filter.velocity) <= activeProfile.ffVelBlendPxps)) {
                        /*
                         * 到位【且】基本停住才回真实水平点保持，静态精度由死区决定。
                         * 必须同时判速度：只看误差的话，球高速穿过目标的那一刻会被
                         * 当成"到位"而清零输出，白白放弃这一段最该刹车的窗口。
                         */
                        pulseDelta = 0.0F;
                    } else {
                        /* 纯 PD，无 I 项：这是希望作用在钢珠上的净驱动力。 */
                        float pd = (activeProfile.kxPulsePerPx * rawError) -
                                   (activeProfile.kvPulsePerPxps * filter.velocity);
                        float ffDir;

                        /*
                         * 库仑摩擦前馈：摩擦永远反抗【运动】，所以补偿方向由速度定，
                         * 补上之后净驱动力恰好等于 pd。球接近静止（|v| 低于过渡带）
                         * 时没有运动方向可言，按比例过渡到沿 pd 方向补，用于打破静
                         * 摩擦。绝不能整体改用 sign(pd)——pd 在接近途中反复过零，会
                         * 让命令每次都跳 2*B，超出摆杆行程能力而发散（见文件头）。
                         */
                        ffDir = BallControl_Clamp(filter.velocity / activeProfile.ffVelBlendPxps,
                                                  -1.0F, 1.0F);
                        if (BallControl_Abs(ffDir) < 1.0F) {
                            ffDir += (1.0F - BallControl_Abs(ffDir)) *
                                     ((pd >= 0.0F) ? 1.0F : -1.0F);
                            ffDir = BallControl_Clamp(ffDir, -1.0F, 1.0F);
                        }
                        pulseDelta = (ffDir * activeProfile.frictionFfPulse) + pd;
                    }

                    output = (float)activeProfile.levelTrimPulse +
                             (activeProfile.outputSign * pulseDelta);

                    /*
                     * 软件平滑：把 PD 算出的理想目标按每帧最大步进逼近，使驱动器
                     * 收到的是渐进推进的目标而不是阶跃。maxPulseStepPerFrame==0
                     * 时整段退化为直接下发，与历史行为完全一致。
                     * 首帧（smoothPrimed==false）不限速：此时 commandPulse 还是上次
                     * 退出时的残留值，与电机当前真实位置无关，从它开始爬没有意义。
                     */
                    {
                        int32_t idealPulse = BallControl_RoundToInt32(output);

                        if (!smoothPrimed || (activeProfile.maxPulseStepPerFrame == 0U)) {
                            commandPulse = idealPulse;
                            smoothPrimed = true;
                        } else {
                            int32_t maxStep = (int32_t)activeProfile.maxPulseStepPerFrame;
                            int32_t delta = idealPulse - commandPulse;

                            if (delta > maxStep) {
                                delta = maxStep;
                            } else if (delta < -maxStep) {
                                delta = -maxStep;
                            }
                            commandPulse += delta;
                        }
                    }

                    Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, commandPulse,
                                           activeProfile.positionRpm,
                                           activeProfile.positionAcc);
                    state = BALL_CTRL_INTERNAL_ACTIVE;

#if (BALL_CTRL_DEBUG_LOG_ENABLE != 0U)
                    /* 降频打印，避免视觉 15~60fps 全量打印刷屏/占满带宽。 */
                    debugLogCounter++;
                    if (debugLogCounter >= BALL_CTRL_DEBUG_LOG_DIVIDER) {
                        debugLogCounter = 0U;
                        BallControl_LogSample(seenSampleSeq, targetPx, measuredPx,
                                              &filter, rawError, commandPulse,
                                              (pulseDelta == 0.0F) ? "HOLD" : "RUN");
                    }
#endif

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
            /* 失能后电机位置不再受命令约束，下次启动首帧必须重新不限速定起点。 */
            smoothPrimed = false;
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
