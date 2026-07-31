#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ball_control_task.h"
#include "app_vision_link.h"
#include "bsp_uart.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 6 题：Emm42 ID1（摆杆升降丝杆）位置模式 API 冒烟测试
 *
 * 目的：把摆杆的控制量从"角速度"降回"角度"。速度模式下 PID 输出的 RPM 对
 * 摆杆角度是一次积分、球位置对摆杆角度又是二次积分，整条链路三阶，纯 PID
 * 很难镇定；改用位置模式后每帧直接给"摆杆该停在哪个脉冲位置"，系统降为二阶。
 * 本题只负责验证底层 API，不含任何球位置闭环。
 *
 * 测试内容由下方开关组合决定：
 *   1. 相对位置模式单程：+T6_TEST_REVS 圈（当前默认，用来量丝杆实际行进多少 mm）；
 *   2. T6_RETURN_ENABLE=1 时追加 -T6_TEST_REVS 圈回程，净位移应回原处；
 *   3. T6_ABS_TEST_ENABLE=1 时再追加绝对位置验证：+MOVE → 0 → -MOVE → 0，
 *      两次回 0 应停在同一物理点；
 *   4. T6_RAMP_TEST_ENABLE=1 时【完全替代】上面三项，改为标定斜坡：极缓慢地把
 *      绝对目标往正、负两个方向各加一次，用视觉 X 自动检测钢珠第一次开始滚动的
 *      瞬间，实测出「脱离阈值 B」和「真实水平点 L」两个数（详见下方宏定义处）。
 *      这与前三项测的"丝杆本身能不能被位置模式驱动"是两件事：机构的大幅度运动
 *      早已验证过（10 圈=80mm），这里测的是钢珠自身开始响应的临界倾角。
 *      2026-08-01 换更光滑导轨后，旧阈值作废，必须用本工具重测。
 *
 * 【所有位置都锚在"按 K3 进入本题那一刻的电机位置"上】：相对模式命令本就以当前
 * 实际位置为起点；绝对模式的原点也是进入本题时才建立的，所以不存在跨越上电或
 * 跨越一次进出的持久绝对坐标。⚠️ 反复退出再进入，每次都会以当时的位置重新置零，
 * 单程模式下位移会一路累积，量完记得手摇回行程中部再测下一次。
 *
 * 位置原点：进入本题时（T6_ZERO_ON_ENTER）发 0x0A 0x6D 把当前机械位置定义为 0。
 * 摆杆已加装 PA24 归零限位开关，开机由 app_lift_homing.c 的 AppLiftHoming_RunAtBoot()
 * 自动归零，不再需要人工把杆摆到目视水平再按 K3 进本题；驱动器断电不保留多圈位置
 * 计数，每次上电都要重做一遍。
 *
 * 手册要点（Emm_V5.0 Rev1.3）：
 *   - 位置模式帧 0xFD，相对/绝对由帧内标志区分，绝对模式的"方向"字节表示目标
 *     绝对位置的符号（§6.3.1、§6.5 同步控制示例）；
 *   - 脉冲单位取决于驱动器细分，出厂 16 细分 = 3200 脉冲/圈；
 *   - 位置到达窗口默认 0.1°（约 0.9 脉冲），死区不来自驱动器而来自机械；
 *   - 堵转保护触发后位置命令会返回 E2 且电机纹丝不动，必须先发 0x0E 0x52 解除。
 *     丝杆低速顶死正好满足"转速<40RPM + 电流>2400mA + 持续>4000ms"三个条件，
 *     所以进入本题时先无条件解一次（T6_CLEAR_CLOG_ON_ENTER）。
 *
 * 本题全程只操作 EMM42_ROBOT_LIFT（ID1），不触碰 ID2/ID3 两个轮子。
 * ================================================================== */

/* ---- 机械/协议换算（改驱动器细分要同步改这里） ---- */
#define T6_PULSES_PER_REV        (3200)   /* 16 细分 = 3200 脉冲/圈 */
#define T6_TEST_REVS             (10)     /* 圈数模式用（丝杆行程标定），与下面直接脉冲模式二选一 */
/*
 * 2026-08 机构变更为直驱摇臂后的方向/小角度测试：直接指定脉冲数，不再从
 * T6_TEST_REVS 按圈数换算——900 脉冲 = 900/3200*360 ≈ 101°电机转角，注意这已经
 * 略超过摇臂预期的 ~90°可用范围，第一次测试时人在旁盯着，看到连杆顶死/异响
 * 立刻断电，不要等它自己走完。想恢复旧的"按圈数测丝杆行程"用法，换回：
 *   ((int32_t)T6_TEST_REVS * (int32_t)T6_PULSES_PER_REV)
 */
#define T6_MOVE_PULSES           (900)

/* ---- 运动参数（纯可调，不叠加任何软件限幅或斜坡） ---- */
#define T6_POS_RPM               (30U)    /* 位置模式运行速度上限（RPM） */
#define T6_POS_ACC               (0U)     /* 加速度档位，0=不用曲线直接按设定速度跑 */
#define T6_FIRST_DIR             (+1)     /* 首段方向：+1=角色层正方向（连杆向下） */
#define T6_DWELL_MS              (1200U)  /* 每段【到位之后】额外停留的观察时间 */
#define T6_LOOP_FOREVER          (0U)     /* 1=往返循环；0=跑完一轮停住等 K4 */

/* ---- 流程开关 ---- */
#define T6_ZERO_ON_ENTER         (1U)     /* 进本题时把当前位置定义为原点 */
#define T6_CLEAR_CLOG_ON_ENTER   (1U)     /* 进本题时先解一次堵转保护 */
#define T6_RETURN_ENABLE         (0U)     /* 1=单程后再反向走回来；0=只走单程便于量行程 */
#define T6_ABS_TEST_ENABLE       (0U)     /* 相对段之后追加绝对位置模式验证 */

/*
 * ---- 标定斜坡：实测「钢珠脱离阈值」+「真实水平点」（与上面三项互斥） ----
 *
 * 置 1 后【完全替代】相对/绝对测试。这是给【当前这套机械】现场量数用的工具，
 * 目的就是不再沿用任何旧导轨留下的经验值（2026-08-01 换了更光滑的导轨后，旧的
 * 3200~4000 脉冲阈值已经作废，继续用它会导致每次静摩擦补偿都是过量猛踹）。
 *
 * 原理：位置模式下，命令幅值小于「脱离阈值」时钢珠纹丝不动——丝杆动了，球不动。
 * 所以从命令 0 开始极缓慢地把绝对目标往一个方向加，同时盯视觉 X，球第一次真的
 * 动起来那一刻的命令值，就是这个方向的脱离阈值。
 *
 * 流程（全自动，只需看 OLED 读数）：
 *   1. 抓一帧有效 X 作为起点；
 *   2. 每 T6_RAMP_INTERVAL_MS 把绝对目标加 T6_RAMP_STEP_PULSES，直到视觉 X 相对
 *      起点偏移超过 T6_RAMP_MOVE_THRESHOLD_PX（远大于实测 ±2px 噪声）→ 记下命令值；
 *   3. 命令回 0，留 T6_RAMP_RECENTER_MS 让你把球放回中间；
 *   4. 换负方向重测一次；
 *   5. 出结果。
 *
 * 两个方向各测一次的意义——由 P(正向阈值) 和 N(负向阈值) 可直接算出：
 *   脱离阈值 B = (P + N) / 2   ← 摩擦补偿量的实测依据
 *   真实水平 L = (P - N) / 2   ← LEVEL_TRIM_PULSE 的实测依据
 * 若机械完全对称则 P≈N、L≈0；P 和 N 差得越多说明"命令 0"离真正水平越远。
 * 结果同时显示在 OLED 并经 UART0 打印明细（TX 空闲，视觉只占 RX，不冲突）。
 *
 * ⚠️ 任一方向到 T6_RAMP_MAX_PULSES 仍不动，该方向记 0（表示未测到），不会把上限
 * 当成结果——避免又造出一个假的"实测值"。
 */
#define T6_RAMP_TEST_ENABLE        (0U)     /* 2026-08 临时关闭：先做直驱摇臂方向/角度测试，用不到钢珠标定斜坡 */
#define T6_RAMP_STEP_PULSES        (20)      /* 每次增量：越小越精细但越慢（20 脉冲=0.05mm 升程） */
#define T6_RAMP_INTERVAL_MS        (90U)     /* 增量间隔，取 UI 拍 30ms 的整数倍 */
#define T6_RAMP_RPM                (60U)     /* 单个小增量的执行转速（20 脉冲约 6ms 走完，远小于间隔） */
#define T6_RAMP_MAX_PULSES         (8000)    /* 安全上限，覆盖旧导轨阈值两倍还多 */
#define T6_RAMP_MOVE_THRESHOLD_PX  (8)       /* 判定"球动了"的位移；实测静止噪声仅 ±2px */
#define T6_RAMP_RECENTER_MS        (12000U)  /* 两方向之间留给你把球放回中间的时间 */

/*
 * 每个运动段必须等到电机真正走完才能发下一帧，否则新的位置命令会【覆盖】上一条
 * 并重新规划，表现就是"命令发了但几乎没走到位"。
 *
 * acc=0 没有加减速曲线，理论运动时间 = 圈数 / 转速（分钟）。实际带载略慢，
 * 再乘一个余量系数，最后叠加 T6_DWELL_MS 的观察时间。
 * ⚠️ 转速越低这个等待越长：10 圈 @30RPM 理论就要 20 秒，觉得慢就调高 T6_POS_RPM。
 */
#if (T6_POS_RPM == 0U)
#error "T6_POS_RPM 不能为 0，位置模式速度为 0 时驱动器不执行"
#endif
#define T6_MOVE_MARGIN_PCT       (30U)
/* 由 T6_MOVE_PULSES 直接换算理论运动时间，不再依赖 T6_TEST_REVS（见上方说明）。 */
#define T6_MOVE_TIME_MS          ((60000UL * (uint32_t)(((T6_MOVE_PULSES) < 0) ? \
                                  -(T6_MOVE_PULSES) : (T6_MOVE_PULSES))) / \
                                  ((uint32_t)(T6_PULSES_PER_REV) * (uint32_t)(T6_POS_RPM)))
#define T6_SEGMENT_WAIT_MS       (((T6_MOVE_TIME_MS) * (100U + T6_MOVE_MARGIN_PCT) / 100U) + \
                                  (uint32_t)(T6_DWELL_MS))

/* ---- 沿用 task2/task3 已实测的 Emm42 时序，不要往下调 ---- */
#define T6_RESET_SETTLE_MS       (60U)
#define T6_ENABLE_SETTLE_MS      (180U)   /* 90ms 实测约一半概率使能不生效 */
#define T6_EMM_CMD_GAP_MS        (6U)

/* 本题在题目表中的下标（用于运行结束时通知视觉端）。 */
#define T6_TASK_INDEX            (5U)

typedef enum {
    T6_STATE_WAIT_BALL_RELEASE = 0, /* 等后台钢球闭环释放 ID1 */
    T6_STATE_RESET_DISABLE,         /* 失能 ID1 清残留状态 */
    T6_STATE_RESET_WAIT,            /* 等失能生效 */
    T6_STATE_CLEAR_CLOG,            /* 解除可能残留的堵转保护 */
    T6_STATE_ENABLE,                /* 使能 ID1 */
    T6_STATE_ENABLE_WAIT,           /* 等驱动器完成使能 */
    T6_STATE_ZERO,                  /* 把当前位置定义为原点 */
    T6_STATE_REL_A,                 /* 相对模式：首段 */
    T6_STATE_REL_B,                 /* 相对模式：反向回程 */
    T6_STATE_ABS_P,                 /* 绝对模式：+MOVE */
    T6_STATE_ABS_Z1,                /* 绝对模式：回 0 */
    T6_STATE_ABS_N,                 /* 绝对模式：-MOVE */
    T6_STATE_ABS_Z2,                /* 绝对模式：再回 0 */
    T6_STATE_RAMP_START,            /* 标定斜坡：抓取起点 X */
    T6_STATE_RAMP_UP,               /* 标定斜坡：缓慢递增命令，等球开始动 */
    T6_STATE_RAMP_BACK,             /* 标定斜坡：命令回水平，换方向或出结果 */
    T6_STATE_RAMP_REPORT,           /* 标定斜坡：两个方向都测完，发布结果 */
    T6_STATE_DWELL,                 /* 通用停留，到时切到 s_dwellNext */
    T6_STATE_CYCLE_END,             /* 一轮结束：循环或收尾 */
    T6_STATE_STOP,                  /* 已急停，下一拍失能 */
    T6_STATE_DONE                   /* 测试结束，保持静止等 K4 */
} Task6State_t;

static Task6State_t s_state;
static Task6State_t s_dwellNext;
static uint32_t     s_dwellMs;
static TickType_t   s_stateStartTick;

/* 任务层记账的绝对目标脉冲，仅用于 OLED 显示，不参与控制。 */
static int32_t      s_absTarget;

/* ---- 标定斜坡的运行状态与结果 ---- */
static int32_t       s_rampDir;          /* 当前测的方向：+1 / -1 */
static int32_t       s_rampCmd;          /* 当前已下发的绝对命令 */
static int16_t       s_rampStartX;       /* 本方向斜坡开始时的钢珠 X */
static int32_t       s_rampPosBreak;     /* 正向脱离阈值（幅值），0=未测到 */
static int32_t       s_rampNegBreak;     /* 负向脱离阈值（幅值），0=未测到 */
static bool          s_rampReportReady;  /* 结果已就绪，OLED 持续显示 */

/* OLED 一行状态文本：阶段名 + 关联脉冲数。 */
static const char  *s_uiPhase = "IDLE";
static int32_t      s_uiValue;
static bool         s_uiHasValue;
static char         s_uiStatusBuf[24];

static bool Task6_Elapsed(TickType_t now, TickType_t then, uint32_t timeoutMs)
{
    return (TickType_t)(now - then) >= pdMS_TO_TICKS(timeoutMs);
}

/* 记录当前阶段供 OLED 显示；value 为该阶段关联的脉冲数（增量或绝对目标）。 */
static void Task6_SetUi(const char *phase, int32_t value, bool hasValue)
{
    s_uiPhase    = phase;
    s_uiValue    = value;
    s_uiHasValue = hasValue;
}

/*
 * 下发一帧后统一进入等待状态，保证每个 30ms 轮询拍最多发一帧。
 * waitMs 对运动段要用 T6_SEGMENT_WAIT_MS（必须覆盖整段运动时间，否则下一帧会
 * 覆盖尚未走完的位置命令）；对清零这种瞬时动作用短的 T6_DWELL_MS 即可。
 */
static void Task6_EnterDwell(TickType_t now, Task6State_t next, uint32_t waitMs)
{
    s_stateStartTick = now;
    s_dwellNext      = next;
    s_dwellMs        = waitMs;
    s_state          = T6_STATE_DWELL;
}

#if (T6_RAMP_TEST_ENABLE != 0U)
/* 取绝对值，供斜坡判限和结果换算使用。 */
static int32_t Task6_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* 把带符号整数追加到 dst[idx] 起，返回新的写入下标；limit 为可写上界。 */
static uint32_t Task6_AppendSigned(char *dst, uint32_t idx, int32_t value, uint32_t limit)
{
    uint32_t magnitude;
    char     digits[12];
    uint32_t n = 0U;

    if (idx >= limit) {
        return idx;
    }
    if (value < 0) {
        dst[idx++] = '-';
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }
    if (magnitude == 0U) {
        if (idx < limit) {
            dst[idx++] = '0';
        }
        return idx;
    }
    while (magnitude > 0U) {
        digits[n++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    }
    while ((n > 0U) && (idx < limit)) {
        dst[idx++] = digits[--n];
    }
    return idx;
}

/* 标定数据经 UART0 打印明细（TX 空闲，视觉只占 RX）；整行加锁保证不被打断。 */
static void Task6_RampLog(const char *label, int32_t value)
{
    BspUart0_Lock();
    BspUart0_SendString(label);
    if (value < 0) {
        BspUart0_SendByte((uint8_t)'-');
        BspUart0_SendUint((uint32_t)(-value));
    } else {
        BspUart0_SendUint((uint32_t)value);
    }
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}
#endif /* T6_RAMP_TEST_ENABLE */

/* 相对段首段之后的下一站：按开关决定是否走回程。 */
static Task6State_t Task6_AfterFirstMove(void)
{
#if (T6_RETURN_ENABLE != 0U)
    return T6_STATE_REL_B;
#elif (T6_ABS_TEST_ENABLE != 0U)
    return T6_STATE_ABS_P;
#else
    return T6_STATE_CYCLE_END;
#endif
}

/* 相对段全部结束后的下一站：开了绝对测试就继续，否则进入一轮收尾。 */
static Task6State_t Task6_AfterRelative(void)
{
#if (T6_ABS_TEST_ENABLE != 0U)
    return T6_STATE_ABS_P;
#else
    return T6_STATE_CYCLE_END;
#endif
}

/*
 * 清零完成后（或跳过清零时）第一个要进入的运动状态：阶梯测试模式下走
 * 阶梯步长测试，否则走原有的相对/绝对测试。
 */
static Task6State_t Task6_FirstMoveState(void)
{
#if (T6_RAMP_TEST_ENABLE != 0U)
    /* 标定斜坡固定从正方向开始，测完自动换负方向。 */
    s_rampDir         = +1;
    s_rampCmd         = 0;
    s_rampPosBreak    = 0;
    s_rampNegBreak    = 0;
    s_rampReportReady = false;
    return T6_STATE_RAMP_START;
#else
    return T6_STATE_REL_A;
#endif
}

void Task6_OnEnter(void)
{
    /*
     * ID1 同时被后台钢球闭环使用，两者不能并存。先请求闭环安全退出，
     * 再由 OnLoop 等它真正释放（与 task3.c 同一套做法）。
     */
    AppBallControl_RequestStop();

    s_state          = T6_STATE_WAIT_BALL_RELEASE;
    s_dwellNext      = T6_STATE_DONE;
    s_dwellMs        = T6_DWELL_MS;
    s_stateStartTick = 0U;
    s_absTarget      = 0;
#if (T6_RAMP_TEST_ENABLE != 0U)
    s_rampDir         = +1;
    s_rampCmd         = 0;
    s_rampStartX      = 0;
    s_rampPosBreak    = 0;
    s_rampNegBreak    = 0;
    s_rampReportReady = false;
#endif
    Task6_SetUi("WAIT BALL", 0, false);
}

void Task6_OnLoop(void)
{
    TickType_t now = xTaskGetTickCount();

    switch (s_state) {
    case T6_STATE_WAIT_BALL_RELEASE:
        if (!AppBallControl_IsActive()) {
            s_state = T6_STATE_RESET_DISABLE;
        }
        break;

    case T6_STATE_RESET_DISABLE:
        /* 仅失能 ID1，绝不触碰题目二/四/五使用的 ID2/ID3。 */
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        Task6_SetUi("RESET", 0, false);
        s_stateStartTick = now;
        s_state = T6_STATE_RESET_WAIT;
        break;

    case T6_STATE_RESET_WAIT:
        if (Task6_Elapsed(now, s_stateStartTick, T6_RESET_SETTLE_MS)) {
#if (T6_CLEAR_CLOG_ON_ENTER != 0U)
            s_state = T6_STATE_CLEAR_CLOG;
#else
            s_state = T6_STATE_ENABLE;
#endif
        }
        break;

    case T6_STATE_CLEAR_CLOG:
        /* 丝杆上次顶死可能留下堵转保护，不清掉后面所有位置命令都会被拒。 */
        Emm42Robot_ClearClogProtection(EMM42_ROBOT_LIFT);
        Task6_SetUi("CLOG CLR", 0, false);
        s_state = T6_STATE_ENABLE;
        break;

    case T6_STATE_ENABLE:
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
        Task6_SetUi("ENABLE", 0, false);
        s_stateStartTick = now;
        s_state = T6_STATE_ENABLE_WAIT;
        break;

    case T6_STATE_ENABLE_WAIT:
        if (Task6_Elapsed(now, s_stateStartTick, T6_ENABLE_SETTLE_MS)) {
#if (T6_ZERO_ON_ENTER != 0U)
            s_state = T6_STATE_ZERO;
#else
            s_state = Task6_FirstMoveState();
#endif
        }
        break;

    case T6_STATE_ZERO:
        /* 把人工摆平后的当前位置定义为绝对位置 0（levelPulse）。 */
        Emm42Robot_ResetPosToZero(EMM42_ROBOT_LIFT);
        s_absTarget = 0;
        Task6_SetUi("ZERO", 0, true);
        /* 清零是瞬时动作，不需要等运动时间。 */
        Task6_EnterDwell(now, Task6_FirstMoveState(), T6_DWELL_MS);
        break;

    case T6_STATE_REL_A:
        Emm42Robot_MoveRelative(EMM42_ROBOT_LIFT,
                                (int32_t)T6_FIRST_DIR * T6_MOVE_PULSES,
                                T6_POS_RPM, T6_POS_ACC);
        s_absTarget += (int32_t)T6_FIRST_DIR * T6_MOVE_PULSES;
        Task6_SetUi("REL", (int32_t)T6_FIRST_DIR * T6_MOVE_PULSES, true);
        Task6_EnterDwell(now, Task6_AfterFirstMove(), T6_SEGMENT_WAIT_MS);
        break;

    case T6_STATE_REL_B:
        Emm42Robot_MoveRelative(EMM42_ROBOT_LIFT,
                                -(int32_t)T6_FIRST_DIR * T6_MOVE_PULSES,
                                T6_POS_RPM, T6_POS_ACC);
        s_absTarget -= (int32_t)T6_FIRST_DIR * T6_MOVE_PULSES;
        Task6_SetUi("REL", -(int32_t)T6_FIRST_DIR * T6_MOVE_PULSES, true);
        Task6_EnterDwell(now, Task6_AfterRelative(), T6_SEGMENT_WAIT_MS);
        break;

    case T6_STATE_ABS_P:
        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, T6_MOVE_PULSES,
                                T6_POS_RPM, T6_POS_ACC);
        s_absTarget = T6_MOVE_PULSES;
        Task6_SetUi("ABS", s_absTarget, true);
        Task6_EnterDwell(now, T6_STATE_ABS_Z1, T6_SEGMENT_WAIT_MS);
        break;

    case T6_STATE_ABS_Z1:
        /* 绝对目标 0 必须真的回到清零时的物理位置，这是后续外环的前提。 */
        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, 0, T6_POS_RPM, T6_POS_ACC);
        s_absTarget = 0;
        Task6_SetUi("ABS", s_absTarget, true);
        Task6_EnterDwell(now, T6_STATE_ABS_N, T6_SEGMENT_WAIT_MS);
        break;

    case T6_STATE_ABS_N:
        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, -T6_MOVE_PULSES,
                                T6_POS_RPM, T6_POS_ACC);
        s_absTarget = -T6_MOVE_PULSES;
        Task6_SetUi("ABS", s_absTarget, true);
        Task6_EnterDwell(now, T6_STATE_ABS_Z2, T6_SEGMENT_WAIT_MS);
        break;

    case T6_STATE_ABS_Z2:
        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, 0, T6_POS_RPM, T6_POS_ACC);
        s_absTarget = 0;
        Task6_SetUi("ABS", s_absTarget, true);
        Task6_EnterDwell(now, T6_STATE_CYCLE_END, T6_SEGMENT_WAIT_MS);
        break;

#if (T6_RAMP_TEST_ENABLE != 0U)
    case T6_STATE_RAMP_START: {
        AppVisionXSample_t sample;

        /*
         * 必须先拿到一帧有效 X 当起点，否则无从判断球有没有动。视觉没就绪就
         * 停在本状态等（OLED 显示 NOX），不会盲目开始加命令。
         */
        AppVisionLink_GetLatestX(&sample);
        if (!sample.valid) {
            Task6_SetUi((s_rampDir > 0) ? "RAMP+ NOX" : "RAMP- NOX", 0, false);
            break;
        }

        /* 每个方向都从命令 0（归零点）重新起步，两个方向的读数才可比。 */
        s_rampStartX = sample.pixel;
        s_rampCmd    = 0;
        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, 0, T6_RAMP_RPM, T6_POS_ACC);
        s_absTarget  = 0;
        Task6_RampLog((s_rampDir > 0) ? "T6 RAMP+ startX=" : "T6 RAMP- startX=",
                      (int32_t)s_rampStartX);
        s_stateStartTick = now;
        s_state = T6_STATE_RAMP_UP;
        break;
    }

    case T6_STATE_RAMP_UP: {
        AppVisionXSample_t sample;

        /* 先判球动没动：只用有效帧，NA/无效帧直接跳过，避免误触发。 */
        AppVisionLink_GetLatestX(&sample);
        if (sample.valid &&
            (Task6_Abs32((int32_t)sample.pixel - (int32_t)s_rampStartX) >=
             T6_RAMP_MOVE_THRESHOLD_PX)) {
            int32_t magnitude = Task6_Abs32(s_rampCmd);

            if (s_rampDir > 0) {
                s_rampPosBreak = magnitude;
                Task6_RampLog("T6 RAMP+ break=", magnitude);
            } else {
                s_rampNegBreak = magnitude;
                Task6_RampLog("T6 RAMP- break=", magnitude);
            }
            s_state = T6_STATE_RAMP_BACK;
            break;
        }

        /* 到点才加一档，加完立刻下发；单档 20 脉冲远小于间隔，不会被覆盖。 */
        if (Task6_Elapsed(now, s_stateStartTick, T6_RAMP_INTERVAL_MS)) {
            s_stateStartTick = now;
            s_rampCmd += s_rampDir * (int32_t)T6_RAMP_STEP_PULSES;
            if (Task6_Abs32(s_rampCmd) > T6_RAMP_MAX_PULSES) {
                /* 到安全上限仍不动：本方向记 0 表示未测到，绝不把上限当结果。 */
                if (s_rampDir > 0) {
                    s_rampPosBreak = 0;
                    Task6_RampLog("T6 RAMP+ NOT FOUND up to ", T6_RAMP_MAX_PULSES);
                } else {
                    s_rampNegBreak = 0;
                    Task6_RampLog("T6 RAMP- NOT FOUND up to ", T6_RAMP_MAX_PULSES);
                }
                s_state = T6_STATE_RAMP_BACK;
                break;
            }
            Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, s_rampCmd, T6_RAMP_RPM, T6_POS_ACC);
            s_absTarget = s_rampCmd;
        }
        Task6_SetUi((s_rampDir > 0) ? "RAMP+" : "RAMP-", s_rampCmd, true);
        break;
    }

    case T6_STATE_RAMP_BACK:
        /* 命令回水平，别让球一直被推着滚到端点。 */
        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, 0, T6_RAMP_RPM, T6_POS_ACC);
        s_rampCmd   = 0;
        s_absTarget = 0;
        if (s_rampDir > 0) {
            /* 正向测完，留时间把球放回中间，再自动测负向。 */
            s_rampDir = -1;
            Task6_SetUi("RECENTER BALL", 0, false);
            Task6_EnterDwell(now, T6_STATE_RAMP_START, T6_RAMP_RECENTER_MS);
        } else {
            s_state = T6_STATE_RAMP_REPORT;
        }
        break;

    case T6_STATE_RAMP_REPORT:
        /* 结果交给 GetUiStatus 持续显示；串口再打一遍换算好的两个数。 */
        s_rampReportReady = true;
        Task6_RampLog("T6 RAMP RESULT P=", s_rampPosBreak);
        Task6_RampLog("T6 RAMP RESULT N=", s_rampNegBreak);
        if ((s_rampPosBreak != 0) && (s_rampNegBreak != 0)) {
            Task6_RampLog("T6 breakaway B=", (s_rampPosBreak + s_rampNegBreak) / 2);
            Task6_RampLog("T6 level trim L=", (s_rampPosBreak - s_rampNegBreak) / 2);
        }
        RobotCore_NotifyTaskFinished(T6_TASK_INDEX);
        s_state = T6_STATE_DONE;
        break;
#endif /* T6_RAMP_TEST_ENABLE */

    case T6_STATE_DWELL:
        if (Task6_Elapsed(now, s_stateStartTick, s_dwellMs)) {
            s_state = s_dwellNext;
        }
        break;

    case T6_STATE_CYCLE_END:
        /* 本状态不发帧，直接决定继续循环还是收尾。 */
#if (T6_LOOP_FOREVER != 0U)
        s_state = Task6_FirstMoveState();
#else
        /*
         * 不跑急停帧：此时电机已经到位，保持使能让驱动器用保持力矩停在目标位置，
         * 方便拿尺子量行程。K4 退出时才急停并失能。
         */
        s_state = T6_STATE_DONE;
#endif
        break;

    case T6_STATE_STOP:
        /* 先急停，下一个 UI 周期再失能，让两条 UART1 帧自然错开。 */
        Emm42Robot_Stop(EMM42_ROBOT_LIFT);
        Task6_SetUi("STOP", 0, false);
        s_state = T6_STATE_DONE;
        break;

    case T6_STATE_DONE:
        Task6_SetUi("DONE", s_absTarget, true);
        RobotCore_NotifyTaskFinished(T6_TASK_INDEX);
        break;

    default:
        /* 状态异常时只对本题的 ID1 做安全收尾。 */
        Emm42Robot_Stop(EMM42_ROBOT_LIFT);
        Task6_SetUi("FAULT", 0, false);
        s_state = T6_STATE_DONE;
        break;
    }
}

void Task6_OnExit(void)
{
    /* K4 退出时仅急停、失能 ID1；两帧间隔 5ms，避免共享总线连续帧互相干扰。 */
    Emm42Robot_Stop(EMM42_ROBOT_LIFT);
    vTaskDelay(pdMS_TO_TICKS(T6_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);

    s_state          = T6_STATE_WAIT_BALL_RELEASE;
    s_dwellNext      = T6_STATE_DONE;
    s_dwellMs        = T6_DWELL_MS;
    s_stateStartTick = 0U;
    s_absTarget      = 0;
#if (T6_RAMP_TEST_ENABLE != 0U)
    /* 不清 s_rampPosBreak/NegBreak/ReportReady：退出后结果仍可留在串口日志里核对。 */
    s_rampDir         = +1;
    s_rampCmd         = 0;
#endif
    Task6_SetUi("IDLE", 0, false);
}

const char *Task6_GetUiStatus(void)
{
    uint32_t idx = 0U;
    const char *p = s_uiPhase;

#if (T6_RAMP_TEST_ENABLE != 0U)
    /*
     * 标定结果优先显示，且一直保持在屏上直到 K4 退出，方便抄数。
     * 两个方向都测到就直接给换算好的 B/L；有一个方向没测到就给原始 P/N，
     * 不算平均——否则会输出一个看似合理其实无意义的"实测值"。
     */
    if (s_rampReportReady) {
        if ((s_rampPosBreak != 0) && (s_rampNegBreak != 0)) {
            s_uiStatusBuf[idx++] = 'B';
            idx = Task6_AppendSigned(s_uiStatusBuf, idx,
                                     (s_rampPosBreak + s_rampNegBreak) / 2,
                                     sizeof(s_uiStatusBuf) - 1U);
            s_uiStatusBuf[idx++] = ' ';
            s_uiStatusBuf[idx++] = 'L';
            idx = Task6_AppendSigned(s_uiStatusBuf, idx,
                                     (s_rampPosBreak - s_rampNegBreak) / 2,
                                     sizeof(s_uiStatusBuf) - 1U);
        } else {
            s_uiStatusBuf[idx++] = 'P';
            idx = Task6_AppendSigned(s_uiStatusBuf, idx, s_rampPosBreak,
                                     sizeof(s_uiStatusBuf) - 1U);
            s_uiStatusBuf[idx++] = ' ';
            s_uiStatusBuf[idx++] = 'N';
            idx = Task6_AppendSigned(s_uiStatusBuf, idx, s_rampNegBreak,
                                     sizeof(s_uiStatusBuf) - 1U);
        }
        s_uiStatusBuf[idx] = '\0';
        return s_uiStatusBuf;
    }
#endif

    while ((*p != '\0') && (idx < (sizeof(s_uiStatusBuf) - 1U))) {
        s_uiStatusBuf[idx++] = *p++;
    }

    if (s_uiHasValue && (idx < (sizeof(s_uiStatusBuf) - 13U))) {
        uint32_t value;
        char     digits[12];
        uint32_t n = 0U;

        s_uiStatusBuf[idx++] = ' ';
        if (s_uiValue < 0) {
            s_uiStatusBuf[idx++] = '-';
            value = (uint32_t)(-s_uiValue);
        } else {
            s_uiStatusBuf[idx++] = '+';
            value = (uint32_t)s_uiValue;
        }

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
    }

    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}
