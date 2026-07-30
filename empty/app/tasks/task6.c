#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ball_control_task.h"
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
 *   4. T6_STEP_TEST_ENABLE=1 时【完全替代】上面三项，改为阶梯步长测试：
 *      从水平位置依次下发 s_stepTestPulses[] 里的小增量绝对目标，每步停留
 *      T6_STEP_TEST_DWELL_MS 观察钢球是否开始可见地滚动，然后回到水平再测
 *      下一步。用于标定"钢球在凹槽里的静摩擦阈值"——多小的倾角变化才能真的
 *      让球动起来，这与题目六前三项测的"丝杆本身能不能被位置模式驱动"是
 *      两件事：机构本身的大幅度运动早已验证过，这里测的是小增量下球的响应。
 *
 * 【所有位置都锚在"按 K3 进入本题那一刻的电机位置"上】：相对模式命令本就以当前
 * 实际位置为起点；绝对模式的原点也是进入本题时才建立的，所以不存在跨越上电或
 * 跨越一次进出的持久绝对坐标。⚠️ 反复退出再进入，每次都会以当时的位置重新置零，
 * 单程模式下位移会一路累积，量完记得手摇回行程中部再测下一次。
 *
 * 位置原点：进入本题时（T6_ZERO_ON_ENTER）发 0x0A 0x6D 把当前机械位置定义为 0。
 * 摆杆没有限位开关/角度传感器，所以【必须先人工把杆摆到目视水平再按 K3 进本题】；
 * 驱动器断电不保留多圈位置计数，每次上电都要重做一遍。
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
#define T6_TEST_REVS             (10)     /* 每段转几圈 */
#define T6_MOVE_PULSES           ((int32_t)T6_TEST_REVS * (int32_t)T6_PULSES_PER_REV)

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
 * ---- 阶梯步长测试（静摩擦阈值标定，与上面三项互斥） ----
 * 置 1 后【完全替代】相对/绝对测试：从水平位置依次下发下方数组里的小增量
 * 绝对目标，每步等到理论运动时间走完后再停留 T6_STEP_TEST_DWELL_MS 观察
 * 钢球是否开始可见地滚动，然后回到水平再测下一步。目的是标定"钢球在凹槽
 * 里的静摩擦阈值"——摆杆倾角变化多小，球才会真的开始滚，而不是丝杆已经
 * 动了、球却纹丝不动。只测 T6_FIRST_DIR 方向；要测另一个方向就把
 * T6_FIRST_DIR 改成 -1 重新跑一遍。
 *
 * 2026-07-31 实测 200~1200 脉冲全程无反应，且已确认进题目前摆杆确实水平
 * （排除"零点没摆平、一上来就顶到限位"），改成更大的步长（0.5~4 圈）继续测。
 */
#define T6_STEP_TEST_ENABLE      (1U)
#define T6_STEP_TEST_COUNT       (6U)
#define T6_STEP_TEST_DWELL_MS    (3000U)  /* 到位后【额外】观察时间，够肉眼判断球是否开始滚动 */
static const int32_t s_stepTestPulses[T6_STEP_TEST_COUNT] =
    {1600, 3200, 4800, 6400, 9600, 12800};

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
#define T6_MOVE_TIME_MS          ((60000UL * (uint32_t)(T6_TEST_REVS)) / (uint32_t)(T6_POS_RPM))
#define T6_SEGMENT_WAIT_MS       (((T6_MOVE_TIME_MS) * (100U + T6_MOVE_MARGIN_PCT) / 100U) + \
                                  (uint32_t)(T6_DWELL_MS))

/* ---- 沿用 task2/task3 已实测的 Emm42 时序，不要往下调 ---- */
#define T6_RESET_SETTLE_MS       (60U)
#define T6_ENABLE_SETTLE_MS      (180U)   /* 90ms 实测约一半概率使能不生效 */
#define T6_EMM_CMD_GAP_MS        (5U)

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
    T6_STATE_STEP_CLEAR_CLOG,       /* 阶梯测试：每步前先解一次堵转保护 */
    T6_STATE_STEP_APPLY,            /* 阶梯测试：下发本步的绝对目标脉冲 */
    T6_STATE_STEP_RETURN,           /* 阶梯测试：命令回到水平，为下一步做准备 */
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

/* 阶梯测试当前测到第几步（s_stepTestPulses 的下标）。 */
static uint32_t      s_stepIndex;

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

/*
 * 阶梯测试的步长是运行时数组值（不是编译期常量），不能直接套用
 * T6_SEGMENT_WAIT_MS 那个基于 T6_TEST_REVS 的宏，改成按传入脉冲数现算：
 * 理论运动时间(ms) = 脉冲数 / 每圈脉冲数 / 转速(RPM) 换算成毫秒，
 * 加上同样的余量百分比，再叠加观察时间。步长越大，等待也要跟着变长，
 * 否则大步长会重蹈"命令被覆盖、丝杆根本没走到位"的覆辙。
 */
static uint32_t Task6_StepWaitMs(int32_t pulses)
{
    uint32_t absPulses  = (uint32_t)((pulses < 0) ? -pulses : pulses);
    uint32_t moveTimeMs = (60000UL * absPulses) /
                          ((uint32_t)T6_PULSES_PER_REV * (uint32_t)T6_POS_RPM);
    uint32_t withMargin = (moveTimeMs * (100U + T6_MOVE_MARGIN_PCT)) / 100U;

    return withMargin + T6_STEP_TEST_DWELL_MS;
}

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
#if (T6_STEP_TEST_ENABLE != 0U)
    s_stepIndex = 0U;
    return T6_STATE_STEP_CLEAR_CLOG;
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
    s_stepIndex      = 0U;
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

    case T6_STATE_STEP_CLEAR_CLOG:
        /*
         * 每步都先解一次堵转保护：如果上一步已经把丝杆顶到机械限位触发了
         * 堵转保护，不在这里清掉的话，后面所有 MoveAbsolute 都会被驱动器
         * 拒绝执行（返回 E2）且电机纹丝不动——表现就是"发的脉冲越来越大，
         * 但从头到尾什么反应都没有"，容易被误判成钢球静摩擦特别大。
         */
        Emm42Robot_ClearClogProtection(EMM42_ROBOT_LIFT);
        s_state = T6_STATE_STEP_APPLY;
        break;

    case T6_STATE_STEP_APPLY: {
        int32_t pulses = (int32_t)T6_FIRST_DIR * s_stepTestPulses[s_stepIndex];

        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, pulses, T6_POS_RPM, T6_POS_ACC);
        s_absTarget = pulses;
        Task6_SetUi("STEP", pulses, true);
        Task6_EnterDwell(now, T6_STATE_STEP_RETURN, Task6_StepWaitMs(pulses));
        break;
    }

    case T6_STATE_STEP_RETURN: {
        /* 回程距离跟去程一样远，等待时间也要按同一个步长的脉冲数现算。 */
        uint32_t waitMs = Task6_StepWaitMs(s_stepTestPulses[s_stepIndex]);

        Emm42Robot_MoveAbsolute(EMM42_ROBOT_LIFT, 0, T6_POS_RPM, T6_POS_ACC);
        s_absTarget = 0;
        Task6_SetUi("STEP RET", 0, true);
        s_stepIndex++;
        if (s_stepIndex >= T6_STEP_TEST_COUNT) {
            Task6_EnterDwell(now, T6_STATE_CYCLE_END, waitMs);
        } else {
            Task6_EnterDwell(now, T6_STATE_STEP_CLEAR_CLOG, waitMs);
        }
        break;
    }

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
    s_stepIndex      = 0U;
    Task6_SetUi("IDLE", 0, false);
}

const char *Task6_GetUiStatus(void)
{
    uint32_t idx = 0U;
    const char *p = s_uiPhase;

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
