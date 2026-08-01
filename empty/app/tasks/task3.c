#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ball_control_task.h"
#include "bsp_buzzer.h"
#include "bsp_home_switch.h"
#include "emm42_robot.h"

/* ==================================================================
 * 题目 3：左右摆球。
 *
 * 进题后先让 ID1 触碰 PA24 归零限位，再抬升到水平附近；随后自动以 X=350
 * 启动钢球闭环并等待 K3。确认后 ID2/ID3 仅使能并保持 0 RPM，钢珠开始单次
 * 摆向左侧，再摆向右侧。
 * ================================================================== */

/* ---- 中心等待目标与左右摆动目标：左右目标可独立按实际物理位置标定。 ---- */
#define T3_BALL_CENTER_X_PX               (350)  /* 进题回零抬升后、等待 K3 时持续保持的中心位置。 */
#define T3_BALL_LEFT_TARGET_X_PX          (240)  /* K3 后的第 1 段目标；首次进入到达带后切第 2 段。 */
#define T3_BALL_MIDDLE_TARGET_X_PX        (380)  /* 第 2 段目标；从左向右经过阈值后切最终段。 */
#define T3_BALL_MIDDLE_PASS_X_PX          (320)  /* 第 2 段经过线：达到此值即切最终段，不等待停稳。 */
#define T3_BALL_FINAL_TARGET_X_PX         (470)  /* 第 3 段最终目标；满足稳定条件后鸣叫并结束。 */
#define T3_BALL_LEFT_ARRIVAL_BAND_PX      (30)   /* 第 1 段到达带半宽。 */
#define T3_BALL_FINAL_GUARD_X_PX          (550)  /* 最终段右侧保护线：测量值达到此处立即重投最终点回拉。 */
#define T3_BALL_FINAL_HOLD_TIME_MS        (80U) /* 最终点的位置、速度均合格后，连续保持多久才算完成。 */

/* ---- 本题独立的限位回零与轮子使能参数。 ---- */
#define T3_HOME_SEEK_RPM                  (5)    /* 找限位开关的速度；代码以负方向运动，数值越大越快。 */
#define T3_HOME_LIFT_RPM                  (5U)   /* 压限位后正方向抬升曲柄的速度。 */
#define T3_HOME_ACC                       (5U)   /* 回零/抬升的驱动器加速度档位；0 表示立即达到命令速度。 */
#define T3_HOME_LEVEL_OFFSET_PULSES       (240)  /* 从限位触发点向上抬升的脉冲数，决定球臂初始工作高度。 */
#define T3_HOME_RESET_SETTLE_TICKS        (2U)   /* ID1 失能后等待拍数，避免紧接着解堵/使能时丢帧。 */
#define T3_HOME_ENABLE_SETTLE_TICKS       (6U)   /* ID1 使能后等待拍数，确认驱动器已真正受控。 */
#define T3_HOME_STOP_SETTLE_TICKS         (1U)   /* 碰限位急停后等待拍数，再发送抬升位置命令。 */
#define T3_HOME_LIFT_WAIT_TICKS           (35U)  /* 等待抬升走完的拍数；增大抬升脉冲或降低速度时需同步增大。 */
#define T3_WHEEL_RESET_SETTLE_TICKS       (2U)   /* 清除 ID2/ID3 残留状态后等待拍数。 */
#define T3_WHEEL_ENABLE_SETTLE_TICKS      (6U)   /* 每个轮子使能后的等待拍数；不要低于已验证的稳定值。 */
#define T3_WHEEL_ZERO_ACC                 (80U)  /* 给轮子发送 0 RPM 保持帧时使用的加速度档位。 */
#define T3_EMM_CMD_GAP_MS                 (6U)   /* K4 收尾时 UART1 相邻电机帧的最小间隔，防止总线丢帧。 */

/*
 * 本题私有钢珠闭环参数。初值按任务五当前实车参数复制，后续只调 T3_BALL_*，
 * 不会影响任务四、五或菜单默认 profile。
 */
#define T3_BALL_FRICTION_FF_PULSE         (0.0F) /* 库仑摩擦前馈脉冲；0 表示关闭，当前仅用 PD 控制。 */
#define T3_BALL_LEVEL_TRIM_PULSE          (-54)  /* 真实物理水平点相对位置零点的脉冲修正；用于消除固定偏置。 */
#define T3_BALL_FILTER_ALPHA              (0.9F) /* α-β 滤波的位置更新权重；越大越跟随视觉，越小越平滑。 */
#define T3_BALL_FILTER_BETA               (0.2F) /* α-β 滤波的速度更新权重；越大速度响应越快，也更易放大噪声。 */
#define T3_BALL_OUTPUT_SIGN               (-1.0F)/* 闭环输出方向；实机方向反了只改正负号，不改全局电机标定。 */
#define T3_BALL_KX_PULSE_PER_PX           (1.066F)/* 位置比例增益；增大响应更快，但过大容易过冲。 */
#define T3_BALL_KV_PULSE_PER_PXPS         (0.504F)/* 速度阻尼增益；增大刹车更强，过大可能跟随速度噪声抖动。 */
#define T3_BALL_SETTLE_DEADBAND_PX        (4.0F) /* 控制死区；误差进入此范围且球慢时回水平保持，防止频繁微调。 */
#define T3_BALL_FF_VEL_BLEND_PXPS         (15.0F)/* 球接近静止的速度门限，供保持判定和摩擦前馈切换使用。 */
#define T3_BALL_POS_RPM                   (200U) /* ID1 位置模式的最高转速；三段目标共用，任务五同值。 */
#define T3_BALL_POS_ACC                   (240U) /* ID1 位置模式加速度档位；越大起停越猛，任务五同值。 */
#define T3_BALL_MAX_PULSE_STEP            (0U)   /* 单帧目标脉冲变化上限；0 不限速，非零可削弱突变但会变慢。 */
#define T3_BALL_HOLD_POSITION_PX          (6.0F) /* 最终 450 到位的位置误差上限。 */
#define T3_BALL_HOLD_VELOCITY_PXPS        (10.0F)/* 最终 450 到位的速度上限，必须与位置误差同时满足。 */
#define T3_BALL_HOLD_TIME_MS              T3_BALL_FINAL_HOLD_TIME_MS /* 最终到位条件连续成立的时间。 */

/* 题目三在任务表中的固定下标。 */
#define T3_TASK_INDEX                     (2U)

/*
 * K3 后计时与录像窗口（参考任务二秒表与任务一录像时长）：K3 触发三段摆动那一刻
 * 开始计时并通知视觉端开始录像；累计校准时间达到 T3_RECORD_DURATION_MS 后停录像、
 * 暂停计时，小球继续走完三段不受影响。
 */
#define T3_TICK_MS                        (30U)    /* OnLoop 周期，与 UIMENU 一致。 */
#define T3_RECORD_DURATION_MS             (5000U)  /* 录像+计时窗口，到时停录像并暂停计时。 */
#define T3_STOPWATCH_CAL_SCALE            (0.897F) /* 秒表校准系数，与任务二/四/五同值。 */

typedef enum {
    T3_STATE_WAIT_BALL_RELEASE = 0,
    T3_STATE_HOME_DISABLE,
    T3_STATE_HOME_DISABLE_WAIT,
    T3_STATE_HOME_CLEAR_CLOG,
    T3_STATE_HOME_ENABLE,
    T3_STATE_HOME_ENABLE_WAIT,
    T3_STATE_HOME_SEEK,
    T3_STATE_HOME_STOP_WAIT,
    T3_STATE_HOME_LIFT,
    T3_STATE_HOME_LIFT_WAIT,
    T3_STATE_HOME_ZERO,
    T3_STATE_CENTER_REQUEST,
    T3_STATE_CENTER_WAIT,
    T3_STATE_WHEEL_STOP_LEFT,
    T3_STATE_WHEEL_STOP_RIGHT,
    T3_STATE_WHEEL_DISABLE_LEFT,
    T3_STATE_WHEEL_DISABLE_RIGHT,
    T3_STATE_WHEEL_RESET_WAIT,
    T3_STATE_WHEEL_ENABLE_LEFT,
    T3_STATE_WHEEL_ENABLE_LEFT_WAIT,
    T3_STATE_WHEEL_ENABLE_RIGHT,
    T3_STATE_WHEEL_ENABLE_RIGHT_WAIT,
    T3_STATE_WHEEL_ZERO_LEFT,
    T3_STATE_WHEEL_ZERO_RIGHT,
    T3_STATE_WAIT_CONFIRM,
    T3_STATE_LEFT_REQUEST,
    T3_STATE_LEFT_APPROACH,
    T3_STATE_MIDDLE_REQUEST,
    T3_STATE_MIDDLE_APPROACH,
    T3_STATE_FINAL_REQUEST,
    T3_STATE_FINAL_HOLD,
    T3_STATE_BALL_RECOVER_WAIT,
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
static Task3State_t s_resumeState;
static uint32_t s_settleTicks;
static bool s_confirmRequested;
static bool s_finishBeeped;
static bool s_finalGuardTriggered;
static uint32_t s_elapsedTicks;   /* K3 后累计 OnLoop 拍数；5 秒到后停止累加（计时暂停）。 */
static bool s_timerRunning;       /* 计时是否在跑（K3 后到 5 秒之间）。 */
static bool s_timerFinished;      /* 5 秒已到，录像已停、计时已暂停。 */
static char s_uiStatusBuf[24];    /* 秒表文本缓冲。 */

/* 秒表校准后的累计毫秒数，与 OLED 显示使用同一套换算。 */
static uint32_t Task3_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T3_TICK_MS) * T3_STOPWATCH_CAL_SCALE);
}

static bool Task3_IsLeftState(Task3State_t state)
{
    return (state == T3_STATE_LEFT_REQUEST) ||
           (state == T3_STATE_LEFT_APPROACH);
}

static bool Task3_IsMiddleState(Task3State_t state)
{
    return (state == T3_STATE_MIDDLE_REQUEST) ||
           (state == T3_STATE_MIDDLE_APPROACH);
}

static bool Task3_IsCenterState(Task3State_t state)
{
    return (state == T3_STATE_CENTER_REQUEST) ||
           (state == T3_STATE_CENTER_WAIT) ||
           (state == T3_STATE_WAIT_CONFIRM);
}

static Task3State_t Task3_GetRecoveryState(void)
{
    if (Task3_IsCenterState(s_state)) {
        return T3_STATE_CENTER_REQUEST;
    }

    if (Task3_IsLeftState(s_state)) {
        return T3_STATE_LEFT_REQUEST;
    }
    if (Task3_IsMiddleState(s_state)) {
        return T3_STATE_MIDDLE_REQUEST;
    }
    return T3_STATE_FINAL_REQUEST;
}

static bool Task3_RequestBallTarget(int16_t targetX)
{
    return AppBallControl_RequestTargetWithProfile(targetX, &s_task3BallProfile);
}

static bool Task3_IsBallHoldingTarget(const AppBallControlStatus_t *status,
                                      int16_t targetX)
{
    return (status->targetPx == targetX) &&
           (status->state == APP_BALL_CONTROL_HOLDING);
}

const char *Task3_GetUiStatus(void)
{
    uint32_t totalMs;
    uint32_t secWhole;
    uint32_t tenths;
    uint32_t idx = 0U;
    uint32_t n = 0U;
    uint32_t value;
    char digits[10];
    const char *phase;

    /* K3 前：显示阶段提示，不显示秒表。 */
    if (!s_timerRunning && !s_timerFinished) {
        switch (s_state) {
        case T3_STATE_WAIT_BALL_RELEASE:
            return "T3 B RELEASE";
        case T3_STATE_HOME_DISABLE:
        case T3_STATE_HOME_DISABLE_WAIT:
        case T3_STATE_HOME_CLEAR_CLOG:
        case T3_STATE_HOME_ENABLE:
        case T3_STATE_HOME_ENABLE_WAIT:
        case T3_STATE_HOME_SEEK:
        case T3_STATE_HOME_STOP_WAIT:
        case T3_STATE_HOME_LIFT:
        case T3_STATE_HOME_LIFT_WAIT:
        case T3_STATE_HOME_ZERO:
            return "T3 HOME";
        case T3_STATE_CENTER_REQUEST:
        case T3_STATE_CENTER_WAIT:
            return "T3 X350 WAIT";
        case T3_STATE_WAIT_CONFIRM:
            return "T3 K3=GO";
        case T3_STATE_BALL_RECOVER_WAIT:
            return "T3 B RECOV";
        default:
            return "T3 ???";
        }
    }

    /* K3 后：显示秒表 T:x.xs + 阶段简称，5 秒到后定格在 T:5.0s。 */
    totalMs = Task3_GetElapsedMs();
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

    switch (s_state) {
    case T3_STATE_WHEEL_STOP_LEFT:
    case T3_STATE_WHEEL_STOP_RIGHT:
    case T3_STATE_WHEEL_DISABLE_LEFT:
    case T3_STATE_WHEEL_DISABLE_RIGHT:
    case T3_STATE_WHEEL_RESET_WAIT:
    case T3_STATE_WHEEL_ENABLE_LEFT:
    case T3_STATE_WHEEL_ENABLE_LEFT_WAIT:
    case T3_STATE_WHEEL_ENABLE_RIGHT:
    case T3_STATE_WHEEL_ENABLE_RIGHT_WAIT:
    case T3_STATE_WHEEL_ZERO_LEFT:
    case T3_STATE_WHEEL_ZERO_RIGHT:
        phase = "ARM";
        break;
    case T3_STATE_LEFT_REQUEST:
    case T3_STATE_LEFT_APPROACH:
        phase = "LFT";
        break;
    case T3_STATE_MIDDLE_REQUEST:
    case T3_STATE_MIDDLE_APPROACH:
        phase = "MID";
        break;
    case T3_STATE_FINAL_REQUEST:
    case T3_STATE_FINAL_HOLD:
        phase = "FIN";
        break;
    case T3_STATE_BALL_RECOVER_WAIT:
        phase = "REC";
        break;
    case T3_STATE_FINISHED:
        phase = "DONE";
        break;
    default:
        phase = "???";
        break;
    }

    while ((*phase != '\0') && (idx < (sizeof(s_uiStatusBuf) - 1U))) {
        s_uiStatusBuf[idx++] = *phase++;
    }
    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}

void Task3_OnConfirm(void)
{
    if (s_state == T3_STATE_WAIT_CONFIRM) {
        s_confirmRequested = true;
    }
}

void Task3_OnEnter(void)
{
    s_state = T3_STATE_WAIT_BALL_RELEASE;
    s_resumeState = T3_STATE_CENTER_REQUEST;
    s_settleTicks = 0U;
    s_confirmRequested = false;
    s_finishBeeped = false;
    s_finalGuardTriggered = false;
    s_elapsedTicks = 0U;
    s_timerRunning = false;
    s_timerFinished = false;
}

void Task3_OnLoop(void)
{
    AppBallControlStatus_t ballStatus;

    AppBallControl_GetStatus(&ballStatus);

    /* K3 后开始计时+录像：累计到 5 秒停录像并暂停计时，球继续走三段。 */
    if (s_timerRunning) {
        s_elapsedTicks++;
        if (Task3_GetElapsedMs() >= T3_RECORD_DURATION_MS) {
            s_timerRunning = false;
            s_timerFinished = true;
            RobotCore_NotifyTaskFinished(T3_TASK_INDEX);
        }
    }

    if (s_state == T3_STATE_WAIT_BALL_RELEASE) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) {
            s_state = T3_STATE_HOME_DISABLE;
        }
        return;
    }

    if (s_state == T3_STATE_HOME_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);
        s_settleTicks = T3_HOME_RESET_SETTLE_TICKS;
        s_state = T3_STATE_HOME_DISABLE_WAIT;
        return;
    }
    if (s_state == T3_STATE_HOME_DISABLE_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_HOME_CLEAR_CLOG;
        return;
    }
    if (s_state == T3_STATE_HOME_CLEAR_CLOG) {
        Emm42Robot_ClearClogProtection(EMM42_ROBOT_LIFT);
        s_state = T3_STATE_HOME_ENABLE;
        return;
    }
    if (s_state == T3_STATE_HOME_ENABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
        s_settleTicks = T3_HOME_ENABLE_SETTLE_TICKS;
        s_state = T3_STATE_HOME_ENABLE_WAIT;
        return;
    }
    if (s_state == T3_STATE_HOME_ENABLE_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_HOME_SEEK;
        return;
    }
    if (s_state == T3_STATE_HOME_SEEK) {
        if (BspHomeSwitch_IsPressed()) {
            Emm42Robot_Stop(EMM42_ROBOT_LIFT);
            s_settleTicks = T3_HOME_STOP_SETTLE_TICKS;
            s_state = T3_STATE_HOME_STOP_WAIT;
        } else {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT,
                                   (int16_t)-T3_HOME_SEEK_RPM, T3_HOME_ACC);
        }
        return;
    }
    if (s_state == T3_STATE_HOME_STOP_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_HOME_LIFT;
        return;
    }
    if (s_state == T3_STATE_HOME_LIFT) {
        Emm42Robot_MoveRelative(EMM42_ROBOT_LIFT, T3_HOME_LEVEL_OFFSET_PULSES,
                                T3_HOME_LIFT_RPM, T3_HOME_ACC);
        s_settleTicks = T3_HOME_LIFT_WAIT_TICKS;
        s_state = T3_STATE_HOME_LIFT_WAIT;
        return;
    }
    if (s_state == T3_STATE_HOME_LIFT_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_HOME_ZERO;
        return;
    }
    if (s_state == T3_STATE_HOME_ZERO) {
        /*
         * 每次进题都重新经过限位和水平偏移，必须同步刷新驱动器的多圈位置
         * 原点；否则 BALLCTRL 的绝对位置命令会沿用上次进题的坐标。
         */
        Emm42Robot_ResetPosToZero(EMM42_ROBOT_LIFT);
        s_state = T3_STATE_CENTER_REQUEST;
        return;
    }

    /*
     * 轮子只保留使能和保持力矩。先清掉上次任务留下的速度，再按已验证的
     * "单轴使能→等待 6 拍"时序使能，最后明确发送 0 RPM。
     */
    if (s_state == T3_STATE_WHEEL_STOP_LEFT) {
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
        s_state = T3_STATE_WHEEL_STOP_RIGHT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_STOP_RIGHT) {
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
        s_state = T3_STATE_WHEEL_DISABLE_LEFT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_DISABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        s_state = T3_STATE_WHEEL_DISABLE_RIGHT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_DISABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_settleTicks = T3_WHEEL_RESET_SETTLE_TICKS;
        s_state = T3_STATE_WHEEL_RESET_WAIT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_RESET_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_WHEEL_ENABLE_LEFT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_ENABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_settleTicks = T3_WHEEL_ENABLE_SETTLE_TICKS;
        s_state = T3_STATE_WHEEL_ENABLE_LEFT_WAIT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_ENABLE_LEFT_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_WHEEL_ENABLE_RIGHT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_ENABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_settleTicks = T3_WHEEL_ENABLE_SETTLE_TICKS;
        s_state = T3_STATE_WHEEL_ENABLE_RIGHT_WAIT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_ENABLE_RIGHT_WAIT) {
        if (s_settleTicks-- > 0U) {
            return;
        }
        s_state = T3_STATE_WHEEL_ZERO_LEFT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_ZERO_LEFT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T3_WHEEL_ZERO_ACC);
        s_state = T3_STATE_WHEEL_ZERO_RIGHT;
        return;
    }
    if (s_state == T3_STATE_WHEEL_ZERO_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T3_WHEEL_ZERO_ACC);
        s_state = T3_STATE_LEFT_REQUEST;
        return;
    }

    /*
     * BALLCTRL 触发边缘保护后会锁在 FAULT_EDGE。任务侧必须先请求停止、等
     * ID1 真正释放，再从中断前的中心、左侧或右侧目标重新开始闭环。
     */
    if ((s_state != T3_STATE_BALL_RECOVER_WAIT) &&
        (ballStatus.state == APP_BALL_CONTROL_FAULT_EDGE)) {
        s_resumeState = Task3_GetRecoveryState();
        AppBallControl_RequestStop();
        s_state = T3_STATE_BALL_RECOVER_WAIT;
        return;
    }
    if (s_state == T3_STATE_BALL_RECOVER_WAIT) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) {
            s_state = s_resumeState;
        }
        return;
    }

    if (s_state == T3_STATE_CENTER_REQUEST) {
        if (Task3_RequestBallTarget(T3_BALL_CENTER_X_PX)) {
            s_state = T3_STATE_CENTER_WAIT;
        }
        return;
    }
    if (s_state == T3_STATE_CENTER_WAIT) {
        /* 目标已被控制线程接管后即可等待人工确认，期间持续保持 X=350。 */
        if ((ballStatus.targetPx == T3_BALL_CENTER_X_PX) &&
            ((ballStatus.state == APP_BALL_CONTROL_RUNNING) ||
             (ballStatus.state == APP_BALL_CONTROL_HOLDING))) {
            s_state = T3_STATE_WAIT_CONFIRM;
        }
        return;
    }
    if (s_state == T3_STATE_WAIT_CONFIRM) {
        if (s_confirmRequested) {
            s_confirmRequested = false;
            /* K3 触发三段摆动：开始计时并通知视觉端开始录像。 */
            s_timerRunning = true;
            RobotCore_NotifyTaskStarted(T3_TASK_INDEX);
            s_state = T3_STATE_WHEEL_STOP_LEFT;
        }
        return;
    }
    if (s_state == T3_STATE_LEFT_REQUEST) {
        if (Task3_RequestBallTarget(T3_BALL_LEFT_TARGET_X_PX)) {
            s_state = T3_STATE_LEFT_APPROACH;
        }
        return;
    }
    if (s_state == T3_STATE_LEFT_APPROACH) {
        /* 左侧不等待完全静止，视觉新样本首次进入第 1 段到达带即切阶段二。 */
        if ((ballStatus.targetPx == T3_BALL_LEFT_TARGET_X_PX) &&
            (ballStatus.measuredPx >=
             (T3_BALL_LEFT_TARGET_X_PX - T3_BALL_LEFT_ARRIVAL_BAND_PX)) &&
            (ballStatus.measuredPx <=
             (T3_BALL_LEFT_TARGET_X_PX + T3_BALL_LEFT_ARRIVAL_BAND_PX))) {
            s_state = T3_STATE_MIDDLE_REQUEST;
        }
        return;
    }
    if (s_state == T3_STATE_MIDDLE_REQUEST) {
        if (Task3_RequestBallTarget(T3_BALL_MIDDLE_TARGET_X_PX)) {
            s_state = T3_STATE_MIDDLE_APPROACH;
        }
        return;
    }
    if (s_state == T3_STATE_MIDDLE_APPROACH) {
        /*
         * 阶段二只需从左向右轻触经过阈值即可进入最终阶段，
         * 不等待钢球在阶段二目标点停稳，5 秒计时也不参与切段。
         */
        if ((ballStatus.targetPx == T3_BALL_MIDDLE_TARGET_X_PX) &&
            (ballStatus.measuredPx >= T3_BALL_MIDDLE_PASS_X_PX)) {
            s_finalGuardTriggered = false;
            s_state = T3_STATE_FINAL_REQUEST;
        }
        return;
    }
    if (s_state == T3_STATE_FINAL_REQUEST) {
        if (Task3_RequestBallTarget(T3_BALL_FINAL_TARGET_X_PX)) {
            s_state = T3_STATE_FINAL_HOLD;
        }
        return;
    }
    if (s_state == T3_STATE_FINAL_HOLD) {
        /*
         * 即使位置越过右侧保护线，也不失能 ID1；重新投递右侧目标，让
         * BALLCTRL 在下一帧继续把球拉回。每次右摆只重投递一次，避免刷队列。
         */
        if (!s_finalGuardTriggered &&
            (ballStatus.measuredPx >= T3_BALL_FINAL_GUARD_X_PX)) {
            s_finalGuardTriggered = Task3_RequestBallTarget(
                T3_BALL_FINAL_TARGET_X_PX);
        }
        if (Task3_IsBallHoldingTarget(&ballStatus, T3_BALL_FINAL_TARGET_X_PX)) {
            if (!s_finishBeeped) {
                s_finishBeeped = true;
                BspBuzzer_BeepShort();
                RobotCore_NotifyTaskFinished(T3_TASK_INDEX);
            }
            s_state = T3_STATE_FINISHED;
        }
        return;
    }
    if (s_state == T3_STATE_FINISHED) {
        return;
    }
}

void Task3_OnExit(void)
{
    /* K4 退出时安全停止并失能全部三个本题涉及的电机。 */
    AppBallControl_RequestStop();
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_LIFT);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
    vTaskDelay(pdMS_TO_TICKS(T3_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_LIFT, false);

    s_confirmRequested = false;
    s_finishBeeped = false;
    s_finalGuardTriggered = false;
    s_elapsedTicks = 0U;
    s_timerRunning = false;
    s_timerFinished = false;
    s_state = T3_STATE_WAIT_BALL_RELEASE;
}
