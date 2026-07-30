#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "bsp_buzzer.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 2 题：8 路灰度循迹 PID
 *
 * 硬件物理左→右为 LINE8→LINE1，而 BspLine_ReadAll() 的位图固定为
 * bit7=LINE8 … bit0=LINE1。本题按物理左右顺序做加权平均：
 * 线偏右时误差为正 → 左轮加速、右轮减速 → 小车向右修正。
 *
 * 左右轮由 UART1 上的 Emm42 闭环控制器驱动：ID2=左轮、ID3=右轮；ID1 摆杆
 * 不参与本题。为避免同一拍在总线上连续下发两帧，左右速度命令交替下发。
 * PID 算出的目标 RPM 每拍直接下发，任务层不再叠加软件斜坡（曾经加过，会让
 * 响应变慢）；平滑改由控制器内置曲线加减速档位 T2_EMM_ACC 承担——acc=0 时
 * 每次变速都是阶跃，实测车身会突兀抖动，故改用非 0 档位（说明书 §6.3.1：
 * 档位越大曲线越陡，t2-t1=(256-acc)*50us 每步变化 1RPM），具体数值按需微调。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 可调参数：首次上车建议先只调 T2_BASE_RPM、T2_KP，再逐步加入 Ki/Kd。
 * ------------------------------------------------------------------ */

/* PID 增益。误差单位为加权位置 -7、-5、-3、-1、+1、+3、+5、+7。 */
#define T2_KP                      (5.0F)
#define T2_KI                      (0.15F)
#define T2_KD                      (0.2F)

/* PID 积分与转向输出限幅，单位分别为误差累计值和 RPM。 */
#define T2_INTEGRAL_LIMIT          (20.0F)
#define T2_MAX_STEER_RPM           (100.0F)

/*
 * 基础前进速度与单轮安全范围，正 RPM 为小车前进。
 * T2_MAX_WHEEL_RPM 必须 >= T2_BASE_RPM，否则直道会被此上限硬顶住、
 * T2_BASE_RPM 设的值不生效；还应留出 >= 2*T2_MAX_STEER_RPM 的总跨度
 * （即 T2_MAX_WHEEL_RPM - T2_MIN_WHEEL_RPM），否则急弯时 Task2_ClampWheelPair
 * 仍会被迫压缩差速。当前 200 + 2*100 的组合下取 230，按此换算的实车速度、
 * 打滑/失控风险需要上车实测确认，偏快就调低。
 */
#define T2_BASE_RPM                (130.0F)
#define T2_MIN_WHEEL_RPM           (5.0F)
#define T2_MAX_WHEEL_RPM           (230.0F)

/*
 * Emm42 速度命令的加速度档位（0~255，越大曲线越陡/越接近立即生效）。
 * acc=0 实测车身会突兀抖动，改用非 0 档位让驱动器内部把每次变速摊开成
 * 短暂曲线，减少冲击；数值越小越平滑但越滞后，按需微调。
 */
#define T2_EMM_ACC                 (150U)

/* UIMENU 的固定控制周期。 */
#define T2_DT_SEC                  (0.03F)

/* 连续丢线达到此周期数后，将目标速度降为 0；10 拍约 300ms。 */
#define T2_LINE_LOST_TICKS         (20U)

/*
 * 复位（失能）后等待再开始使能流程；两路分别使能后也各等待再继续。
 * 两个延时性质不同：T2_ENABLE_SETTLE_TICKS 已实测过——3 拍(90ms) 会有约一半
 * 概率使能不生效（驱动器来不及处理），6 拍(180ms) 才稳定，这里不再往下压；
 * T2_RESET_SETTLE_TICKS（失能后的等待）目前没有实测证据证明需要多长，是偏
 * 保守加的，所以优先从这个上面压缩入场总延时。当前 2+6+6=14 拍≈420ms，
 * 如果还想更快，只能继续压 ENABLE 那一半，但有重新触发上面那个概率性
 * 不转问题的风险，压之前务必多测几次确认没有复现。
 */
#define T2_RESET_SETTLE_TICKS      (2U)     /* 失能后等待，约 60ms */
#define T2_ENABLE_SETTLE_TICKS     (6U)     /* 使能后等待，约 180ms（已验证不能再低） */

/*
 * UART1 为共享总线（多台驱动器 TX 并联，见 emm42_robot.h 电气风险说明），
 * emm42_v5.h 协议层明确要求"连续下发多帧时调用方须自行留间隔"。终点硬停车
 * 和 K4 退出这两处需要在一次调用里背靠背给左右两轮分别下发命令，用短暂阻塞
 * 延时留出间隔，避免两路驱动器的回复在共享总线上互相干扰导致丢帧。
 */
#define T2_EMM_CMD_GAP_MS          (6U)

/*
 * 误差一阶低通滤波系数（0~1，越小滤波越强但越滞后）。
 * 8 路循迹在线偏移时命中路数只有 1~3 路，误差随命中传感器切换而台阶式跳变，
 * PID 直接对台阶求导会放大成尖峰转向指令，是震荡的主要来源之一；
 * 先对误差做低通再喂给 PID，能显著抑制这种由离散传感器量化带来的抖动。
 */
#define T2_ERROR_FILTER_ALPHA      (0.5F)

/*
 * 误差死区：|滤波后误差| 小于此值时直接当成 0 喂给 PID，不产生转向修正。
 * 车身接近赛道中心时，命中路数经常在相邻两档（如 2 路和 3 路）来回跳变
 * ——两档对应的加权误差通常只差 1~2（如 0 与 ±1），本不是真实的位置
 * 偏差，却会被 PID（尤其是微分项）当成扰动持续修正，表现为小车在中心
 * 附近不停小幅左右摆动。设死区后这种量级的抖动会被直接忽略，只有真正
 * 偏出死区的位置误差才会触发转向。
 */
#define T2_ERROR_DEADBAND          (1.0F)

/*
 * 转弯减速：按 |转向输出| 的比例降低基础前进速度，直道全速、弯道自动放慢，
 * 可在不加剧震荡的前提下提高整体平均速度（弯道更稳后可尝试调高 T2_BASE_RPM）。
 */
#define T2_CORNER_SLOWDOWN_GAIN    (0.6F)   /* 每 1 RPM 转向量对应降低的基础速度 RPM */
#define T2_MIN_BASE_RPM            (10.0F)  /* 基础速度下限，避免急弯把基础速度压到过低 */

/*
 * 终点判定：命中路数达到 6 路及以上（含 7、8 路）视为压到终点宽线。为避免
 * 出发瞬间（仍压在宽出发线上）被立即误判为"已完成一圈"，仍需先连续检测到
 * "细线正常循迹"一段时间（确认已离开出发线）才允许武装终点检测，武装后
 * 再连续命中宽线才判定到达终点。
 */
#define T2_FINISH_ARM_HIT_MAX      (3U)     /* 命中路数 <= 此值视为正常细线 */
#define T2_FINISH_ARM_TICKS        (15U)    /* 连续满足以上条件的周期数，约 450ms，确认已离开出发线 */
#define T2_FINISH_HIT_MIN          (6U)     /* 命中路数 >= 此值视为压到终点宽线 */
#define T2_FINISH_HIT_TICKS        (1U)     /* 连续满足以上条件的周期数，约 30ms 消抖，避免单拍噪声误停 */

/* 每个 OnLoop 周期对应的毫秒数，与 T2_DT_SEC 一致，供秒表换算用。 */
#define T2_TICK_MS                 (30U)

/*
 * 秒表校准系数：实测显示计时比真实时间快，怀疑是 FreeRTOS tick 依赖的主频跟
 * 工程假设的 80MHz 有偏差（根因未定位，见与用户的相关讨论），先用系数硬补偿。
 * 历次实测：第一次显示16.0s/实测15.0s(比例0.9375)；应用该系数后再跑一圈，
 * 显示16.3s/实测15.6s，说明还偏快，在上一次系数基础上再乘以(15.6/16.3)
 * 校正：0.9375 * (15.6/16.3) ≈ 0.897。如果后续继续偏，按同样方法在当前
 * 系数上再乘以(最新实测秒数/最新显示秒数)得到新系数，不用每次都从1.0推倒重来。
 */
#define T2_STOPWATCH_CAL_SCALE     (0.897F)

/*
 * 定时减速：按秒表校准后的时间（跟 OLED 显示的一致），跑到第
 * T2_DECEL_START_MS 起基础速度开始线性下降，直到降到 T2_DECEL_MIN_RPM 为止；
 * 减速后的基础速度会跟转弯减速（T2_CORNER_SLOWDOWN_GAIN 算出的那个）取
 * 更小值生效，两者不冲突、谁更保守听谁的。跑一圈约 15.3s，14.5s 起降留
 * 出约 0.8s 的减速窗口；如果车还没到终点就一直按这个斜率往下降，相当于
 * 给"跑得比预期久"的情况兜底，不会一直全速冲。
 *
 * 两个可调参数：
 *   T2_DECEL_GRADIENT_RPM_PER_SEC 越大，同样时间里基础速度降得越多、减速
 *   越"陡"；T2_DECEL_MIN_RPM 是能降到的最低基础速度，不会再往下减。
 */
#define T2_DECEL_START_MS             (14500U)  /* 触发减速的秒表时间(ms)，固定值 */
#define T2_DECEL_GRADIENT_RPM_PER_SEC (60.0F)    /* 梯度系数：每秒降低的基础速度(RPM) */
#define T2_DECEL_MIN_RPM              (10.0F)    /* 线性减速能降到的最低基础速度(RPM) */

typedef enum {
    T2_STATE_RESET_DISABLE = 0,  /* 复位：失能 ID2/ID3，清掉上一次残留状态 */
    T2_STATE_RESET_WAIT,          /* 失能后等待驱动器处理 */
    T2_STATE_ENABLE_LEFT,
    T2_STATE_ENABLE_LEFT_WAIT,
    T2_STATE_ENABLE_RIGHT,
    T2_STATE_ENABLE_RIGHT_WAIT,
    T2_STATE_RUN,      /* 正常循迹或短时丢线保持 */
    T2_STATE_STOP,     /* 长时间丢线，平滑停车，重新识别到线后自动恢复 */
    T2_STATE_FINISHED  /* 已检测到终点线，硬停车锁定，等待 K4 手动退出 */
} Task2State_t;

static Task2State_t s_state;
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
static bool         s_finishArmed;      /* 是否已确认离开出发线，开始监测终点 */
static uint32_t     s_armTicks;         /* 连续细线周期计数 */
static uint32_t     s_finishHitTicks;   /* 连续宽线（终点线）周期计数 */
static uint32_t     s_elapsedTicks;     /* 秒表计时：进入本题起累计的 OnLoop 拍数，到终点后停止累加 */
static char         s_uiStatusBuf[16];  /* Task2_GetUiStatus() 返回的秒表文本缓冲 */

static float Task2_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

/*
 * 左右轮目标 RPM 的安全限幅：整体平移以保住差速（转向权限），而不是对左右
 * 两轮分别独立夹到 [MIN,MAX]。独立夹会在 T2_BASE_RPM 已接近或超过
 * T2_MAX_WHEEL_RPM 时把外侧轮压扁到上限、内侧轮却不受影响，导致实际下发的
 * 差速远小于 PID 算出的转向量——高速直道正常、一到弯道差速被"吃掉"、
 * 转不过来冲出赛道，正是本题的典型症状。整体平移能在夹到上/下限的同时
 * 保留左右轮的差值，只有差速本身超过 [MIN,MAX] 的跨度时才会被迫压缩。
 */
static void Task2_ClampWheelPair(float *leftRpm, float *rightRpm)
{
    float hi = (*leftRpm > *rightRpm) ? *leftRpm : *rightRpm;
    float lo;
    float shift;

    if (hi > T2_MAX_WHEEL_RPM) {
        shift      = hi - T2_MAX_WHEEL_RPM;
        *leftRpm  -= shift;
        *rightRpm -= shift;
    }

    lo = (*leftRpm < *rightRpm) ? *leftRpm : *rightRpm;
    if (lo < T2_MIN_WHEEL_RPM) {
        shift      = T2_MIN_WHEEL_RPM - lo;
        *leftRpm  += shift;
        *rightRpm += shift;
    }

    /* 兜底：差速跨度超过 [MIN,MAX] 时上面两步无法同时满足，这里强制压到安全范围。 */
    *leftRpm  = Task2_Clamp(*leftRpm,  T2_MIN_WHEEL_RPM, T2_MAX_WHEEL_RPM);
    *rightRpm = Task2_Clamp(*rightRpm, T2_MIN_WHEEL_RPM, T2_MAX_WHEEL_RPM);
}

/*
 * 将浮点轮速四舍五入为 RPM 整数。UART1 为共享总线，每拍至多发送一帧，
 * 左右轮交替更新，避免两帧背靠背造成驱动器漏收。
 */
static void Task2_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t leftInt = (leftRpm >= 0.0F) ? (int32_t)(leftRpm + 0.5F)
                                        : (int32_t)(leftRpm - 0.5F);
    int32_t rightInt = (rightRpm >= 0.0F) ? (int32_t)(rightRpm + 0.5F)
                                          : (int32_t)(rightRpm - 0.5F);

    if (s_sendLeftNext) {
        if (leftInt != s_sentLeftRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_L, (int16_t)leftInt,
                                   T2_EMM_ACC);
            s_sentLeftRpm = leftInt;
        }
    } else {
        if (rightInt != s_sentRightRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_R, (int16_t)rightInt,
                                   T2_EMM_ACC);
            s_sentRightRpm = rightInt;
        }
    }
    s_sendLeftNext = !s_sendLeftNext;
}

/*
 * 读取 8 路并算出线相对车体中心的位置。
 * 物理从左至右的权重为 -7、-5、-3、-1、+1、+3、+5、+7；
 * hitCount 输出实际命中路数（供丢线保护和终点判定使用），返回 false 表示 8 路均未识别到线。
 */
static bool Task2_GetLineError(float *error, uint32_t *hitCountOut)
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

/*
 * 秒表校准后的累计毫秒数，跟 OLED 上显示的时间是同一套换算（供定时减速复用，
 * 避免和 Task2_GetUiStatus 各算一遍不一致）。
 */
static uint32_t Task2_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T2_TICK_MS) * T2_STOPWATCH_CAL_SCALE);
}

/*
 * 秒表文本："T:12.3s"，到终点后追加" DONE"并定格（s_elapsedTicks 已停止累加）。
 * 供 UIMENU 运行界面周期性显示，OLED 只能 ASCII，手写十进制转换，不用 sprintf。
 */
const char *Task2_GetUiStatus(void)
{
    uint32_t totalMs  = Task2_GetElapsedMs();
    uint32_t secWhole = totalMs / 1000U;
    uint32_t tenths   = (totalMs / 100U) % 10U;
    uint32_t idx      = 0U;
    char     digits[10];
    uint32_t n        = 0U;
    uint32_t v         = secWhole;

    s_uiStatusBuf[idx++] = 'T';
    s_uiStatusBuf[idx++] = ':';

    if (v == 0U) {
        s_uiStatusBuf[idx++] = '0';
    } else {
        while (v > 0U) {
            digits[n++] = (char)('0' + (v % 10U));
            v /= 10U;
        }
        while (n > 0U) {
            s_uiStatusBuf[idx++] = digits[--n];
        }
    }

    s_uiStatusBuf[idx++] = '.';
    s_uiStatusBuf[idx++] = (char)('0' + tenths);
    s_uiStatusBuf[idx++] = 's';

    if (s_state == T2_STATE_FINISHED) {
        s_uiStatusBuf[idx++] = ' ';
        s_uiStatusBuf[idx++] = 'D';
        s_uiStatusBuf[idx++] = 'O';
        s_uiStatusBuf[idx++] = 'N';
        s_uiStatusBuf[idx++] = 'E';
    }
    s_uiStatusBuf[idx] = '\0';

    return s_uiStatusBuf;
}

void Task2_OnEnter(void)
{
    /* 复位失能放进 OnLoop 状态机（T2_STATE_RESET_DISABLE）里做，跟使能一样
     * 走"发命令→等待拍数"的节拍化流程，而不是在这里同步阻塞太久；
     * OnEnter 只负责复位变量，不直接下发 Emm42 命令。 */
    Pid_Init(&s_pid, T2_KP, T2_KI, T2_KD,
             T2_INTEGRAL_LIMIT, T2_MAX_STEER_RPM);
    s_state          = T2_STATE_RESET_DISABLE;
    s_lastSteerRpm   = 0.0F;
    s_leftRpm        = 0.0F;
    s_rightRpm       = 0.0F;
    s_filteredError  = 0.0F;
    s_sentLeftRpm    = 0;
    s_sentRightRpm   = 0;
    s_lineLostTicks  = 0U;
    s_enableSettleTicks = 0U;
    s_sendLeftNext   = true;
    s_finishArmed    = false;
    s_armTicks       = 0U;
    s_finishHitTicks = 0U;
    s_elapsedTicks   = 0U;
}

void Task2_OnLoop(void)
{
    float    rawError;
    float    targetLeftRpm;
    float    targetRightRpm;
    uint32_t hitCount;

    /* 秒表：从进入本题起每拍累加，到终点后不再累加（定格显示）。 */
    if (s_state != T2_STATE_FINISHED) {
        s_elapsedTicks++;
    }

    if (s_state == T2_STATE_RESET_DISABLE) {
        /* 复位：先失能 ID2/ID3，清掉上一次残留的速度/使能状态。两帧之间留
         * T2_EMM_CMD_GAP_MS 间隔，避免共享总线互相干扰（同 OnExit 的做法）。 */
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        vTaskDelay(pdMS_TO_TICKS(T2_EMM_CMD_GAP_MS));
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_enableSettleTicks = T2_RESET_SETTLE_TICKS;
        s_state = T2_STATE_RESET_WAIT;
        return;
    }

    if (s_state == T2_STATE_RESET_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T2_STATE_ENABLE_LEFT;
        return;
    }

    if (s_state == T2_STATE_ENABLE_LEFT) {
        /* ID2=左轮：先使能并等待驱动器处理使能帧。 */
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_enableSettleTicks = T2_ENABLE_SETTLE_TICKS;
        s_state = T2_STATE_ENABLE_LEFT_WAIT;
        return;
    }

    if (s_state == T2_STATE_ENABLE_LEFT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T2_STATE_ENABLE_RIGHT;
        return;
    }

    if (s_state == T2_STATE_ENABLE_RIGHT) {
        /* ID3=右轮：同样等待，规避已观察到的使能后立即发速度帧问题。 */
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_enableSettleTicks = T2_ENABLE_SETTLE_TICKS;
        s_state = T2_STATE_ENABLE_RIGHT_WAIT;
        return;
    }

    if (s_state == T2_STATE_ENABLE_RIGHT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T2_STATE_RUN;
    }

    if (s_state == T2_STATE_FINISHED) {
        /* 已到终点：停车帧只需在刚判定到达的那一拍发送一次（见下方），
         * 之后每拍直接返回，不再重复读传感器/跑 PID，等待 K4 手动退出。 */
        return;
    }

    if (Task2_GetLineError(&rawError, &hitCount)) {
        /* 从非 RUN 态恢复（首次进入 RUN 或丢线后重新压线）时，滤波器直接跳到
         * 当前值，避免继续对停车期间的陈旧值做低通造成恢复瞬间的滞后阶跃。 */
        if (s_state != T2_STATE_RUN) {
            s_filteredError = rawError;
        } else {
            s_filteredError += T2_ERROR_FILTER_ALPHA * (rawError - s_filteredError);
        }

        /* 重新识别到线后立即恢复闭环，丢线期间的积分不带入。 */
        if (s_state == T2_STATE_STOP) {
            Pid_Reset(&s_pid);
        }
        s_state         = T2_STATE_RUN;
        s_lineLostTicks = 0U;
        {
            /* 死区：接近中心的量化跳变误差不进 PID，避免中心附近来回小幅修正。 */
            float pidError = s_filteredError;
            if ((pidError > -T2_ERROR_DEADBAND) && (pidError < T2_ERROR_DEADBAND)) {
                pidError = 0.0F;
            }
            s_lastSteerRpm = Pid_Update(&s_pid, pidError, T2_DT_SEC);
        }
    } else {
        /* 短时漏检时保持上一次转向，避免单个采样空洞让小车立即急停。 */
        s_lineLostTicks++;
        if (s_lineLostTicks >= T2_LINE_LOST_TICKS) {
            /* 长时间丢线：停止而非盲目搜索，重新压线后会自动恢复。 */
            s_state = T2_STATE_STOP;
        }
    }

    /* 终点判定：先确认已离开出发线（细线持续 T2_FINISH_ARM_TICKS 拍），
     * 武装后再连续 T2_FINISH_HIT_TICKS 拍命中路数 >= T2_FINISH_HIT_MIN(6)
     * 才判定到达终点，避免出发瞬间仍压在宽线上被立即误判为"跑完一圈"。 */
    if (!s_finishArmed) {
        if (hitCount <= T2_FINISH_ARM_HIT_MAX) {
            s_armTicks++;
            if (s_armTicks >= T2_FINISH_ARM_TICKS) {
                s_finishArmed = true;
            }
        } else {
            s_armTicks = 0U;
        }
    } else if (s_state != T2_STATE_FINISHED) {
        if (hitCount >= T2_FINISH_HIT_MIN) {
            s_finishHitTicks++;
            if (s_finishHitTicks >= T2_FINISH_HIT_TICKS) {
                s_state = T2_STATE_FINISHED;
            }
        } else {
            s_finishHitTicks = 0U;
        }
    }

    if (s_state == T2_STATE_FINISHED) {
        /* 到达终点：硬停车锁定（不再重新进入 RUN），等待用户按 K4 手动退出。
         * 左右两轮此刻通常都在跑，两帧之间留 5ms 间隔，避免共享总线互相干扰。 */
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
        vTaskDelay(pdMS_TO_TICKS(T2_EMM_CMD_GAP_MS));
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
        s_leftRpm  = 0.0F;
        s_rightRpm = 0.0F;
        /* 自动到达终点时与按键共用同一短促提示音。 */
        BspBuzzer_BeepShort();
        RobotCore_NotifyTaskFinished(1U);
        return;
    }

    if (s_state == T2_STATE_RUN) {
        /* 转弯减速：转向输出越大，基础速度越低，直道快、弯道稳。 */
        float steerAbs   = (s_lastSteerRpm >= 0.0F) ? s_lastSteerRpm : -s_lastSteerRpm;
        float dynBaseRpm = T2_BASE_RPM - T2_CORNER_SLOWDOWN_GAIN * steerAbs;
        dynBaseRpm = Task2_Clamp(dynBaseRpm, T2_MIN_BASE_RPM, T2_BASE_RPM);

        /* 定时减速：秒表时间到 T2_DECEL_START_MS 后基础速度开始线性下降，
         * 跟转弯减速取更小值生效（谁更保守听谁的），不会互相抵消。 */
        {
            uint32_t elapsedMs = Task2_GetElapsedMs();
            if (elapsedMs >= T2_DECEL_START_MS) {
                float decelSec     = (float)(elapsedMs - T2_DECEL_START_MS) * 0.001F;
                float decelBaseRpm = T2_BASE_RPM - T2_DECEL_GRADIENT_RPM_PER_SEC * decelSec;
                decelBaseRpm = Task2_Clamp(decelBaseRpm, T2_DECEL_MIN_RPM, T2_BASE_RPM);
                if (decelBaseRpm < dynBaseRpm) {
                    dynBaseRpm = decelBaseRpm;
                }
            }
        }

        targetLeftRpm  = dynBaseRpm + s_lastSteerRpm;
        targetRightRpm = dynBaseRpm - s_lastSteerRpm;
        Task2_ClampWheelPair(&targetLeftRpm, &targetRightRpm);
    } else {
        targetLeftRpm  = 0.0F;
        targetRightRpm = 0.0F;
    }

    /* 目标 RPM 直接下发，不经任务层软件斜坡；平滑交给 T2_EMM_ACC 曲线档位。 */
    s_leftRpm  = targetLeftRpm;
    s_rightRpm = targetRightRpm;
    Task2_ApplyWheelRpm(s_leftRpm, s_rightRpm);
}

void Task2_OnExit(void)
{
    /* K4 退出时对 ID2/ID3 先急停再失能（轮子无重力负载，失能不会溜车/下坠，
     * 比保持力矩更省电更安全）；ID1 摆杆不属于本题，不发送任何命令。四帧背靠背
     * 下发容易在共享总线上互相干扰导致驱动器丢帧（emm42_v5.h 协议层明确要求
     * 连续下发需自行留间隔），每帧之间留 5ms 间隔。 */
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T2_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T2_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T2_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);

    Pid_Reset(&s_pid);
    s_state          = T2_STATE_STOP;
    s_finishArmed    = false;
    s_armTicks       = 0U;
    s_finishHitTicks = 0U;
}
