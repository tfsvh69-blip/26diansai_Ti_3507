#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "emm42_robot.h"

/* ==================================================================
 * 第 5 题：张大头 Emm42_V5.0 闭环步进电机【方向标定测试】（速度模式）
 *
 * 硬件：UART1（PA17=TX → 驱动器 RX，PB5=RX ← 驱动器 TX），115200 8N1，
 *       v1.1 排针 H7；3 台驱动器挂同一总线，靠设备地址（1/2/3）区分。
 *       地址 → 角色映射见 module/emm42/emm42_robot.h：
 *         1=摆杆高低调节(LIFT)、2=左轮(WHEEL_L)、3=右轮(WHEEL_R)。
 *
 * ⚠️ 本题当前用途 = 方向标定观察，不是最终业务动作：
 *    依次让 ID1 → ID2 → ID3【单独】以【正 RPM】转一段时间再停，一次只转一路，
 *    便于逐个肉眼确认"正 RPM 对应哪个物理方向"（摆杆是抬升还是下降、
 *    左/右轮是朝小车前进还是后退方向转）。三路测完后保持停车，不循环，
 *    等使用者观察记录后按 K4 退出。
 *
 *    这里的"正 RPM"会经过 Emm42Robot_SetSpeedRpm() 的角色方向标定后再下发：
 *    ID2 已直通、ID3 已取反、ID1 暂待确认。后续确认 ID1 方向时，只需改
 *    emm42_robot 层的标定表，不用改本文件。
 *
 * 使用方法：进入本题后静置观察，依次看到摆杆动一下、左轮转一下、右轮转
 * 一下（每路约 3 秒），记录每路的实际物理方向，反馈给开发者用于标定。
 *
 * 【关键设计】一次只有一路在转，帧节奏很宽松（每路只需 3 帧：使能/转/停），
 * 不存在"一个 OnLoop 里连发多帧"的问题，故不需要像并行测试那样按拍强制错开。
 *
 * 【上板判断】某路电机不转时的排查顺序：
 *   1. 量 PA17 是否有数据波形（有 = MCU 侧已发出）；
 *   2. Emm42_GetRxByteCount() > 0 说明总线上至少有驱动器在回话，链路通；
 *      恒为 0 → 查驱动器地址、波特率（默认 115200，见 ti_msp_dl_config.h）、TX/RX 是否接反；
 *   3. 驱动器电源 4S 是否供电（Emm42_V5.0 需 12~36V，仅接 3V3 逻辑电不会转）。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 可调参数
 * ------------------------------------------------------------------ */

/* 未接全 3 台时，把对应角色置 0 即可跳过（不测试该角色，直接跳到下一路）。 */
#define T5_ENABLE_LIFT      (1)
#define T5_ENABLE_WHEEL_L   (1)
#define T5_ENABLE_WHEEL_R   (1)

/* 测试转速（RPM）。先用低速看清方向，确认映射关系后再调整实际业务转速。 */
#define T5_TEST_RPM         (60)

/* 加速度档位：0=不使用曲线立即变速，数值越大加速越快。低速测试用温和值。 */
#define T5_ACC              (10U)

/* 使能命令后等待的 30ms 周期数：给首个被测驱动器留出就绪和处理命令的时间。 */
#define T5_ENABLE_SETTLE_TICKS  (10U)

/* 每路运行时长（30ms 轮询周期数）。100 拍 ≈ 3.0s，足够肉眼看清转动方向。 */
#define T5_RUN_TICKS        (100U)

/* 每路测完后的停车间歇（30ms 周期数）。34 拍 ≈ 1.0s，用于分隔"这路测完、下一路开始"。 */
#define T5_STOP_TICKS       (34U)

/* ------------------------------------------------------------------
 * 依次测试的角色顺序表：ID1→ID2→ID3，即 摆杆→左轮→右轮。
 * s_roleEnabled 对应上面的 T5_ENABLE_* 编译期开关，跳过未接的角色。
 * ------------------------------------------------------------------ */

static const Emm42RobotId_t s_roleOrder[3] = {
    EMM42_ROBOT_LIFT,      /* ID1 */
    EMM42_ROBOT_WHEEL_L,   /* ID2 */
    EMM42_ROBOT_WHEEL_R,   /* ID3 */
};

static const uint8_t s_roleEnabled[3] = {
    (uint8_t)T5_ENABLE_LIFT,
    (uint8_t)T5_ENABLE_WHEEL_L,
    (uint8_t)T5_ENABLE_WHEEL_R,
};

/* ------------------------------------------------------------------
 * 状态机：s_roleIdx(0~2)=当前测试到第几路，3=三路（或已启用的几路）测完；
 *         s_phase=当前路内部的四段小节：使能→就绪等待→正转→停止。
 * ------------------------------------------------------------------ */

typedef enum {
    T5_PHASE_ENABLE = 0,  /* 使能当前角色（一帧） */
    T5_PHASE_ENABLE_WAIT, /* 使能后等待驱动器处理命令 */
    T5_PHASE_RUN,         /* 以 +T5_TEST_RPM 正转 T5_RUN_TICKS 拍 */
    T5_PHASE_STOP         /* 停止该角色，间歇 T5_STOP_TICKS 拍后切下一角色 */
} Task5Phase_t;

static uint8_t      s_roleIdx;   /* 0~2 当前测试角色下标；3=全部测完 */
static Task5Phase_t s_phase;
static uint32_t     s_tick;

/* 从 from 开始找下一个已启用的角色下标；找不到返回 3（表示测完/无可测）。 */
static uint8_t Task5_NextEnabledRole(uint8_t from)
{
    uint8_t i = from;
    while ((i < 3U) && (s_roleEnabled[i] == 0U)) {
        i++;
    }
    return i;
}

const char *Task5_GetUiStatus(void)
{
    if (s_roleIdx >= 3U) {
        return "EMM DONE";
    }

    switch (s_phase) {
    case T5_PHASE_ENABLE:
        return (s_roleIdx == 0U) ? "ID1 EN" :
               (s_roleIdx == 1U) ? "ID2 EN" : "ID3 EN";

    case T5_PHASE_ENABLE_WAIT:
        return (s_roleIdx == 0U) ? "ID1 WAIT" :
               (s_roleIdx == 1U) ? "ID2 WAIT" : "ID3 WAIT";

    case T5_PHASE_RUN:
        return (s_roleIdx == 0U) ? "ID1 RUN" :
               (s_roleIdx == 1U) ? "ID2 RUN" : "ID3 RUN";

    case T5_PHASE_STOP:
        return (s_roleIdx == 0U) ? "ID1 STOP" :
               (s_roleIdx == 1U) ? "ID2 STOP" : "ID3 STOP";

    default:
        return "EMM ERR";
    }
}

void Task5_OnEnter(void)
{
    /* 进入本题即从第一个已启用的角色开始，不预先批量使能——按小节逐帧下发。 */
    s_roleIdx = Task5_NextEnabledRole(0U);
    s_phase   = T5_PHASE_ENABLE;
    s_tick    = 0U;
}

void Task5_OnLoop(void)
{
    if (s_roleIdx >= 3U) {
        /* 三路（或已启用的几路）均测试完毕：保持停车，等待观察记录后 K4 退出。 */
        return;
    }

    switch (s_phase) {
    case T5_PHASE_ENABLE:
        Emm42Robot_Enable(s_roleOrder[s_roleIdx], true);
        /* 使能和速度命令至少间隔 300ms，避免首路 ID1 尚未就绪就收到转速帧。 */
        s_phase = T5_PHASE_ENABLE_WAIT;
        s_tick  = 0U;
        break;

    case T5_PHASE_ENABLE_WAIT:
        s_tick++;
        if (s_tick >= T5_ENABLE_SETTLE_TICKS) {
            s_phase = T5_PHASE_RUN;
            s_tick  = 0U;
        }
        break;

    case T5_PHASE_RUN:
        if (s_tick == 0U) {
            /* 只需下发一次：驱动器收到速度命令后会持续转，不用每拍重发。 */
            Emm42Robot_SetSpeedRpm(s_roleOrder[s_roleIdx], (int16_t)T5_TEST_RPM, T5_ACC);
        }
        s_tick++;
        if (s_tick >= T5_RUN_TICKS) {
            s_phase = T5_PHASE_STOP;
            s_tick  = 0U;
        }
        break;

    case T5_PHASE_STOP:
        if (s_tick == 0U) {
            Emm42Robot_Stop(s_roleOrder[s_roleIdx]);
        }
        s_tick++;
        if (s_tick >= T5_STOP_TICKS) {
            s_roleIdx = Task5_NextEnabledRole((uint8_t)(s_roleIdx + 1U));
            s_phase   = T5_PHASE_ENABLE;
            s_tick    = 0U;
        }
        break;

    default:
        break;
    }
}

void Task5_OnExit(void)
{
    /*
     * 退出本题：只下发急停帧（3 台背靠背，Emm42Robot_StopAll 内部实现），保证电机立刻停住。
     * 这里【不发失能帧】——闭环驱动器停车后保持力矩，比失力更安全（不会溜车/摆杆下坠）；
     * 且退出是一次性调用、无法像 OnLoop 那样按拍错开，帧数越少越可靠。
     * 下次进入本题会从 ID1 重新开始测试序列。
     */
    Emm42Robot_StopAll();

    s_roleIdx = 0U;
    s_phase   = T5_PHASE_ENABLE;
    s_tick    = 0U;
}
