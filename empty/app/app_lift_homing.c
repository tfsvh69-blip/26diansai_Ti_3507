#include "app_lift_homing.h"

#include <stdbool.h>
#include <stdint.h>

#include "Delay.h"
#include "bsp_home_switch.h"
#include "bsp_uart.h"
#include "emm42_robot.h"

/* ==================================================================
 * 开机 ID1（摆杆曲柄摇杆）自动归零
 *
 * 硬件：P1 接口（原继电器接口）已改接一颗轻触开关到 PA24，一端接地，按下
 * 时限位开关被压下（BspHomeSwitch_IsPressed() 返回 true）。
 *
 * 2026-08 机构由「丝杆升降」改为「电机直驱曲柄摇杆」后重写流程：
 *   1. 【下降】（负方向）移动，直到压下限位开关——这就是物理归零参考点；
 *   2. 急停；
 *   3. 【抬升】（正方向）移动 LIFT_HOMING_LEVEL_OFFSET_PULSES 个脉冲，
 *      到达摆杆水平位置（替代此前"人工把杆摆平"的做法）。
 * 极端情况：若开机时开关已被按下（上次断电恰好停在开关位置或更低），跳过
 * 第 1 步的下降逼近（避免继续往下顶硬限位），直接抬升。
 *
 * 方向依据（2026-08 题目六 900 脉冲实测）：ID1 正脉冲 = 抬升曲柄摇杆。
 *
 * 归零终点的意义：本流程结束后 ID1 停在【摆杆水平】处，BALLCTRL 首次启动时
 * 会把这个位置清零为绝对位置原点（hasZeroedSinceBoot），因此各题目的
 * LEVEL_TRIM_PULSE 才能以 0 为基准。offset 没调准 = 所有题目的水平点都偏。
 *
 * 本函数只应在调度器启动前（App_Init 内，Emm42Robot_Init() 之后）调用一次，
 * 全程用 Delay_ms 忙等轮询开关，与 App_Emm42BootDisableAll() 写法一致；
 * 此时还没有任何任务能与本函数争抢 ID1。
 *
 * ⚠️ 故意不加超时保护：若限位开关故障或没装好导致第 1 步永远读不到触发，
 * 本函数会一直忙等，调度器不会启动（LED1 不闪、OLED 不亮），摆杆会持续
 * 往下顶到机械死点。这是归零流程本身的含义——找不到物理参考点就不能继续；
 * 首次上电测试时请守在电源开关旁，异常立刻断电。
 * ================================================================== */

/* ---- 运动参数（本模块私有，不与其它任务共享；纯可调，不叠加软件限幅） ---- */
#define LIFT_HOMING_SEEK_RPM            (5)     /* 下降逼近限位开关的速度；越慢触发点越精确 */
#define LIFT_HOMING_LIFT_RPM            (5U)    /* 触发后抬升到水平位置的速度 */
#define LIFT_HOMING_ACC                 (0U)     /* 加速度档位，0=不用曲线直接按设定速度跑 */
#define LIFT_HOMING_POLL_MS             (5U)     /* 轮询限位开关的间隔 */
#define LIFT_HOMING_STOP_SETTLE_MS      (20U)    /* 触发急停后的稳定等待，再下发抬升命令 */
#define LIFT_HOMING_CLOG_CLEAR_SETTLE_MS (20U)   /* 解堵转保护后的稳定等待 */
#define LIFT_HOMING_ENABLE_SETTLE_MS    (180U)   /* 使能后的稳定等待，沿用 task2/task6 已实测数据 */

/*
 * 【由使用者指定】限位开关触发点 → 摆杆水平位置的抬升脉冲数（正方向，正数）。
 *
 * 换算：3200 脉冲 = 电机转一整圈 = 360°，即 1° ≈ 8.9 脉冲。
 * 当前值 711 脉冲 ≈ 80°，来自"撞到开关后大约要抬 80°"的目测估计，**不是实测值**，
 * 首次上电必须守在电源旁，看实际停位再修正：
 *   停得比水平【低】→ 调大；停得比水平【高】→ 调小。
 * 改这一个数即可，方向和流程不用动。
 */
#define LIFT_HOMING_LEVEL_OFFSET_PULSES (205)

/* ---- 机械/协议换算（与 task6 各自独立维护同一常量，不共享） ---- */
#define LIFT_HOMING_PULSES_PER_REV      (3200U)  /* 16 细分 = 3200 脉冲/圈，须与驱动器 MStep 一致 */
#define LIFT_HOMING_MOVE_MARGIN_PCT     (30U)    /* 理论运动时间的余量百分比 */

/*
 * 位置模式命令是"发出即规划"，必须等够理论运动时间才能发下一条，否则会被
 * 覆盖导致"发了却没走到位"（task6 已验证的坑）。这里现算等待时间。
 */
static uint32_t AppLiftHoming_MoveWaitMs(int32_t pulses, uint32_t rpm)
{
    uint32_t absPulses  = (uint32_t)((pulses < 0) ? -pulses : pulses);
    uint32_t moveTimeMs = (60000UL * absPulses) /
                          (LIFT_HOMING_PULSES_PER_REV * rpm);

    return (moveTimeMs * (100U + LIFT_HOMING_MOVE_MARGIN_PCT)) / 100U;
}

void AppLiftHoming_RunAtBoot(void)
{
    bool pressedAtStart;

    BspUart0_SendString("HOMING: ID1 start\r\n");

    /* 清一次可能残留的堵转保护，再使能 ID1（App_Emm42BootDisableAll 已把它失能）。 */
    Emm42Robot_ClearClogProtection(EMM42_ROBOT_LIFT);
    Delay_ms(LIFT_HOMING_CLOG_CLEAR_SETTLE_MS);
    Emm42Robot_Enable(EMM42_ROBOT_LIFT, true);
    Delay_ms(LIFT_HOMING_ENABLE_SETTLE_MS);

    pressedAtStart = BspHomeSwitch_IsPressed();

    if (!pressedAtStart) {
        /* 下降（负方向）逼近限位开关，直到压下——压下点即物理归零参考点。 */
        BspUart0_SendString("HOMING: lowering to switch\r\n");
        Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT, (int16_t)-LIFT_HOMING_SEEK_RPM, LIFT_HOMING_ACC);
        while (!BspHomeSwitch_IsPressed()) {
            Delay_ms(LIFT_HOMING_POLL_MS);
        }
        Emm42Robot_Stop(EMM42_ROBOT_LIFT);
        Delay_ms(LIFT_HOMING_STOP_SETTLE_MS);
    } else {
        /* 极端情况：开机时开关已被压下，不能再往下顶，直接抬升。 */
        BspUart0_SendString("HOMING: switch pre-pressed, lifting\r\n");
    }

    /* 从触发点抬升（正方向）到摆杆水平位置。 */
    {
        int32_t  offsetPulses = (int32_t)LIFT_HOMING_LEVEL_OFFSET_PULSES;
        uint32_t waitMs;

        Emm42Robot_MoveRelative(EMM42_ROBOT_LIFT, offsetPulses,
                                 LIFT_HOMING_LIFT_RPM, LIFT_HOMING_ACC);
        waitMs = AppLiftHoming_MoveWaitMs(offsetPulses, LIFT_HOMING_LIFT_RPM);
        Delay_ms(waitMs);
    }

    BspUart0_SendString("HOMING: ID1 done\r\n");
}
