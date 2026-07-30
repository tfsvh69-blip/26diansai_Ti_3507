#include "app_lift_homing.h"

#include <stdbool.h>
#include <stdint.h>

#include "Delay.h"
#include "bsp_home_switch.h"
#include "bsp_uart.h"
#include "emm42_robot.h"

/* ==================================================================
 * 开机 ID1（摆杆升降丝杆）自动归零
 *
 * 硬件：P1 接口（原继电器接口）已改接一颗轻触开关到 PA24，一端接地，按下
 * 时限位开关被压下（BspHomeSwitch_IsPressed() 返回 true）。归零流程：
 *   1. 正方向移动，直到压下限位开关；
 *   2. 压下的瞬间反向（负方向）退让，直到开关释放——释放点即物理归零参考点；
 *   3. 继续往负方向移动 LIFT_HOMING_TARGET_OFFSET_PULSES 个脉冲，到达
 *      指定的相对工作位置（替代此前"人工把杆摆平"的做法）。
 * 极端情况：若开机时开关已被按下（比如上次断电时恰好停在开关位置），跳过
 * 第 1 步的正方向逼近（避免继续顶向硬限位），直接进入第 2 步的退让。
 *
 * 本函数只应在调度器启动前（App_Init 内，Emm42Robot_Init() 之后）调用一次，
 * 全程用 Delay_ms 忙等轮询开关，与 App_Emm42BootDisableAll() 写法一致；
 * 此时还没有任何任务能与本函数争抢 ID1。
 *
 * ⚠️ 故意不加超时保护：若限位开关故障导致第 1 步永远读不到触发，本函数会
 * 一直忙等，调度器不会启动（LED1 不闪、OLED 不亮）。这是归零流程本身的
 * 含义——找不到物理参考点就不能继续，先不要为一个理论故障场景加时间兜底。
 * ================================================================== */

/* ---- 运动参数（本模块私有，不与其它任务共享；纯可调，不叠加软件限幅） ---- */
#define LIFT_HOMING_APPROACH_RPM        (80)     /* 正方向逼近限位开关的速度 */
#define LIFT_HOMING_BACKOFF_RPM         (80)     /* 压下瞬间反向退让的速度，比逼近速度慢，退让停止点更精确 */
#define LIFT_HOMING_MOVE_RPM            (100U)    /* 退让完成后，移动到目标相对位置的速度 */
#define LIFT_HOMING_ACC                 (0U)     /* 加速度档位，0=不用曲线直接按设定速度跑 */
#define LIFT_HOMING_POLL_MS             (5U)     /* 轮询限位开关的间隔 */
#define LIFT_HOMING_STOP_SETTLE_MS      (100U)   /* 退让急停后的稳定等待，再下发下一条命令 */
#define LIFT_HOMING_CLOG_CLEAR_SETTLE_MS (20U)   /* 解堵转保护后的稳定等待 */
#define LIFT_HOMING_ENABLE_SETTLE_MS    (180U)   /* 使能后的稳定等待，沿用 task2/task6 已实测数据 */

/*
 * ⚠️ 待用户提供实测值：退让释放点到最终工作位置的相对脉冲数（负方向，绝对值）。
 * 当前占位为 0（即归零后停在开关释放点，不再继续移动）。
 * 脉冲单位随驱动器细分，出厂 16 细分 = 3200 脉冲/圈。
 */
#define LIFT_HOMING_TARGET_OFFSET_PULSES (18400)

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
        /* 正方向逼近限位开关，直到压下。 */
        BspUart0_SendString("HOMING: seeking switch\r\n");
        Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT, (int16_t)LIFT_HOMING_APPROACH_RPM, LIFT_HOMING_ACC);
        while (!BspHomeSwitch_IsPressed()) {
            Delay_ms(LIFT_HOMING_POLL_MS);
        }
    } else {
        /* 极端情况：开机时开关已被压下，不能再往正方向顶，直接退让。 */
        BspUart0_SendString("HOMING: switch pre-pressed, backing off\r\n");
    }

    /* 压下的瞬间（或本来就压着）立即反向退让，直到开关释放——这是物理归零参考点。 */
    Emm42Robot_SetSpeedRpm(EMM42_ROBOT_LIFT, (int16_t)-LIFT_HOMING_BACKOFF_RPM, LIFT_HOMING_ACC);
    while (BspHomeSwitch_IsPressed()) {
        Delay_ms(LIFT_HOMING_POLL_MS);
    }
    Emm42Robot_Stop(EMM42_ROBOT_LIFT);
    Delay_ms(LIFT_HOMING_STOP_SETTLE_MS);

    /* 继续往负方向移动到指定的相对工作位置。 */
    {
        int32_t  offsetPulses = (int32_t)LIFT_HOMING_TARGET_OFFSET_PULSES;
        uint32_t waitMs;

        Emm42Robot_MoveRelative(EMM42_ROBOT_LIFT, -offsetPulses,
                                 LIFT_HOMING_MOVE_RPM, LIFT_HOMING_ACC);
        waitMs = AppLiftHoming_MoveWaitMs(offsetPulses, LIFT_HOMING_MOVE_RPM);
        Delay_ms(waitMs);
    }

    BspUart0_SendString("HOMING: ID1 done\r\n");
}
