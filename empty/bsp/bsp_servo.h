#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 舵机板级封装（v1.1，TIMA0 50Hz PWM）。
 * 四路舵机共用 TIMA0 时基，各通道独立比较值；脉冲宽度范围 500~2500us。
 * 当前先实现舵机1（SERVO1=PA8，TIMA0_CCP0）。
 */

/*
 * 安全脉宽范围（微秒）：800~2200us，中心 1500us。
 *
 * 【为什么收窄到 800~2200】(2026-07-16 决策)
 * - 常规舵机在 ~600us / ~2400us 的两个极端常顶到机械限位：要么不动、要么堵转发抖/发热，
 *   即"命令了最大/最小角，实际转不到或无效"。收到 800~2200 给机械行程留安全余量，
 *   仍有 ±700us≈±63° 的充足行程。
 * - 注意：这不是 MCU 侧问题。bsp_servo 的极性补偿(period-pulseUs)在任何脉宽都精确，
 *   800/2200us 也会精确输出对应高电平；极值"无效"是舵机机械限位所致，与软件无关。
 * - 若某舵机行程更大/更小，按需调这两个值即可；SetPulseUs 会自动把超范围的命令限幅到此区间。
 */
#define SERVO_PULSE_MIN_US    (800U)
#define SERVO_PULSE_MAX_US   (2200U)
#define SERVO_PULSE_CENTER_US (1500U)

/* 舵机编号。 */
typedef enum {
    BSP_SERVO_1 = 0,
    BSP_SERVO_2,
    BSP_SERVO_3,
    BSP_SERVO_4,
    BSP_SERVO_COUNT
} BspServoId_t;

/* 设置指定舵机的脉宽（微秒），自动限幅到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]。 */
void BspServo_SetPulseUs(BspServoId_t id, uint16_t pulseUs);

/*
 * 可指定安全范围的脉宽设置（供舵机行程测试使用）。
 * 脉宽会被限幅到 [minUs, maxUs]，传入 0 的 min/max 回退到默认 SAFE 范围。
 * 典型用法：BspServo_SetPulseUsRange(id, 500, 500, 2500) → 测试 270° 舵机全行程。
 * ⚠️ 仅在测试时使用宽范围，正常业务仍走 SetPulseUs（800~2200us 安全区间）。
 */
void BspServo_SetPulseUsRange(BspServoId_t id, uint16_t pulseUs,
                               uint16_t minUs, uint16_t maxUs);

/* 启动/停止 TIMA0 计数器（启停所有舵机 PWM 输出）。 */
void BspServo_Start(void);
void BspServo_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
