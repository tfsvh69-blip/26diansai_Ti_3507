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

/* 安全脉宽范围（微秒）：600~2400us，超出可能打坏舵机限位。 */
#define SERVO_PULSE_MIN_US    (600U)
#define SERVO_PULSE_MAX_US   (2400U)
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

/* 启动/停止 TIMA0 计数器（启停所有舵机 PWM 输出）。 */
void BspServo_Start(void);
void BspServo_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
