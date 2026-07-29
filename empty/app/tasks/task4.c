#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_servo.h"

/* ==================================================================
 * 第 4 题：四路舵机 2 秒间隔 0° ↔ 270° 翻转测试
 *
 * 每隔 2 秒四路舵机同时在 0°（500us）和 270°（2500us）之间切换，
 * 用于快速验证舵机全行程覆盖能力和 PWM 独立性。
 *
 * 使用 BspServo_SetPulseUsRange() 临时放宽限幅到 500~2500us；
 * 正常业务仍走默认 800~2200us 安全区间。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 可调参数
 * ------------------------------------------------------------------ */

/* 测试用脉宽范围（微秒）。500us≈0°、2500us≈270°（典型 270° 舵机）。 */
#define T4_PULSE_MIN_US   (500U)
#define T4_PULSE_MAX_US   (2500U)

/* 翻转间隔（30ms 轮询周期数）。67 拍 ≈ 2.0 秒。 */
#define T4_TOGGLE_TICKS   (67U)

/* ------------------------------------------------------------------
 * 状态机
 * ------------------------------------------------------------------ */

typedef enum {
    T4_STATE_START = 0,   /* 初始化：归 0° */
    T4_STATE_TOGGLE,      /* 每 2 秒翻转 */
    T4_STATE_DONE         /* 异常（未使用） */
} Task4State_t;

static Task4State_t s_state;
static bool         s_atMax;       /* true=当前在 270°, false=在 0° */
static uint32_t     s_tickCount;   /* 距离上次翻转的拍数 */

/* 四路同时设相同脉宽，省去重复调用。 */
static void Task4_SetAllServos(uint16_t pulseUs)
{
    BspServo_SetPulseUsRange(BSP_SERVO_1, pulseUs, T4_PULSE_MIN_US, T4_PULSE_MAX_US);
    BspServo_SetPulseUsRange(BSP_SERVO_2, pulseUs, T4_PULSE_MIN_US, T4_PULSE_MAX_US);
    BspServo_SetPulseUsRange(BSP_SERVO_3, pulseUs, T4_PULSE_MIN_US, T4_PULSE_MAX_US);
    BspServo_SetPulseUsRange(BSP_SERVO_4, pulseUs, T4_PULSE_MIN_US, T4_PULSE_MAX_US);
}

void Task4_OnEnter(void)
{
    /* 四路归 0°，启动 PWM。 */
    s_atMax     = false;
    s_tickCount = 0U;
    Task4_SetAllServos(T4_PULSE_MIN_US);
    BspServo_Start();
    s_state = T4_STATE_TOGGLE;
}

void Task4_OnLoop(void)
{
    switch (s_state) {
    case T4_STATE_START:
        break;

    case T4_STATE_TOGGLE:
        s_tickCount++;
        if (s_tickCount >= T4_TOGGLE_TICKS) {
            s_tickCount = 0U;
            s_atMax = !s_atMax;
            Task4_SetAllServos(s_atMax ? T4_PULSE_MAX_US : T4_PULSE_MIN_US);
        }
        break;

    case T4_STATE_DONE:
        break;

    default:
        break;
    }
}

void Task4_OnExit(void)
{
    /* 归中后停 PWM，保证退出时舵机回到安全位置。 */
    Task4_SetAllServos((T4_PULSE_MIN_US + T4_PULSE_MAX_US) / 2U);
    BspServo_Stop();
    s_atMax     = false;
    s_tickCount = 0U;
    s_state     = T4_STATE_START;
}
