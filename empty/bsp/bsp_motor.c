#include "bsp_motor.h"

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * 梯形加减速参数（定时器时钟 4MHz，period 越小步频越高）。
 * 直接高速起转/切速会失步（电机只抖几度），故从 500Hz 起步逐脉冲逼近目标速度，
 * 停止时对称减速回起步速度再停表，保证全程平稳、不丢步。
 * RAMP_DELTA 降到 4（原为 8），加倍延长加减速段，减轻换驱动芯片后的轻微抖动。
 */
#define MOTOR_STEP_PERIOD_START   (8000U)   /* 8000 → 500Hz 起步/停止速度 */
#define MOTOR_RAMP_DELTA          (4U)      /* 每个脉冲周期的增/减量，决定加减速斜率 */

/* 当前定时器周期（加减速过程中动态变化）。 */
static volatile uint32_t s_curPeriod    = MOTOR_STEP_PERIOD_START;
/* 目标巡航（最高速）周期，由 RunContinuous/MoveSteps 在线设定。 */
static volatile uint32_t s_targetPeriod = MOTOR_STEP_PERIOD_START;
/* 定时器是否正在输出脉冲（1=转动中或正在减速停止）。 */
static volatile uint8_t  s_running      = 0U;

/* ---- 连续旋转模式（RunContinuous / RequestStop） ---- */
/* 停止请求：置 1 后 ISR 把目标拉回起步速度，减速到位即停表。 */
static volatile uint8_t  s_stopRequest  = 1U;

/* ---- 定步数位置移动模式（MoveSteps） ---- */
static volatile uint32_t s_stepsTotal   = 0U;   /* 本次移动总脉冲数 */
static volatile uint32_t s_stepsDone    = 0U;   /* 已发出的脉冲数 */
static volatile uint32_t s_accelSteps   = 0U;   /* 加减速段所需脉冲数阈值 */
static volatile uint8_t  s_positionMove = 0U;   /* 1=位置移动模式，0=连续旋转模式 */

/*
 * 设置 MS1/MS2 电平。
 * MS1/MS2 四路共用，改细分会同时影响所有电机；运行中不建议频繁切换。
 */
static void BspTmc_ApplyMicrostep(uint8_t ms1, uint8_t ms2)
{
    if (ms1) {
        DL_GPIO_setPins(TMC_MS1_PORT, TMC_MS1_PIN);
    } else {
        DL_GPIO_clearPins(TMC_MS1_PORT, TMC_MS1_PIN);
    }

    if (ms2) {
        DL_GPIO_setPins(TMC_MS2_PORT, TMC_MS2_PIN);
    } else {
        DL_GPIO_clearPins(TMC_MS2_PORT, TMC_MS2_PIN);
    }
}

/* 把周期写入定时器 LOAD 与 50% 占空 CC，下个脉冲生效。 */
static void BspMotor1_ApplyPeriod(uint32_t period)
{
    DL_TimerG_setLoadValue(MOTOR_STEP_TIMER_INST, period - 1U);
    DL_TimerG_setCaptureCompareValue(MOTOR_STEP_TIMER_INST,
        period / 2U, DL_TIMER_CC_0_INDEX);
}

/* 巡航周期限幅：不快于安全上限、不慢于起步速度。 */
static uint32_t BspMotor1_ClampPeriod(uint32_t period)
{
    if (period < BSP_MOTOR_PERIOD_MIN) {
        period = BSP_MOTOR_PERIOD_MIN;
    }
    if (period > MOTOR_STEP_PERIOD_START) {
        period = MOTOR_STEP_PERIOD_START;
    }
    return period;
}

void BspMotor_Init(void)
{
    /*
     * 上电安全状态：先禁用驱动，再给定方向和默认细分，最后保持 STEP 停止。
     * GPIO 默认电平已在 SYSCFG_DL_GPIO_init 设好，这里再显式收敛一次，避免误动作。
     */
    BspTmc_DisableAll();
    BspMotor1_SetDir(MOTOR_DIR_FORWARD);
    BspTmc_SetMicrostep(TMC_MICROSTEP_32);
    BspMotor1_StopStep();
}

void BspTmc_SetMicrostep(BspTmcMicrostep_t microstep)
{
    switch (microstep) {
    case TMC_MICROSTEP_16:
        BspTmc_ApplyMicrostep(1U, 1U);
        break;
    case TMC_MICROSTEP_32:
        BspTmc_ApplyMicrostep(1U, 0U);
        break;
    case TMC_MICROSTEP_64:
        BspTmc_ApplyMicrostep(0U, 1U);
        break;
    case TMC_MICROSTEP_8:
    default:
        BspTmc_ApplyMicrostep(0U, 0U);
        break;
    }
}

void BspTmc_EnableAll(void)
{
    /* ENN 低有效，拉低使能四路驱动。 */
    DL_GPIO_clearPins(TMC_ENN_PORT, TMC_ENN_PIN);
}

void BspTmc_DisableAll(void)
{
    /* ENN 拉高禁用四路驱动，电机失力。 */
    DL_GPIO_setPins(TMC_ENN_PORT, TMC_ENN_PIN);
}

void BspMotor1_SetDir(BspMotorDir_t dir)
{
    if (dir == MOTOR_DIR_REVERSE) {
        DL_GPIO_setPins(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    }
}

void BspMotor1_StopStep(void)
{
    /* 立即停表：STEP 输出停在当前电平，电机停步；同步收敛软件状态。 */
    DL_TimerG_stopCounter(MOTOR_STEP_TIMER_INST);
    NVIC_DisableIRQ(MOTOR_STEP_TIMER_IRQn);
    s_running      = 0U;
    s_stopRequest  = 1U;
    s_positionMove = 0U;
    s_curPeriod    = MOTOR_STEP_PERIOD_START;
}

void BspMotor1_RunContinuous(uint32_t cruisePeriod)
{
    s_targetPeriod = BspMotor1_ClampPeriod(cruisePeriod);
    s_stopRequest  = 0U;

    if (s_running != 0U) {
        /* 已在转：本调用等同在线调速，ISR 会平滑逼近新目标。 */
        return;
    }

    /* 从慢速起步，避免直接高速起转失步；后续由 ISR 逐步加速到目标速度。 */
    s_curPeriod = MOTOR_STEP_PERIOD_START;
    BspMotor1_ApplyPeriod(MOTOR_STEP_PERIOD_START);

    /* 清除可能残留的 ZERO 中断标志，再使能中断和 NVIC。 */
    DL_TimerG_clearInterruptStatus(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(MOTOR_STEP_TIMER_IRQn);

    s_running = 1U;
    DL_TimerG_startCounter(MOTOR_STEP_TIMER_INST);
}

void BspMotor1_SetSpeed(uint32_t cruisePeriod)
{
    /* 仅在运行且未请求停止时更新目标速度；停止流程中忽略，避免打断减速。 */
    if ((s_running != 0U) && (s_stopRequest == 0U)) {
        s_targetPeriod = BspMotor1_ClampPeriod(cruisePeriod);
    }
}

void BspMotor1_RequestStop(void)
{
    /* 连续旋转模式：请求平滑减速停止。位置模式会被覆盖为停止。 */
    if (s_running != 0U) {
        s_stopRequest  = 1U;
        s_positionMove = 0U;
    }
}

/*
 * 定步数位置移动：走 steps 个 STEP 脉冲后自动平滑停止。
 * 内部自动计算梯形加减速阈值——若步数太少不足以加速到巡航速度，
 * 自动退化为三角形速度曲线（加速到中点即减速）。
 * 巡航周期 cruisePeriod 会被限幅到安全范围。
 */
void BspMotor1_MoveSteps(uint32_t steps, uint32_t cruisePeriod)
{
    uint32_t accel;

    if (steps == 0U) {
        return;
    }

    cruisePeriod = BspMotor1_ClampPeriod(cruisePeriod);

    /* 从起步速度加速到巡航速度所需的脉冲数。 */
    accel = (MOTOR_STEP_PERIOD_START - cruisePeriod) / MOTOR_RAMP_DELTA;

    if (steps <= (2U * accel)) {
        /* 三角形速度曲线：加速到中点立即减速，不巡航。 */
        s_accelSteps = steps / 2U;
    } else {
        s_accelSteps = accel;
    }

    s_stepsTotal   = steps;
    s_stepsDone    = 0U;
    s_targetPeriod = cruisePeriod;
    s_stopRequest  = 0U;
    s_positionMove = 1U;

    /* 从慢速起步，避免直接高速起转失步。 */
    s_curPeriod = MOTOR_STEP_PERIOD_START;
    BspMotor1_ApplyPeriod(MOTOR_STEP_PERIOD_START);

    /* 清除可能残留的 ZERO 中断标志，再使能中断和 NVIC。 */
    DL_TimerG_clearInterruptStatus(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(MOTOR_STEP_TIMER_IRQn);

    s_running = 1U;
    DL_TimerG_startCounter(MOTOR_STEP_TIMER_INST);
}

uint32_t BspMotor1_GetRemainingSteps(void)
{
    if (s_positionMove == 0U) {
        return 0U;
    }
    if (s_stepsDone >= s_stepsTotal) {
        return 0U;
    }
    return s_stepsTotal - s_stepsDone;
}

bool BspMotor1_IsStopped(void)
{
    return (s_running == 0U);
}

uint32_t BspMotor1_GetCurPeriod(void)
{
    return s_curPeriod;
}

bool BspTmc_IsEnabled(void)
{
    /* ENN 低有效：读输出寄存器 DOUT，低电平=已使能。 */
    return ((TMC_ENN_PORT->DOUT31_0 & TMC_ENN_PIN) == 0U);
}

/*
 * TIMG0 ZERO 中断处理：每个 STEP 脉冲周期结束时触发一次。
 *
 * 连续旋转模式（s_positionMove==0）：每脉冲把当前周期朝目标逼近一个 RAMP_DELTA，
 *   实现梯形加减速与在线调速；减速到起步速度且已请求停止时停表。
 *
 * 位置移动模式（s_positionMove==1）：每脉冲步数+1，根据剩余步数自动判断
 *   加速段→巡航段→减速段，走完所有步数且回到起步速度后自动停表。
 *   若总步数太少不足以上到巡航速度，自动退化为三角形曲线（无巡航段）。
 *
 * ISR 中不调用 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。
 */
void TIMG0_IRQHandler(void)
{
    uint32_t target;
    uint32_t remaining;

    DL_TimerG_clearInterruptStatus(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);

    if (s_running == 0U) {
        return;
    }

    if (s_positionMove != 0U) {
        /* ---- 位置移动模式：步数驱动 ---- */
        if (s_stepsDone < s_stepsTotal) {
            s_stepsDone++;
        }
        remaining = s_stepsTotal - s_stepsDone;

        if (remaining == 0U) {
            /* 所有步数已走完，收尾减速。 */
            target = MOTOR_STEP_PERIOD_START;
        } else if (remaining <= s_accelSteps) {
            /* 进入减速段：朝起步速度减速。 */
            target = MOTOR_STEP_PERIOD_START;
        } else if (s_stepsDone <= s_accelSteps) {
            /* 加速段：朝巡航速度加速。 */
            target = s_targetPeriod;
        } else {
            /* 巡航段：保持巡航速度。 */
            target = s_targetPeriod;
        }
    } else {
        /* ---- 连续旋转模式：手动启停 ---- */
        target = (s_stopRequest != 0U) ? MOTOR_STEP_PERIOD_START : s_targetPeriod;
    }

    /* 通用梯形斜坡：逐脉冲朝 target 逼近一个 RAMP_DELTA。 */
    if (s_curPeriod < target) {
        /* 减速（周期变大）。 */
        s_curPeriod += MOTOR_RAMP_DELTA;
        if (s_curPeriod > target) {
            s_curPeriod = target;
        }
        BspMotor1_ApplyPeriod(s_curPeriod);
    } else if (s_curPeriod > target) {
        /* 加速（周期变小）。 */
        s_curPeriod -= MOTOR_RAMP_DELTA;
        if (s_curPeriod < target) {
            s_curPeriod = target;
        }
        BspMotor1_ApplyPeriod(s_curPeriod);
    }

    /* 停止判定。 */
    if (s_positionMove != 0U) {
        /* 位置模式：所有步数走完且回到起步速度 → 停表。 */
        remaining = s_stepsTotal - s_stepsDone;
        if ((remaining == 0U) && (s_curPeriod >= MOTOR_STEP_PERIOD_START)) {
            DL_TimerG_stopCounter(MOTOR_STEP_TIMER_INST);
            NVIC_DisableIRQ(MOTOR_STEP_TIMER_IRQn);
            s_running      = 0U;
            s_positionMove = 0U;
        }
    } else {
        /* 连续模式：已请求停止且减速到起步速度 → 停表。 */
        if ((s_stopRequest != 0U) && (s_curPeriod >= MOTOR_STEP_PERIOD_START)) {
            DL_TimerG_stopCounter(MOTOR_STEP_TIMER_INST);
            NVIC_DisableIRQ(MOTOR_STEP_TIMER_IRQn);
            s_running = 0U;
        }
    }
}
