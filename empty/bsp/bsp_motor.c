#include "bsp_motor.h"

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * 梯形加减速参数（定时器时钟 4MHz，period 越小步频越高）。
 * 直接高速起转会失步（电机只抖几度），故从 1kHz 起步逐步加速到调用指定的巡航速度，
 * 末段对称减速回 1kHz，保证整段脉冲平稳跑完。
 * 巡航（最高）速度由 StartRotateSteps 的 cruisePeriod 参数决定，不再写死。
 */
#define MOTOR_STEP_PERIOD_START   (4000U)                    /* 4000 → 1kHz 起步速度 */
#define MOTOR_RAMP_DELTA          (8U)                       /* 每步周期增减量 */
#define MOTOR_RAMP_STEPS          (475U)                     /* 减速段触发的剩余步数阈值 */

/* 定长步进剩余脉冲数，ISR 中倒计；为 0 时表示本次旋转已完成。 */
static volatile uint32_t s_stepsRemaining = 0U;
/* 旋转完成标志；StartRotateSteps 置 0，ISR 完成后置 1。 */
static volatile uint8_t  s_rotateDone     = 1U;
/* 当前定时器周期（加减速过程中动态变化）。 */
static volatile uint32_t s_curPeriod      = MOTOR_STEP_PERIOD_START;
/* 本次旋转的巡航（最高速）周期，由 StartRotateSteps 设定。 */
static volatile uint32_t s_cruisePeriod   = MOTOR_STEP_PERIOD_START;

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

void BspMotor_Init(void)
{
    /*
     * 上电安全状态：先禁用驱动，再给定方向和默认细分，最后保持 STEP 停止。
     * GPIO 默认电平已在 SYSCFG_DL_GPIO_init 设好，这里再显式收敛一次，避免误动作。
     */
    BspTmc_DisableAll();
    BspMotor1_SetDir(MOTOR_DIR_FORWARD);
    BspTmc_SetMicrostep(TMC_MICROSTEP_8);
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

void BspMotor1_StartStep(void)
{
    /* 启动 STEP 定时器，CCP0 连续输出方波，电机持续步进。 */
    DL_TimerG_startCounter(MOTOR_STEP_TIMER_INST);
}

void BspMotor1_StopStep(void)
{
    /* 停止计数，STEP 输出停在当前电平，电机停步。 */
    DL_TimerG_stopCounter(MOTOR_STEP_TIMER_INST);
}

void BspMotor1_StartRotateSteps(uint32_t steps, uint32_t cruisePeriod)
{
    /* 确保上一次旋转已结束，避免重入冲突。 */
    DL_TimerG_stopCounter(MOTOR_STEP_TIMER_INST);
    NVIC_DisableIRQ(MOTOR_STEP_TIMER_IRQn);

    s_rotateDone     = 0U;
    s_stepsRemaining = steps;

    /* 巡航周期限幅：不快于安全下限、不慢于起步速度（慢于起步则全程按起步速度跑）。 */
    if (cruisePeriod < 1U) {
        cruisePeriod = 1U;
    }
    if (cruisePeriod > MOTOR_STEP_PERIOD_START) {
        cruisePeriod = MOTOR_STEP_PERIOD_START;
    }
    s_cruisePeriod = cruisePeriod;

    /* 从慢速起步，避免直接高速起转失步；后续由 ISR 逐步加速到巡航速度。 */
    s_curPeriod = MOTOR_STEP_PERIOD_START;
    DL_TimerG_setLoadValue(MOTOR_STEP_TIMER_INST, MOTOR_STEP_PERIOD_START - 1U);
    DL_TimerG_setCaptureCompareValue(MOTOR_STEP_TIMER_INST,
        MOTOR_STEP_PERIOD_START / 2U, DL_TIMER_CC_0_INDEX);

    /* 清除可能残留的 ZERO 中断标志，再使能中断和 NVIC。 */
    DL_TimerG_clearInterruptStatus(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(MOTOR_STEP_TIMER_IRQn);

    DL_TimerG_startCounter(MOTOR_STEP_TIMER_INST);
}

bool BspMotor1_IsRotateDone(void)
{
    return (s_rotateDone != 0U);
}

/*
 * TIMG0 ZERO 中断处理：每个 STEP 脉冲周期结束时触发一次。
 * 倒计步数；归零后停定时器、关中断并置完成标志。
 * ISR 中不调用 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。
 */
void TIMG0_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(MOTOR_STEP_TIMER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);

    if (s_stepsRemaining == 0U) {
        return;
    }

    s_stepsRemaining--;

    if (s_stepsRemaining == 0U) {
        DL_TimerG_stopCounter(MOTOR_STEP_TIMER_INST);
        NVIC_DisableIRQ(MOTOR_STEP_TIMER_IRQn);
        s_rotateDone = 1U;
        return;
    }

    /*
     * 梯形加减速：每个脉冲调整下一周期的定时器 LOAD/CC。
     * 末段(剩余<=RAMP_STEPS)减速、起步段加速、中段巡航保持 MIN。
     * 减速分支优先，避免与加速条件在 MIN 附近来回冲突。
     */
    if (s_stepsRemaining <= MOTOR_RAMP_STEPS) {
        if (s_curPeriod < MOTOR_STEP_PERIOD_START) {
            s_curPeriod += MOTOR_RAMP_DELTA;
            if (s_curPeriod > MOTOR_STEP_PERIOD_START) {
                s_curPeriod = MOTOR_STEP_PERIOD_START;
            }
            DL_TimerG_setLoadValue(MOTOR_STEP_TIMER_INST, s_curPeriod - 1U);
            DL_TimerG_setCaptureCompareValue(MOTOR_STEP_TIMER_INST,
                s_curPeriod / 2U, DL_TIMER_CC_0_INDEX);
        }
    } else if (s_curPeriod > s_cruisePeriod) {
        s_curPeriod -= MOTOR_RAMP_DELTA;
        if (s_curPeriod < s_cruisePeriod) {
            s_curPeriod = s_cruisePeriod;
        }
        DL_TimerG_setLoadValue(MOTOR_STEP_TIMER_INST, s_curPeriod - 1U);
        DL_TimerG_setCaptureCompareValue(MOTOR_STEP_TIMER_INST,
            s_curPeriod / 2U, DL_TIMER_CC_0_INDEX);
    }
}
