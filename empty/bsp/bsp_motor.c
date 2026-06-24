#include "bsp_motor.h"

#include "ti_msp_dl_config.h"

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
