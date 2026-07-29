#include "bsp_servo.h"

#include "ti_msp_dl_config.h"

/* CCP 通道索引表，CCP0=SERVO1 依次对应。 */
static const uint32_t s_ccpIndex[BSP_SERVO_COUNT] = {
    DL_TIMER_CC_0_INDEX,   /* SERVO1 */
    DL_TIMER_CC_1_INDEX,   /* SERVO2 */
    DL_TIMER_CC_2_INDEX,   /* SERVO3 */
    DL_TIMER_CC_3_INDEX    /* SERVO4 */
};

void BspServo_SetPulseUs(BspServoId_t id, uint16_t pulseUs)
{
    BspServo_SetPulseUsRange(id, pulseUs, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
}

void BspServo_SetPulseUsRange(BspServoId_t id, uint16_t pulseUs,
                               uint16_t minUs, uint16_t maxUs)
{
    uint32_t cc;

    if ((uint32_t)id >= (uint32_t)BSP_SERVO_COUNT) {
        return;
    }

    /* min/max 为 0 时回退到默认安全范围。 */
    if (minUs == 0U) { minUs = SERVO_PULSE_MIN_US; }
    if (maxUs == 0U) { maxUs = SERVO_PULSE_MAX_US; }

    /* 限幅到调用者指定的范围 */
    if (pulseUs < minUs) {
        pulseUs = minUs;
    }
    if (pulseUs > maxUs) {
        pulseUs = maxUs;
    }

    /*
     * 定时器时钟 1MHz → 1 个计数值 = 1us。
     * 【极性补偿·经实测】该定时器输出 高电平 = period - CC（详见
     * ti_msp_dl_config.c 中 SYSCFG_DL_TIMER_SERVO_init 的极性说明）：
     * 直接写 pulseUs 会得到 (period - pulseUs) 的高电平（脉冲极性反相）。
     * 故写入 (period - pulseUs)，使 PA8 实际高脉冲宽度正好等于 pulseUs。
     * 通过 CCP 更新方法已设为 IMMEDIATE，写入后下一周期立即生效。
     */
    cc = (uint32_t)SERVO_TIMER_PERIOD - (uint32_t)pulseUs;
    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST, cc, s_ccpIndex[id]);
}

void BspServo_Start(void)
{
    DL_Timer_startCounter(SERVO_TIMER_INST);
}

void BspServo_Stop(void)
{
    DL_Timer_stopCounter(SERVO_TIMER_INST);
}
