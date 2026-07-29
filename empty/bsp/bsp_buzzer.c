#include "bsp_buzzer.h"

#include "ti_msp_dl_config.h"

/* 80MHz 主频下约为 2~3ms；按键与题目完成均调用同一接口，保证响铃时长一致。 */
#define BSP_BUZZER_SHORT_BEEP_LOOPS    (50000U)

void BspBuzzer_Init(void)
{
    /* 上电默认静音，防止误响。 */
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

void BspBuzzer_On(void)
{
    /* 有源蜂鸣器：拉高电平即持续响。 */
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
}

void BspBuzzer_Off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

void BspBuzzer_BeepShort(void)
{
    volatile uint32_t loops;

    BspBuzzer_On();
    for (loops = 0U; loops < BSP_BUZZER_SHORT_BEEP_LOOPS; loops++) {
        /* 使用 volatile 计数器保留短暂忙等，保证蜂鸣器立刻关断。 */
    }
    BspBuzzer_Off();
}

void BspBuzzer_Toggle(void)
{
    DL_GPIO_togglePins(BUZZER_PORT, BUZZER_PIN);
}
