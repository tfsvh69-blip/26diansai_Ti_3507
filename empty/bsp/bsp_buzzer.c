#include "bsp_buzzer.h"

#include "ti_msp_dl_config.h"

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

void BspBuzzer_Toggle(void)
{
    DL_GPIO_togglePins(BUZZER_PORT, BUZZER_PIN);
}
