#include "bsp_relay.h"

#include "ti_msp_dl_config.h"

/*
 * 继电器触发极性：0 = 高电平吸合(通)、低电平断开(停)（**2026-07-25 已上板实测确认**，
 *                     OLED 显示 R:ON 时继电器确实吸合，与接线文档 R7 下拉建议一致）；
 *                 1 = 低电平吸合（换成低电平触发的继电器模块时改这里，其余代码不用动）。
 */
#ifndef BSP_RELAY_ACTIVE_LOW
#define BSP_RELAY_ACTIVE_LOW   (0U)
#endif

/* 内部维护的逻辑状态（true=吸合），随每次 On/Off/Toggle 更新，供 BspRelay_IsOn 只读。 */
static bool s_relayOn = false;

void BspRelay_Init(void)
{
    /* 上电默认断开，避免电磁铁误动作。 */
    BspRelay_Off();
}

void BspRelay_On(void)
{
#if (BSP_RELAY_ACTIVE_LOW != 0U)
    DL_GPIO_clearPins(RELAY_PORT, RELAY_PIN);
#else
    DL_GPIO_setPins(RELAY_PORT, RELAY_PIN);
#endif
    s_relayOn = true;
}

void BspRelay_Off(void)
{
#if (BSP_RELAY_ACTIVE_LOW != 0U)
    DL_GPIO_setPins(RELAY_PORT, RELAY_PIN);
#else
    DL_GPIO_clearPins(RELAY_PORT, RELAY_PIN);
#endif
    s_relayOn = false;
}

void BspRelay_Set(bool on)
{
    if (on) {
        BspRelay_On();
    } else {
        BspRelay_Off();
    }
}

void BspRelay_Toggle(void)
{
    /* 翻转电平即翻转吸合/断开状态，与极性无关。 */
    DL_GPIO_togglePins(RELAY_PORT, RELAY_PIN);
    s_relayOn = !s_relayOn;
}

bool BspRelay_IsOn(void)
{
    return s_relayOn;
}
