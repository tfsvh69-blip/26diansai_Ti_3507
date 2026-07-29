#include "bsp_line.h"

#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * 循迹模块极性开关：0 = 识别到线时信号脚为高电平；
 *                   1 = 识别到线时为低电平（当前模块，用户实测确认）。
 */
#ifndef BSP_LINE_ACTIVE_LOW
#define BSP_LINE_ACTIVE_LOW   (1U)
#endif

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} BspLineResource_t;

/* 顺序即 LINE1~LINE8（PB17~PB24），下标 0=LINE1(最右) … 7=LINE8(最左)。 */
static const BspLineResource_t s_lineResource[BSP_LINE_COUNT] = {
    { LINE1_PORT, LINE1_PIN },
    { LINE2_PORT, LINE2_PIN },
    { LINE3_PORT, LINE3_PIN },
    { LINE4_PORT, LINE4_PIN },
    { LINE5_PORT, LINE5_PIN },
    { LINE6_PORT, LINE6_PIN },
    { LINE7_PORT, LINE7_PIN },
    { LINE8_PORT, LINE8_PIN },
};

uint8_t BspLine_ReadAll(void)
{
    uint8_t  bitmap = 0U;
    uint32_t i;

    for (i = 0U; i < (uint32_t)BSP_LINE_COUNT; i++) {
        /* 读到高电平即信号脚为 1；按极性归一化为“是否识别到线”。 */
        bool high     = (DL_GPIO_readPins(s_lineResource[i].port,
                                          s_lineResource[i].pin) != 0U);
#if (BSP_LINE_ACTIVE_LOW != 0U)
        bool detected = !high;
#else
        bool detected = high;
#endif
        if (detected) {
            bitmap |= (uint8_t)(1U << i);
        }
    }
    return bitmap;
}

bool BspLine_IsDetected(uint8_t ch)
{
    if (ch >= (uint8_t)BSP_LINE_COUNT) {
        return false;
    }
    return (BspLine_ReadAll() & (uint8_t)(1U << ch)) != 0U;
}
