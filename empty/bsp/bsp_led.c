#include "bsp_led.h"

#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} BspLedResource_t;

static const BspLedResource_t s_ledResource[BSP_LED_COUNT] = {
    { LED_LED1_PORT, LED_LED1_PIN },
};

static const BspLedResource_t *BspLed_GetResource(BspLedId_t led)
{
    if ((uint32_t)led >= (uint32_t)BSP_LED_COUNT) {
        return NULL;
    }

    return &s_ledResource[led];
}

void BspLed_Init(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)BSP_LED_COUNT; i++) {
        /* 当前板载 PB22 LED 实测为高电平点亮，上电默认先熄灭。 */
        DL_GPIO_clearPins(s_ledResource[i].port, s_ledResource[i].pin);
    }
}

void BspLed_On(BspLedId_t led)
{
    const BspLedResource_t *resource = BspLed_GetResource(led);

    if (resource == NULL) {
        return;
    }

    DL_GPIO_setPins(resource->port, resource->pin);
}

void BspLed_Off(BspLedId_t led)
{
    const BspLedResource_t *resource = BspLed_GetResource(led);

    if (resource == NULL) {
        return;
    }

    DL_GPIO_clearPins(resource->port, resource->pin);
}

void BspLed_Toggle(BspLedId_t led)
{
    const BspLedResource_t *resource = BspLed_GetResource(led);

    if (resource == NULL) {
        return;
    }

    DL_GPIO_togglePins(resource->port, resource->pin);
}
