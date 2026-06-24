#include "bsp_key.h"

#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} BspKeyResource_t;

static const BspKeyResource_t s_keyResource[BSP_KEY_COUNT] = {
    { KEY1_PORT, KEY1_PIN },
    { KEY2_PORT, KEY2_PIN },
    { KEY3_PORT, KEY3_PIN },
    { KEY4_PORT, KEY4_PIN },
};

bool BspKey_IsPressed(BspKeyId_t key)
{
    if ((uint32_t)key >= (uint32_t)BSP_KEY_COUNT) {
        return false;
    }

    /* 按键一端接 GND、内部上拉：读到低电平(0)即按下。 */
    return (DL_GPIO_readPins(s_keyResource[key].port,
                             s_keyResource[key].pin) == 0U);
}
