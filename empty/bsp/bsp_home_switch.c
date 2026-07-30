#include "bsp_home_switch.h"

#include "ti_msp_dl_config.h"

bool BspHomeSwitch_IsPressed(void)
{
    /* 开关一端接 GND、内部上拉：读到低电平(0)即按下。 */
    return (DL_GPIO_readPins(HOME_SWITCH_PORT, HOME_SWITCH_PIN) == 0U);
}
