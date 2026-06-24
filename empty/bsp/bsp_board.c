#include "bsp_board.h"

#include "bsp_led.h"
#include "bsp_motor.h"
#include "ti_msp_dl_config.h"

void BspBoard_Init(void)
{
    /*
     * 不调用 SYSCFG_DL_init()，因为该函数会初始化 SysTick。
     * FreeRTOS 需要独占 SysTick 作为系统节拍。
     */
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_I2C_1_init();
    SYSCFG_DL_TIMER_STEP_init();

    BspLed_Init();
    BspMotor_Init();
}
