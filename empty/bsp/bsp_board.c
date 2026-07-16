#include "bsp_board.h"

#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
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
    SYSCFG_DL_UART_2_init();
    SYSCFG_DL_I2C_1_init();
    SYSCFG_DL_TIMER_STEP_init();
    SYSCFG_DL_TIMER_SERVO_init();

    BspLed_Init();
    BspMotor_Init();
    BspBuzzer_Init();

    /* UART0 互斥量必须在任一任务打印、调度器启动之前创建。 */
    BspUart0_Init();
}
