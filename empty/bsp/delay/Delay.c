#include "Delay.h"

void Delay_init(void)
{
    /*
     * FreeRTOS 使用 SysTick 作为系统节拍。
     * 这里保持空实现，避免重新调用 SYSCFG_DL_init() 抢占 SysTick。
     */
}

void Delay_us(unsigned long us)
{
    unsigned long cycles;

    if (us == 0UL) {
        return;
    }

    /* 使用 CPU 周期忙等，供 OLED 软件 I2C 这类短延时使用。 */
    cycles = us * (CPUCLK_FREQ / 1000000UL);
    delay_cycles(cycles);
}

void Delay_ms(unsigned long ms)
{
    while (ms > 0UL) {
        Delay_us(1000UL);
        ms--;
    }
}

void Delay_1us(unsigned long us)
{
    Delay_us(us);
}

void Delay_1ms(unsigned long ms)
{
    Delay_ms(ms);
}
