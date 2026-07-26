#include "bsp_nrf24_port.h"

#include "ti_msp_dl_config.h"

/*
 * NRF24L01+ 使用 GPIO 模拟 SPI 模式 0。
 * PA1/CSN 是开漏引脚，高电平必须通过外部 4.7kΩ 上拉获得，因此“置高”实际是释放输出。
 */

#define BSP_NRF24_SPI_HALF_PERIOD_CYCLES (CPUCLK_FREQ / 4000000UL)

static void BspNrf24Port_SpiDelay(void)
{
    DL_Common_delayCycles(BSP_NRF24_SPI_HALF_PERIOD_CYCLES);
}

void BspNrf24Port_Init(void)
{
    /* 上电安全状态：退出收发、SPI 时钟为低、MOSI 为低、CSN 释放为高。 */
    DL_GPIO_clearPins(NRF24_CE_PORT, NRF24_CE_PIN);
    DL_GPIO_clearPins(NRF24_SCK_PORT, NRF24_SCK_PIN);
    DL_GPIO_clearPins(NRF24_MOSI_PORT, NRF24_MOSI_PIN);
    DL_GPIO_enableOutput(NRF24_CE_PORT, NRF24_CE_PIN);
    DL_GPIO_enableOutput(NRF24_SCK_PORT, NRF24_SCK_PIN);
    DL_GPIO_enableOutput(NRF24_MOSI_PORT, NRF24_MOSI_PIN);

    DL_GPIO_clearPins(NRF24_CSN_PORT, NRF24_CSN_PIN);
    DL_GPIO_disableOutput(NRF24_CSN_PORT, NRF24_CSN_PIN);
}

void BspNrf24Port_SetCe(bool high)
{
    if (high) {
        DL_GPIO_setPins(NRF24_CE_PORT, NRF24_CE_PIN);
    } else {
        DL_GPIO_clearPins(NRF24_CE_PORT, NRF24_CE_PIN);
    }
}

void BspNrf24Port_SetCsn(bool high)
{
    if (high) {
        /* PA1 只能开漏驱动，释放输出后由外部上拉电阻把 CSN 拉到 3.3V。 */
        DL_GPIO_disableOutput(NRF24_CSN_PORT, NRF24_CSN_PIN);
    } else {
        DL_GPIO_clearPins(NRF24_CSN_PORT, NRF24_CSN_PIN);
        DL_GPIO_enableOutput(NRF24_CSN_PORT, NRF24_CSN_PIN);
    }
}

uint8_t BspNrf24Port_SpiExchange(uint8_t value)
{
    uint8_t bit;
    uint8_t received = 0U;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            DL_GPIO_setPins(NRF24_MOSI_PORT, NRF24_MOSI_PIN);
        } else {
            DL_GPIO_clearPins(NRF24_MOSI_PORT, NRF24_MOSI_PIN);
        }

        BspNrf24Port_SpiDelay();
        DL_GPIO_setPins(NRF24_SCK_PORT, NRF24_SCK_PIN);
        BspNrf24Port_SpiDelay();

        received <<= 1;
        if (DL_GPIO_readPins(NRF24_MISO_PORT, NRF24_MISO_PIN) != 0U) {
            received |= 1U;
        }

        DL_GPIO_clearPins(NRF24_SCK_PORT, NRF24_SCK_PIN);
        value <<= 1;
    }

    return received;
}

void BspNrf24Port_DelayUs(uint32_t us)
{
    if (us != 0U) {
        DL_Common_delayCycles(us * (CPUCLK_FREQ / 1000000UL));
    }
}

uint8_t BspNrf24Port_ReadPinLevels(void)
{
    uint8_t levels = 0U;

    if (DL_GPIO_readPins(NRF24_CE_PORT, NRF24_CE_PIN) != 0U) {
        levels |= 0x01U;
    }
    if (DL_GPIO_readPins(NRF24_CSN_PORT, NRF24_CSN_PIN) != 0U) {
        levels |= 0x02U;
    }
    if (DL_GPIO_readPins(NRF24_SCK_PORT, NRF24_SCK_PIN) != 0U) {
        levels |= 0x04U;
    }
    if (DL_GPIO_readPins(NRF24_MOSI_PORT, NRF24_MOSI_PIN) != 0U) {
        levels |= 0x08U;
    }
    if (DL_GPIO_readPins(NRF24_MISO_PORT, NRF24_MISO_PIN) != 0U) {
        levels |= 0x10U;
    }

    return levels;
}
