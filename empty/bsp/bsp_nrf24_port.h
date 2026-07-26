#ifndef BSP_NRF24_PORT_H
#define BSP_NRF24_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BspNrf24Port_Init(void);
void BspNrf24Port_SetCe(bool high);
void BspNrf24Port_SetCsn(bool high);
uint8_t BspNrf24Port_SpiExchange(uint8_t value);
void BspNrf24Port_DelayUs(uint32_t us);

/*
 * 返回五根信号线当前物理电平位图，供 Keil Watch 诊断：
 * bit0=CE、bit1=CSN、bit2=SCK、bit3=MOSI、bit4=MISO。
 */
uint8_t BspNrf24Port_ReadPinLevels(void);

#ifdef __cplusplus
}
#endif

#endif
