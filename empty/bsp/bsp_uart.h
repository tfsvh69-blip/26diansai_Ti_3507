#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BspUart0_SendByte(uint8_t byte);
void BspUart0_SendString(const char *str);
bool BspUart0_ReadByte(uint8_t *byte);
void BspUart0_Lock(void);
void BspUart0_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif
