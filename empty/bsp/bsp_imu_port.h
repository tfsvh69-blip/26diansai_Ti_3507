#ifndef BSP_IMU_PORT_H
#define BSP_IMU_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool BspImuPort_WriteReg(uint8_t devAddr, uint8_t reg, const uint8_t *data, uint16_t len);
bool BspImuPort_ReadReg(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t len);
bool BspImuPort_ProbeAddress(uint8_t devAddr, uint8_t probeReg);
bool BspImuPort_ReadIntLevel(void);
void BspImuPort_RecoverBus(void);
void BspImuPort_GetBusState(bool *sclHigh, bool *sdaHigh, uint32_t *status);
void BspImuPort_EnterPinTestMode(void);
void BspImuPort_SetPinTestLevel(bool sclHigh, bool sdaHigh);
void BspImuPort_ReadPinTestLevel(bool *sclHigh, bool *sdaHigh);

#ifdef __cplusplus
}
#endif

#endif
