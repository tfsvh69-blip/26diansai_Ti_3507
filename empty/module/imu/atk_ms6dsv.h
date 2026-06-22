#ifndef ATK_MS6DSV_H
#define ATK_MS6DSV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ATK_MS6DSV_OK = 0,
    ATK_MS6DSV_ERROR_ID,
    ATK_MS6DSV_ERROR_COMM,
    ATK_MS6DSV_ERROR_CONFIG,
} AtkMs6dsvStatus_t;

typedef struct {
    int16_t rollCentideg;
    int16_t pitchCentideg;
    int16_t yawCentideg;
    uint8_t fifoLevel;
    bool intLevel;
} AtkMs6dsvEuler_t;

AtkMs6dsvStatus_t AtkMs6dsv_Init(void);
bool AtkMs6dsv_ReadEuler(AtkMs6dsvEuler_t *euler);
bool AtkMs6dsv_ReadWhoAmI(uint8_t devAddr, uint8_t *id);
bool AtkMs6dsv_GetLastInitId(uint8_t *id);
const char *AtkMs6dsv_GetLastInitStep(void);

#ifdef __cplusplus
}
#endif

#endif
