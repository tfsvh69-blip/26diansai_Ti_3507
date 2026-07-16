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

/*
 * 加速度/角速度原始输出。
 * accRaw/gyrRaw 为寄存器 LSB 原值；accMg/gyrMdps 为按初始化量程换算后的物理值，
 * 单位分别为 mg(0.001g) 与 mdps(0.001dps)，用整数表示以避免串口浮点格式化。
 * 数组下标 0/1/2 对应 X/Y/Z。
 */
typedef struct {
    int16_t accRaw[3];
    int16_t gyrRaw[3];
    int32_t accMg[3];
    int32_t gyrMdps[3];
} AtkMs6dsvImuRaw_t;

AtkMs6dsvStatus_t AtkMs6dsv_Init(void);
bool AtkMs6dsv_ReadEuler(AtkMs6dsvEuler_t *euler);
bool AtkMs6dsv_ReadImuRaw(AtkMs6dsvImuRaw_t *raw);
bool AtkMs6dsv_ReadWhoAmI(uint8_t devAddr, uint8_t *id);
bool AtkMs6dsv_GetLastInitId(uint8_t *id);
const char *AtkMs6dsv_GetLastInitStep(void);

#ifdef __cplusplus
}
#endif

#endif
