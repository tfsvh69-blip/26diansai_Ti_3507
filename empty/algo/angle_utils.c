#include "angle_utils.h"

int32_t Angle_DiffCd(int16_t targetCd, int16_t currentCd)
{
    int32_t diff = (int32_t)targetCd - (int32_t)currentCd;

    /* 只需一次矫正即可归一化到 [−18000, +18000]，
     * while 写法仅为防御极端跑飞值，正常不会循环超过一次。 */
    while (diff > 18000) {
        diff -= 36000;
    }
    while (diff < -18000) {
        diff += 36000;
    }
    return diff;
}

int16_t Angle_NormalizeCd(int32_t rawCd)
{
    while (rawCd > 18000) {
        rawCd -= 36000;
    }
    while (rawCd < -18000) {
        rawCd += 36000;
    }
    return (int16_t)rawCd;
}
