#ifndef ALGO_ANGLE_UTILS_H
#define ALGO_ANGLE_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 角度工具：处理 ±180°（厘度 ±18000）范围的差值计算与归一化。
 *
 * 纯算法，不依赖硬件或 RTOS。单位约定：
 *   - 内部统一使用厘度（0.01°），即 1° = 100 cd；
 *   - 范围 −18000 ~ +18000；
 *   - 调用者负责保证输入在此范围内。
 */

/*
 * 最短路径角度差：target − current，结果包裹在 [−18000, +18000]。
 * 正值 = 需要右转才能到达目标；负值 = 需要左转。
 *
 * 例：target=−17000(−170°), current=17000(+170°)
 *     → 最短路径为 −2000(−20°) = 向左转 20° 更近。
 */
int32_t Angle_DiffCd(int16_t targetCd, int16_t currentCd);

/*
 * 将厘度值归一化到 [−18000, +18000]。
 * 适用于把运算结果（如 target = start − 9000）收束到标准范围。
 */
int16_t Angle_NormalizeCd(int32_t rawCd);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_ANGLE_UTILS_H */
