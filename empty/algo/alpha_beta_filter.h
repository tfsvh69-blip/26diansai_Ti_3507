#ifndef ALGO_ALPHA_BETA_FILTER_H
#define ALGO_ALPHA_BETA_FILTER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 一维 α-β 滤波器。
 *
 * 用常速度模型从离散位置测量中同时估计位置和速度，适合视觉坐标这类低帧率、
 * 带少量抖动的反馈。算法只依赖浮点运算，不依赖硬件或 RTOS。
 */
typedef struct {
    float position;
    float velocity;
    float alpha;
    float beta;
    bool  initialized;
} AlphaBetaFilter_t;

/* 初始化滤波参数；alpha 推荐 0~1，beta 推荐使用较小正数。 */
void AlphaBetaFilter_Init(AlphaBetaFilter_t *filter, float alpha, float beta);

/* 清除历史状态，下一次更新会直接采用首个测量值并令速度为 0。 */
void AlphaBetaFilter_Reset(AlphaBetaFilter_t *filter);

/*
 * 输入最新位置测量和两帧实际间隔 dtSec，更新位置/速度估计。
 * dtSec 非正时忽略本次更新并保持原状态。
 */
void AlphaBetaFilter_Update(AlphaBetaFilter_t *filter, float measurement,
                            float dtSec);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_ALPHA_BETA_FILTER_H */
