#include "alpha_beta_filter.h"

#include <stddef.h>

static float AlphaBetaFilter_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

void AlphaBetaFilter_Init(AlphaBetaFilter_t *filter, float alpha, float beta)
{
    if (filter == NULL) {
        return;
    }

    filter->alpha = AlphaBetaFilter_Clamp(alpha, 0.0F, 1.0F);
    filter->beta = AlphaBetaFilter_Clamp(beta, 0.0F, 1.0F);
    AlphaBetaFilter_Reset(filter);
}

void AlphaBetaFilter_Reset(AlphaBetaFilter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->position = 0.0F;
    filter->velocity = 0.0F;
    filter->initialized = false;
}

void AlphaBetaFilter_Update(AlphaBetaFilter_t *filter, float measurement,
                            float dtSec)
{
    float predictedPosition;
    float residual;

    if ((filter == NULL) || (dtSec <= 0.0F)) {
        return;
    }

    if (!filter->initialized) {
        filter->position = measurement;
        filter->velocity = 0.0F;
        filter->initialized = true;
        return;
    }

    predictedPosition = filter->position + filter->velocity * dtSec;
    residual = measurement - predictedPosition;

    filter->position = predictedPosition + filter->alpha * residual;
    filter->velocity = filter->velocity + filter->beta * residual / dtSec;
}
