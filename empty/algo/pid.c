#include "pid.h"

#include <stddef.h>

void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float integralLimit, float outputLimit)
{
    if (pid == NULL) {
        return;
    }

    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integralLimit  = (integralLimit >= 0.0F) ? integralLimit : -integralLimit;
    pid->outputLimit    = (outputLimit >= 0.0F) ? outputLimit : -outputLimit;
    Pid_Reset(pid);
}

void Pid_Reset(Pid_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral  = 0.0F;
    pid->prevError = 0.0F;
}

void Pid_SetGains(Pid_t *pid, const float *kp, const float *ki, const float *kd)
{
    if (pid == NULL) {
        return;
    }
    if (kp != NULL) { pid->kp = *kp; }
    if (ki != NULL) { pid->ki = *ki; }
    if (kd != NULL) { pid->kd = *kd; }
}

void Pid_SetLimits(Pid_t *pid, const float *integralLimit, const float *outputLimit)
{
    if (pid == NULL) {
        return;
    }
    if (integralLimit != NULL) {
        float lim = (*integralLimit >= 0.0F) ? *integralLimit : -*integralLimit;
        pid->integralLimit = lim;
        /* 新限幅可能比当前累计值小，立即钳位。 */
        if (pid->integral > lim)  { pid->integral = lim; }
        if (pid->integral < -lim) { pid->integral = -lim; }
    }
    if (outputLimit != NULL) {
        float lim = (*outputLimit >= 0.0F) ? *outputLimit : -*outputLimit;
        pid->outputLimit = lim;
    }
}

float Pid_Update(Pid_t *pid, float error, float dtSec)
{
    float p, i, d, output;

    if (pid == NULL) {
        return 0.0F;
    }

    /* 比例项 */
    p = pid->kp * error;

    /* 积分项：梯形积分 + 抗饱和 */
    pid->integral += error * dtSec;
    if (pid->integral > pid->integralLimit) {
        pid->integral = pid->integralLimit;
    } else if (pid->integral < -pid->integralLimit) {
        pid->integral = -pid->integralLimit;
    }
    i = pid->ki * pid->integral;

    /* 微分项：误差变化率，首次调用 (prevError=0) 相当于只用 P+I */
    if (dtSec > 0.0F) {
        d = pid->kd * (error - pid->prevError) / dtSec;
    } else {
        d = 0.0F;
    }
    pid->prevError = error;

    /* 合成 + 输出限幅 */
    output = p + i + d;
    if (output > pid->outputLimit) {
        output = pid->outputLimit;
    } else if (output < -pid->outputLimit) {
        output = -pid->outputLimit;
    }

    return output;
}
