#ifndef ALGO_PID_H
#define ALGO_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 通用 PID 控制器（浮点实现）
 *
 * 适用于 Cortex-M0+ 80MHz，30ms~1ms 控制周期的场景。提供：
 *   - P / I / D 三项独立增益
 *   - 梯形积分 + 抗饱和（integralLimit）
 *   - 输出限幅（outputLimit）
 *   - 运行时在线调参（Pid_SetGains / Pid_SetLimits）
 *
 * 典型用法：
 *   Pid_t pid;
 *   Pid_Init(&pid, 1.5f, 0.02f, 0.2f, 40.0f, 120.0f);
 *   float output = Pid_Update(&pid, target - current, dtSec);
 *
 * 纯算法，不依赖任何硬件或 RTOS。
 */

/* PID 控制器状态。字段可直接读写以实现在线调参或复位。 */
typedef struct {
    float kp;             /* 比例增益 */
    float ki;             /* 积分增益 */
    float kd;             /* 微分增益 */
    float integral;       /* 积分累计值 */
    float prevError;      /* 上一次误差（用于微分项） */
    float integralLimit;  /* 积分抗饱和限幅（绝对值） */
    float outputLimit;    /* 输出限幅（绝对值） */
} Pid_t;

/*
 * 用给定的增益和限幅初始化 PID 控制器（积分归零、prevError 归零）。
 * integralLimit 和 outputLimit 均为绝对值，内部自动对称限幅。
 */
void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float integralLimit, float outputLimit);

/* 重置积分和上一次误差，保留增益和限幅设置。 */
void Pid_Reset(Pid_t *pid);

/*
 * 运行时修改增益（不改变积分/微分状态和限幅）。
 * 传 NULL 的项保持原值不变。
 */
void Pid_SetGains(Pid_t *pid, const float *kp, const float *ki, const float *kd);

/*
 * 运行时修改限幅。
 * 传 NULL 的项保持原值不变。设限幅后若积分/输出超出新范围会立即钳位。
 */
void Pid_SetLimits(Pid_t *pid, const float *integralLimit, const float *outputLimit);

/*
 * 单步更新：传入 error = target − current 和时间步长 dtSec（秒），返回控制量。
 * dtSec 应使用实际或名义控制周期；Pid_Init 后首次调用自动跳过微分项。
 */
float Pid_Update(Pid_t *pid, float error, float dtSec);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_PID_H */
