#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 标准 1.8° 步进电机每圈全步数，用于外部计算旋转圈数。 */
#define BSP_MOTOR_FULL_STEPS_PER_REV    (200U)

/*
 * 巡航速度对应的定时器周期（定时器时钟 4MHz，步频 = 4MHz / 周期）。
 * 周期越小越快；1/8 细分下 1600 脉冲 = 1 圈。
 * SLOW：2500 → 1.6kHz ≈ 1 圈/秒；FAST：500 → 8kHz ≈ 5 圈/秒。
 * 注意：巡航周期应不大于 bsp_motor.c 内的起步周期 MOTOR_STEP_PERIOD_START。
 */
#define BSP_MOTOR_PERIOD_SLOW           (2500U)
#define BSP_MOTOR_PERIOD_FAST           (500U)

/*
 * 步进电机 / TMC2209 板级封装（天猛星扩展板 v1.0）。
 * 四路驱动共用 ENN 使能和 MS1/MS2 细分；当前先实现电机1。
 * STEP 由 TIMG0_CCP0 硬件定时器输出，DIR/ENN/MS1/MS2 为普通 GPIO。
 */

/* TMC2209 细分档位，电平对应引脚文档 §3.3 的 MS1/MS2 表（四路共用）。 */
typedef enum {
    TMC_MICROSTEP_8 = 0,   /* MS1=0 MS2=0 */
    TMC_MICROSTEP_16,      /* MS1=1 MS2=1 */
    TMC_MICROSTEP_32,      /* MS1=1 MS2=0 */
    TMC_MICROSTEP_64       /* MS1=0 MS2=1 */
} BspTmcMicrostep_t;

/* 电机旋转方向，由 DIR 引脚电平决定（实际正反向以装配后实测为准）。 */
typedef enum {
    MOTOR_DIR_FORWARD = 0, /* DIR 低电平 */
    MOTOR_DIR_REVERSE      /* DIR 高电平 */
} BspMotorDir_t;

/* 进入上电安全状态：ENN 禁用、STEP 停止、DIR 正向、默认 1/8 细分。 */
void BspMotor_Init(void);

/* 设置四路共用细分档位。 */
void BspTmc_SetMicrostep(BspTmcMicrostep_t microstep);

/* ENN 低有效：使能/禁用全部四路 TMC2209。 */
void BspTmc_EnableAll(void);
void BspTmc_DisableAll(void);

/* 电机1方向与 STEP 脉冲输出控制。 */
void BspMotor1_SetDir(BspMotorDir_t dir);
void BspMotor1_StartStep(void);
void BspMotor1_StopStep(void);

/*
 * 定长步进（带梯形加减速）：启动后由 TIMG0 ZERO 中断倒计脉冲，数到 0 自动停止定时器。
 * steps：总脉冲数；cruisePeriod：巡航（最高速）周期，越小越快，用 BSP_MOTOR_PERIOD_SLOW/FAST。
 * 非阻塞，调用后用 BspMotor1_IsRotateDone() 轮询结果。
 * 注意：调用前必须已设置方向并使能 ENN，否则电机不动。
 */
void BspMotor1_StartRotateSteps(uint32_t steps, uint32_t cruisePeriod);
bool BspMotor1_IsRotateDone(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
