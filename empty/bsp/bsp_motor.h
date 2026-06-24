#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
