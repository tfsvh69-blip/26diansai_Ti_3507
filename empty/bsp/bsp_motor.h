#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 标准 1.8° 步进电机每圈全步数，用于外部计算转速。 */
#define BSP_MOTOR_FULL_STEPS_PER_REV    (200U)

/*
 * 巡航速度对应的定时器周期（定时器时钟 4MHz，步频 = 4MHz / 周期）。
 * 周期越小越快；1/32 细分下 6400 脉冲 = 1 圈。
 *   周期 625 → 6.4kHz  ≈ 1 圈/秒（最慢档）
 *   周期 125 → 32kHz   ≈ 5 圈/秒（最快档）
 * 连续旋转任务在 [MIN, START] 之间分级取速；MIN 为硬件安全上限，勿再调小。
 */
#define BSP_MOTOR_PERIOD_SLOW           (625U)
#define BSP_MOTOR_PERIOD_FAST           (125U)
#define BSP_MOTOR_PERIOD_MIN            (125U)   /* 最快巡航周期（步频上限） */

/*
 * 步进电机 / TMC2209 板级封装（v1.1）。
 * 电机1：STEP=PB10(TIMG0_CCP0)、DIR=PB11；四路共用 ENN=PA13、MS1=PB0、MS2=PB1。
 * 当前先实现电机1；STEP 由硬件定时器输出，DIR/ENN/MS1/MS2 为普通 GPIO。
 * 采用「连续旋转 + 在线调速调向」模型：TIMG0 ZERO 中断持续输出脉冲，
 * 目标速度由任务在线设定，ISR 用梯形加减速平滑过渡，避免高速直接起转/切速失步。
 */

/* TMC2209 细分档位，电平对应引脚文档 §2.2 的 MS1/MS2 表（四路共用）。 */
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

/* 进入上电安全状态：ENN 禁用、STEP 停止、DIR 正向、默认 1/32 细分。 */
void BspMotor_Init(void);

/* 设置四路共用细分档位。 */
void BspTmc_SetMicrostep(BspTmcMicrostep_t microstep);

/* ENN 低有效：使能/禁用全部四路 TMC2209。 */
void BspTmc_EnableAll(void);
void BspTmc_DisableAll(void);

/* 电机1方向控制；仅在停止时切换方向，运行中换向请先 RequestStop 停稳。 */
void BspMotor1_SetDir(BspMotorDir_t dir);

/* 立即停止 STEP 定时器（不带减速），仅供初始化收敛安全状态使用。 */
void BspMotor1_StopStep(void);

/*
 * 连续旋转控制（非阻塞，配合 ISR 梯形加减速）：
 *   RunContinuous：起转或在线设定目标巡航速度（cruisePeriod 越小越快）。
 *                  未运行时从慢速起步平滑加速；运行中等同在线调速。
 *   SetSpeed     ：运行中平滑改变目标速度（不改变启停）。
 *   RequestStop  ：请求平滑减速到起步速度后自动停表。
 *   IsStopped    ：是否已完全停下（可用于换向前等待停稳）。
 * cruisePeriod 会被限幅到 [BSP_MOTOR_PERIOD_MIN, 起步周期]。
 * 注意：起转前必须已 BspTmc_EnableAll() 并设好方向，否则电机不动。
 */
void BspMotor1_RunContinuous(uint32_t cruisePeriod);
void BspMotor1_SetSpeed(uint32_t cruisePeriod);
void BspMotor1_RequestStop(void);

/*
 * 定步数位置移动（配合 ISR 梯形加减速，自动停表）：
 *   MoveSteps：走 steps 个 STEP 脉冲后自动平滑停止。
 *              cruisePeriod 越小越快，会被限幅到 [PERIOD_MIN, 起步周期]。
 *              起转/巡航/停止全程梯形加减速，不丢步。
 *              注意：起转前必须已 BspTmc_EnableAll() 并设好方向。
 *   GetRemainingSteps：剩余未走的脉冲数（诊断用）。
 *   IsStopped        ：是否已完全停下（位置移动完成或连续旋转停止）。
 *
 * 步数换算（1/32 细分）：1 圈 = 200 × 32 = 6400 脉冲。
 */
void BspMotor1_MoveSteps(uint32_t steps, uint32_t cruisePeriod);
uint32_t BspMotor1_GetRemainingSteps(void);
bool BspMotor1_IsStopped(void);

/*
 * 四电机"一起转"接口（电机测试用）：电机2/3/4(TIMG8/12/6)跟随电机1(TIMG0)，
 * 同频同向一起转，共用同一套梯形斜坡（TIMG0 ZERO 中断为主控）。
 *   BspMotorAll_SetDir   ：四路 DIR 同时设向（低=正向）。
 *   BspMotorAll_MoveSteps ：四电机一起走 steps 个 STEP 脉冲(定圈)后自动减速停表。
 * 起转前须先 BspTmc_EnableAll()；停/转判定复用 BspMotor1_IsStopped()。
 */
void BspMotorAll_SetDir(BspMotorDir_t dir);
void BspMotorAll_MoveSteps(uint32_t steps, uint32_t cruisePeriod, BspMotorDir_t dir);

/*
 * 诊断只读接口：
 *   GetCurPeriod：当前定时器周期。若从起步值逐步变到巡航值，
 *                 说明 TIMG0 ZERO 中断在跑、定时器在计数、STEP 一定有脉冲输出。
 *   IsEnabled   ：读 ENN 引脚输出电平，低=已使能驱动。
 */
uint32_t BspMotor1_GetCurPeriod(void);
bool BspTmc_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
