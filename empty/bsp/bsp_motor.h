#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ================== 步进电机 / TMC2209 板级封装（v1.9，四路完全独立）==================
 *
 * 小车用 4 个步进电机，每路各占一个独立硬件定时器输出 STEP，各自独立计步/调速：
 *   电机1  STEP=PB10(TIMG0)   DIR=PB11
 *   电机2  STEP=PB6 (TIMG8)   DIR=PB7
 *   电机3  STEP=PB13(TIMG12)  DIR=PB14
 *   电机4  STEP=PB26(TIMG6)   DIR=PB27
 *   四路共用：ENN=PA13(低有效使能)、MS1=PB0、MS2=PB1(细分)
 *
 * 【控制模型】每路一套直接执行的计步状态，由该路定时器的 ZERO 中断驱动：
 *   - STEP 由硬件定时器连续输出方波，步频 = 定时器时钟(4MHz) / 定时器周期；
 *   - 所有转速、定距与停止命令均直接下发，不做梯形加减速；
 *   - 全部接口非阻塞：调用后立即返回，定距计步在各自 ISR 后台完成。
 *
 * 【两个核心接口】（对应"指定速度/方向" 与 "指定脉冲/距离"）
 *   BspMotor_SetSpeedRpm(id, rpm)      连续旋转：rpm 正=正转/负=反转/0=立即停止
 *   BspMotor_MoveSteps(id, steps, rpm) 定距移动：走 |steps| 个脉冲后自动停，steps 符号定方向
 *
 * 转速单位为 RPM（转/分），内部按"当前细分"换算成脉冲频率，故换细分后 RPM 依然物理正确。
 * =====================================================================================
 */

/* 标准 1.8° 步进电机每圈全步数（未细分）。整圈脉冲数 = 该值 × 细分倍数。 */
#define BSP_MOTOR_FULL_STEPS_PER_REV    (200U)

/* 电机编号（与舵机 BspServoId_t 风格一致）。 */
typedef enum {
    BSP_MOTOR_1 = 0,   /* STEP=PB10/TIMG0，DIR=PB11 */
    BSP_MOTOR_2,       /* STEP=PB6 /TIMG8，DIR=PB7  */
    BSP_MOTOR_3,       /* STEP=PB13/TIMG12，DIR=PB14 */
    BSP_MOTOR_4,       /* STEP=PB26/TIMG6，DIR=PB27 */
    BSP_MOTOR_COUNT
} BspMotorId_t;

/* TMC2209 细分档位（四路共用），电平对应接线文档 §2.2 的 MS1/MS2 表。 */
typedef enum {
    TMC_MICROSTEP_8 = 0,   /* MS1=0 MS2=0 → 每圈  1600 脉冲 */
    TMC_MICROSTEP_16,      /* MS1=1 MS2=1 → 每圈  3200 脉冲 */
    TMC_MICROSTEP_32,      /* MS1=1 MS2=0 → 每圈  6400 脉冲（默认） */
    TMC_MICROSTEP_64       /* MS1=0 MS2=1 → 每圈 12800 脉冲 */
} BspTmcMicrostep_t;

/* ==================== 初始化 / 全局控制 ==================== */

/* 上电安全态：禁用四路驱动、四路 STEP 停表、DIR 归已标定的逻辑正向、默认 1/32 细分。 */
void BspMotor_Init(void);

/* ENN 低有效：一次使能/禁用全部四路 TMC2209（禁用后电机失力）。起转前必须先 EnableAll。 */
void BspMotor_EnableAll(void);
void BspMotor_DisableAll(void);

/* 设置四路共用细分档位（会影响 RPM↔脉冲频率换算；运行中不建议切换）。 */
void BspMotor_SetMicrostep(BspTmcMicrostep_t microstep);

/*
 * 方向取反（按需一次性标定）：小车左右两侧电机机械镜像安装时，
 * 给某几路设 invert=true，即可让"正 rpm / 正 steps = 小车前进"对四路统一成立。
 * 当前实车标定为 M1/M2=true、M3/M4=false；invert 只改变 DIR 电平方向，
 * 不影响 rpm/steps 的数值含义。
 */
void BspMotor_SetDirInvert(BspMotorId_t id, bool invert);

/* ==================== 核心：速度 / 脉冲两个接口 ==================== */

/*
 * (a) 指定速度和方向（连续旋转，非阻塞）：
 *     rpm > 0  正转、rpm < 0  反转、rpm == 0  立即停止。
 *     - 任意新命令都会先停表，再按新方向和目标 RPM 直接重新输出 STEP；
 *     - 不做梯形加减速，适合当前硬件和函数测试阶段。
 *     转速会被限幅到硬件安全范围（约 5~300 RPM @1/32；超出则贴到最慢/最快）。
 *     注意：调用前须先 BspMotor_EnableAll()，否则驱动未使能电机不动。
 */
void BspMotor_SetSpeedRpm(BspMotorId_t id, int32_t rpm);

/*
 * 与 BspMotor_SetSpeedRpm 行为相同，保留此接口以兼容已有调用。
 * 所有电机速度命令均为立即下发；rpm=0 为立即停表。
 */
void BspMotor_SetSpeedRpmImmediate(BspMotorId_t id, int32_t rpm);

/*
 * (b) 指定脉冲 / 定距移动（非阻塞，走完立即停表）：
 *     steps 符号定方向（正=正转、负=反转），|steps| 为要走的 STEP 脉冲数；
 *     rpm 为巡航转速大小（取绝对值，方向以 steps 为准）。
 *     STEP 以目标 RPM 直接输出；计步由 ISR 完成，最后一个脉冲后立即停表。
 *     建议在该电机停稳时调用；若正在连续旋转，会先急停再从头精确走 steps 步。
 *     整圈脉冲数 = BspMotor_StepsPerRev()（如 1/32 细分 = 6400）。
 */
void BspMotor_MoveSteps(BspMotorId_t id, int32_t steps, uint32_t rpm);

/* 便捷：一次给四路设连续转速（差速/整车驱动常用）。参数含义同 SetSpeedRpm。 */
void BspMotor_SetSpeedRpm4(int32_t rpm1, int32_t rpm2, int32_t rpm3, int32_t rpm4);

/* 便捷：四路直接下发连续 RPM。参数含义同 SetSpeedRpmImmediate。 */
void BspMotor_SetSpeedRpm4Immediate(int32_t rpm1, int32_t rpm2,
                                    int32_t rpm3, int32_t rpm4);

/*
 * 便捷：一次给四路各自定距移动（参数含义同 MoveSteps，rpm 为四路共用巡航转速大小）。
 * 直行 = 四个 steps 相同；原地转弯 = 左右两组 steps 反号。省去题目层手写四次调用。
 */
void BspMotor_MoveSteps4(int32_t steps1, int32_t steps2, int32_t steps3, int32_t steps4,
                         uint32_t rpm);

/* ==================== 停止 ==================== */

/* 立即停止指定电机（等价 SetSpeedRpm(id,0)）。 */
void BspMotor_Stop(BspMotorId_t id);

/* 立即停止全部四路。 */
void BspMotor_StopAll(void);

/* 立即急停指定电机（直接停表）；可用于异常保护/复位。 */
void BspMotor_EmergencyStop(BspMotorId_t id);

/* ==================== 状态查询（供状态机判断动作是否完成） ==================== */

/* 该电机是否已完全停下（连续停稳或定距走完）。 */
bool BspMotor_IsStopped(BspMotorId_t id);

/* 四路是否都已停下（整车"动作是否全部完成"的常用判断）。 */
bool BspMotor_AllStopped(void);

/* 定距模式下剩余未走的脉冲数（连续模式或已停返回 0），用于判断"走完没有"。 */
uint32_t BspMotor_GetRemainingSteps(BspMotorId_t id);

/* 当前细分下每圈脉冲数 = 200 × 细分倍数。方便题目把"圈/角度"换算成 steps。 */
uint32_t BspMotor_StepsPerRev(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
