#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 四轮差速底盘模块
 *
 * M1/M2 为左侧两轮，M3/M4 为右侧两轮。同一侧给相同 RPM 后，四轮固定、
 * 无阿克曼转向的底盘可按左右两组轮的差速模型控制。
 *
 * 本模块的半径均指“小车几何中心到圆心”的半径，单位 mm：
 *   radiusMm > 0：左转，左侧为内轮、右侧为外轮；
 *   radiusMm < 0：右转，右侧为内轮、左侧为外轮；
 *   radiusMm = 0：不属于圆弧，需改用 DiffDrive_CalcPivotTurn() 原地转向。
 */

/* 当前 BSP 在 1/32 细分下的推荐连续运行范围，超出会被底层限幅。 */
#define DIFF_DRIVE_MIN_NONZERO_RPM    (5U)
#define DIFF_DRIVE_MAX_SAFE_RPM        (300U)

/*
 * 底盘几何参数。
 * trackWidthMm 是差速运动学真正使用的左右轮中心距；wheelBaseMm 和 tireWidthMm
 * 记录四轮固定底盘的实际尺寸，当前公式不直接使用它们，但它们会影响侧滑量，
 * 便于后续实测后把 trackWidthMm 修正为“有效轮距”。
 */
typedef struct {
    float trackWidthMm;   /* 左右轮胎中心距，当前实测 201 mm。 */
    float wheelBaseMm;    /* 前后轮胎中心距，当前实测 201 mm。 */
    float tireWidthMm;    /* 单个轮胎宽度，当前实测 27 mm。 */
} DiffDriveGeometry_t;

/* 当前小车的标称几何尺寸。若实测圆弧半径有系统误差，可复制后修改 trackWidthMm。 */
extern const DiffDriveGeometry_t g_diffDriveDefaultGeometry;

/*
 * 左右侧目标 RPM。左侧值将发给 M1/M2，右侧值将发给 M3/M4。
 * speedScalePermille 表示为遵守最大 RPM 而对整组速度做的等比缩放：
 * 1000=未缩放，800=所有 RPM 同比缩为原来的 80%。
 */
typedef struct {
    int32_t  leftRpm;
    int32_t  rightRpm;
    uint16_t speedScalePermille;
} DiffDriveWheelRpm_t;

/*
 * 按指定圆弧计算左右侧 RPM（只计算，不会驱动电机）。
 *
 * centerRpm 为车体中心沿圆弧前进时对应的轮 RPM：正数前进、负数倒退。
 * maxWheelRpm 是允许的单轮最大绝对 RPM，函数会在需要时等比降低左右 RPM，
 * 从而保持转弯半径不变。它必须位于 DIFF_DRIVE_MIN_NONZERO_RPM 到
 * DIFF_DRIVE_MAX_SAFE_RPM 之间。
 *
 * 返回 false 的情况：空指针、轮距或半径非法、最大 RPM 非法，或者计算后有一侧
 * 非零速度低于步进电机最小稳定 RPM。后者应降低半径绝对值或提高 centerRpm。
 */
bool DiffDrive_CalcRadiusTurn(const DiffDriveGeometry_t *geometry,
                               int32_t radiusMm,
                               int32_t centerRpm,
                               uint32_t maxWheelRpm,
                               DiffDriveWheelRpm_t *wheelRpm);

/* 计算直行命令：左右侧均为 centerRpm，不会驱动电机。 */
void DiffDrive_CalcStraight(int32_t centerRpm, DiffDriveWheelRpm_t *wheelRpm);

/*
 * 计算原地转向命令：turnLeft=true 时左侧后退、右侧前进；false 则相反。
 * wheelRpm 的绝对值必须由调用者保证在安全范围内。
 */
void DiffDrive_CalcPivotTurn(bool turnLeft, int32_t wheelRpm,
                              DiffDriveWheelRpm_t *command);

/*
 * 将左右侧 RPM 一次下发到四路电机：M1/M2=左侧，M3/M4=右侧。
 * 底层 BspMotor 自带梯形加减速；重复调用同向的新命令时会平滑调速。
 */
void DiffDrive_ApplyWheelRpm(const DiffDriveWheelRpm_t *wheelRpm);

/* 将左右侧 RPM 立即下发到四路电机，跳过 BSP 梯形加减速。 */
void DiffDrive_ApplyWheelRpmImmediate(const DiffDriveWheelRpm_t *wheelRpm);

/*
 * 最简圆弧执行接口：给定半径和中心 RPM 后立即下发四轮命令。
 * 它固定使用 g_diffDriveDefaultGeometry，且不做速度缩放：如果计算出的任一轮
 * 超过 300 RPM 或低于最小稳定 RPM，函数返回 false 且不会驱动电机。
 * 适合任务代码直接写 DiffDrive_RunRadiusTurn(半径, 中心RPM)。
 */
bool DiffDrive_RunRadiusTurn(int32_t radiusMm, int32_t centerRpm);

#ifdef __cplusplus
}
#endif

#endif /* DIFF_DRIVE_H */
