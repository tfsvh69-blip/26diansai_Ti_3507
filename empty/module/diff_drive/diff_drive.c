#include "diff_drive.h"

#include <stddef.h>

#include "bsp_motor.h"

/* 当前小车的实测几何尺寸。 */
const DiffDriveGeometry_t g_diffDriveDefaultGeometry = {
    201.0F,    /* 左右轮胎中心距 */
    201.0F,    /* 前后轮胎中心距 */
    27.0F      /* 轮胎宽度 */
};

/* 返回浮点数绝对值，避免引入 math 库。 */
static float DiffDrive_Abs(float value)
{
    return (value < 0.0F) ? -value : value;
}

/* 将 RPM 四舍五入为整数；本模块所有输入已限制在安全速度范围内。 */
static int32_t DiffDrive_RoundRpm(float rpm)
{
    return (rpm >= 0.0F) ? (int32_t)(rpm + 0.5F) : (int32_t)(rpm - 0.5F);
}

/* 判断非零 RPM 是否低于步进电机可稳定运行的最低速度。 */
static bool DiffDrive_IsTooSlow(int32_t rpm)
{
    int32_t absRpm = (rpm < 0) ? -rpm : rpm;

    return (absRpm != 0) && ((uint32_t)absRpm < DIFF_DRIVE_MIN_NONZERO_RPM);
}

bool DiffDrive_CalcRadiusTurn(const DiffDriveGeometry_t *geometry,
                               int32_t radiusMm,
                               int32_t centerRpm,
                               uint32_t maxWheelRpm,
                               DiffDriveWheelRpm_t *wheelRpm)
{
    float leftRpm;
    float rightRpm;
    float peakRpm;
    float scale = 1.0F;

    if ((geometry == NULL) || (wheelRpm == NULL) || (radiusMm == 0) ||
        (geometry->trackWidthMm <= 0.0F) ||
        (maxWheelRpm < DIFF_DRIVE_MIN_NONZERO_RPM) ||
        (maxWheelRpm > DIFF_DRIVE_MAX_SAFE_RPM)) {
        return false;
    }

    /*
     * 圆弧运动学：
     * 左侧速度  = 中心速度 × (R - W/2) / R；
     * 右侧速度  = 中心速度 × (R + W/2) / R。
     * R 为带符号半径，W 为左右轮中心距。R 为负时公式自然交换内外轮。
     */
    leftRpm = (float)centerRpm *
              (((float)radiusMm - geometry->trackWidthMm * 0.5F) / (float)radiusMm);
    rightRpm = (float)centerRpm *
               (((float)radiusMm + geometry->trackWidthMm * 0.5F) / (float)radiusMm);

    /* 限制外轮速度时，左右两侧同比例缩小，速度比例和目标圆弧半径不变。 */
    peakRpm = DiffDrive_Abs(leftRpm);
    if (DiffDrive_Abs(rightRpm) > peakRpm) {
        peakRpm = DiffDrive_Abs(rightRpm);
    }
    if (peakRpm > (float)maxWheelRpm) {
        scale = (float)maxWheelRpm / peakRpm;
        leftRpm *= scale;
        rightRpm *= scale;
    }

    wheelRpm->leftRpm  = DiffDrive_RoundRpm(leftRpm);
    wheelRpm->rightRpm = DiffDrive_RoundRpm(rightRpm);
    wheelRpm->speedScalePermille = (uint16_t)(scale * 1000.0F + 0.5F);

    /* 不让 BSP 把一侧的低速悄悄钳到最小 RPM，否则实际半径会偏离计算值。 */
    if (DiffDrive_IsTooSlow(wheelRpm->leftRpm) ||
        DiffDrive_IsTooSlow(wheelRpm->rightRpm)) {
        return false;
    }
    return true;
}

void DiffDrive_CalcStraight(int32_t centerRpm, DiffDriveWheelRpm_t *wheelRpm)
{
    if (wheelRpm == NULL) {
        return;
    }

    wheelRpm->leftRpm           = centerRpm;
    wheelRpm->rightRpm          = centerRpm;
    wheelRpm->speedScalePermille = 1000U;
}

void DiffDrive_CalcPivotTurn(bool turnLeft, int32_t wheelRpm,
                              DiffDriveWheelRpm_t *command)
{
    if (command == NULL) {
        return;
    }

    if (turnLeft) {
        command->leftRpm  = -wheelRpm;
        command->rightRpm = wheelRpm;
    } else {
        command->leftRpm  = wheelRpm;
        command->rightRpm = -wheelRpm;
    }
    command->speedScalePermille = 1000U;
}

void DiffDrive_ApplyWheelRpm(const DiffDriveWheelRpm_t *wheelRpm)
{
    if (wheelRpm == NULL) {
        return;
    }

    /* M1/M2 为左侧，M3/M4 为右侧；BSP 已完成 M1/M2 方向取反标定。 */
    BspMotor_SetSpeedRpm4(wheelRpm->leftRpm, wheelRpm->leftRpm,
                          wheelRpm->rightRpm, wheelRpm->rightRpm);
}

void DiffDrive_ApplyWheelRpmImmediate(const DiffDriveWheelRpm_t *wheelRpm)
{
    if (wheelRpm == NULL) {
        return;
    }

    /* M1/M2 为左侧，M3/M4 为右侧；立即模式不经过 BSP 梯形加减速。 */
    BspMotor_SetSpeedRpm4Immediate(wheelRpm->leftRpm, wheelRpm->leftRpm,
                                   wheelRpm->rightRpm, wheelRpm->rightRpm);
}

bool DiffDrive_RunRadiusTurn(int32_t radiusMm, int32_t centerRpm)
{
    DiffDriveWheelRpm_t wheelRpm;

    /* 先按最高安全 RPM 计算；若需要缩放，说明用户给定的中心 RPM 不能原样执行。 */
    if (!DiffDrive_CalcRadiusTurn(&g_diffDriveDefaultGeometry,
                                   radiusMm,
                                   centerRpm,
                                   DIFF_DRIVE_MAX_SAFE_RPM,
                                   &wheelRpm) ||
        (wheelRpm.speedScalePermille != 1000U)) {
        return false;
    }

    DiffDrive_ApplyWheelRpmImmediate(&wheelRpm);
    return true;
}
