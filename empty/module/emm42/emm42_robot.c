#include "emm42_robot.h"

#include "emm42_v5.h"

/*
 * 角色 → 协议地址映射表，下标即 Emm42RobotId_t。
 * 只在这一处集中记录"哪个地址是哪个部件"，其余代码一律用角色枚举，
 * 避免到处出现裸的 1/2/3 数字。
 */
static const uint8_t s_addrTable[EMM42_ROBOT_COUNT] = {
    EMM42_ADDR_MOTOR1,   /* EMM42_ROBOT_LIFT    摆杆高低调节 */
    EMM42_ADDR_MOTOR2,   /* EMM42_ROBOT_WHEEL_L 左轮 */
    EMM42_ADDR_MOTOR3,   /* EMM42_ROBOT_WHEEL_R 右轮 */
};

/*
 * 角色正方向标定表：1 表示将上层 RPM 符号取反后再下发协议层。
 * ID1 已实测确认：正方向为连杆向下；ID2 已确认方向正确，ID3 已确认与期望
 * 正方向相反。
 */
static const bool s_dirInvertTable[EMM42_ROBOT_COUNT] = {
    false,  /* EMM42_ROBOT_LIFT    ID1，正方向=连杆向下 */
    false,  /* EMM42_ROBOT_WHEEL_L ID2，方向正确 */
    true,   /* EMM42_ROBOT_WHEEL_R ID3，方向取反 */
};

void Emm42Robot_Init(void)
{
    Emm42_Init();
}

void Emm42Robot_Enable(Emm42RobotId_t id, bool enable)
{
    if (id >= EMM42_ROBOT_COUNT) {
        return;
    }
    Emm42_Enable(s_addrTable[id], enable, false);
}

void Emm42Robot_SetSpeedRpm(Emm42RobotId_t id, int16_t rpm, uint8_t acc)
{
    if (id >= EMM42_ROBOT_COUNT) {
        return;
    }
    if (s_dirInvertTable[id]) {
        rpm = (int16_t)-rpm;
    }
    Emm42_SetSpeedRpm(s_addrTable[id], rpm, acc);
}

void Emm42Robot_VelControl(Emm42RobotId_t id, int16_t rpm, uint8_t acc)
{
    int32_t    signedRpm;
    Emm42Dir_t dir;
    uint16_t   magnitude;

    if (id >= EMM42_ROBOT_COUNT) {
        return;
    }

    signedRpm = (int32_t)rpm;
    if (s_dirInvertTable[id]) {
        signedRpm = -signedRpm;
    }

    dir = (signedRpm < 0) ? EMM42_DIR_CCW : EMM42_DIR_CW;
    magnitude = (uint16_t)((signedRpm < 0) ? -signedRpm : signedRpm);
    Emm42_VelControl(s_addrTable[id], dir, magnitude, acc, false);
}

void Emm42Robot_MoveRelative(Emm42RobotId_t id, int32_t pulses, uint16_t rpm,
                             uint8_t acc)
{
    int64_t     signedPulses;
    Emm42Dir_t  dir;
    uint32_t    magnitude;

    if ((id >= EMM42_ROBOT_COUNT) || (pulses == 0) || (rpm == 0U)) {
        return;
    }

    signedPulses = (int64_t)pulses;
    if (s_dirInvertTable[id]) {
        signedPulses = -signedPulses;
    }

    dir = (signedPulses < 0) ? EMM42_DIR_CCW : EMM42_DIR_CW;
    magnitude = (uint32_t)((signedPulses < 0) ? -signedPulses : signedPulses);
    Emm42_PosControl(s_addrTable[id], dir, rpm, acc, magnitude, false, false);
}

void Emm42Robot_MoveAbsolute(Emm42RobotId_t id, int32_t targetPulses,
                             uint16_t rpm, uint8_t acc)
{
    int64_t     signedPulses;
    Emm42Dir_t  dir;
    uint32_t    magnitude;

    /*
     * 与 MoveRelative 不同：这里【不能】把 targetPulses==0 当成"不动"提前返回，
     * 绝对模式下 0 表示回到位置原点，是最常用的目标值。
     */
    if ((id >= EMM42_ROBOT_COUNT) || (rpm == 0U)) {
        return;
    }

    /*
     * 角色方向标定只翻转正负方向，不改变位置原点：原点由驱动器内部维护，
     * 上层无论怎么标定方向，绝对目标 0 始终对应 ResetPosToZero 时的物理位置。
     */
    signedPulses = (int64_t)targetPulses;
    if (s_dirInvertTable[id]) {
        signedPulses = -signedPulses;
    }

    /* 绝对模式下协议的"方向"字节表示目标绝对位置的符号，脉冲数取绝对值。 */
    dir = (signedPulses < 0) ? EMM42_DIR_CCW : EMM42_DIR_CW;
    magnitude = (uint32_t)((signedPulses < 0) ? -signedPulses : signedPulses);
    Emm42_PosControl(s_addrTable[id], dir, rpm, acc, magnitude, true, false);
}

void Emm42Robot_ResetPosToZero(Emm42RobotId_t id)
{
    if (id >= EMM42_ROBOT_COUNT) {
        return;
    }
    Emm42_ResetCurPosToZero(s_addrTable[id]);
}

void Emm42Robot_ClearClogProtection(Emm42RobotId_t id)
{
    if (id >= EMM42_ROBOT_COUNT) {
        return;
    }
    Emm42_ResetClogProtection(s_addrTable[id]);
}

void Emm42Robot_Stop(Emm42RobotId_t id)
{
    if (id >= EMM42_ROBOT_COUNT) {
        return;
    }
    Emm42_StopNow(s_addrTable[id], false);
}

void Emm42Robot_StopAll(void)
{
    /* 背靠背 3 帧：仅用于 OnExit 一次性安全收尾，见头文件说明，不要挪到 OnLoop。 */
    uint32_t i;
    for (i = 0U; i < (uint32_t)EMM42_ROBOT_COUNT; i++) {
        Emm42_StopNow(s_addrTable[i], false);
    }
}

uint8_t Emm42Robot_GetAddr(Emm42RobotId_t id)
{
    if (id >= EMM42_ROBOT_COUNT) {
        return EMM42_ADDR_BROADCAST;
    }
    return s_addrTable[id];
}
