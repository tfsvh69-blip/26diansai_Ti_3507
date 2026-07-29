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
 * ID2 已确认方向正确，ID3 已确认与期望正方向相反；ID1 尚待实机确认，暂不取反。
 */
static const bool s_dirInvertTable[EMM42_ROBOT_COUNT] = {
    false,  /* EMM42_ROBOT_LIFT    ID1，方向待定 */
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
