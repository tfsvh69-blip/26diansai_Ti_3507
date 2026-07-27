#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_motor.h"
#include "diff_drive.h"

/* ==================================================================
 * 第 2 题：固定半径差速转弯测试
 *
 * 只需修改下面两个 T2_TURN_* 参数即可测试不同圆弧：
 *   1. T2_TURN_RADIUS_MM：圆心到小车几何中心的半径；正数左转，负数右转；
 *   2. T2_CENTER_RPM：小车中心的期望 RPM；正数前进，负数倒退；
 *
 * 进入后直接执行 DiffDrive_RunRadiusTurn(半径, 中心RPM)；电机命令均直接下发。
 * 小车持续绕圆弧行驶，按 K4 退出时立即急停。
 * ================================================================== */

/* 测试参数：+200 mm 表示前进左转；改为 -200 mm 即可前进右转。 */
#define T2_TURN_RADIUS_MM    (200)
#define T2_CENTER_RPM        (100)

/* 本题状态机：下发一次圆弧命令后保持行驶，等待 K4 退出。 */
typedef enum {
    T2_STATE_START = 0,  /* 计算半径对应的左右 RPM，并下发给四轮 */
    T2_STATE_TURN,       /* 持续圆弧行驶 */
    T2_STATE_DONE         /* 参数异常时停车 */
} Task2State_t;

static Task2State_t s_state;

void Task2_OnEnter(void)
{
    /* 进入测试前先急停，避免上一个题目的电机命令残留。 */
    BspMotor_EmergencyStop(BSP_MOTOR_1);
    BspMotor_EmergencyStop(BSP_MOTOR_2);
    BspMotor_EmergencyStop(BSP_MOTOR_3);
    BspMotor_EmergencyStop(BSP_MOTOR_4);
    BspMotor_EnableAll();
    s_state = T2_STATE_START;
}

void Task2_OnLoop(void)
{
    switch (s_state) {
    case T2_STATE_START:
        /* 只传入用户给定的圆弧半径和中心 RPM，模块负责计算并立即下发四轮。 */
        if (DiffDrive_RunRadiusTurn(T2_TURN_RADIUS_MM, T2_CENTER_RPM)) {
            s_state = T2_STATE_TURN;
        } else {
            /* 参数超出安全 RPM 范围或半径非法时不动作，保持停车。 */
            s_state = T2_STATE_DONE;
        }
        break;

    case T2_STATE_TURN:
        /* 命令已在 START 下发；持续转弯，K4 会调用 OnExit 急停。 */
        break;

    case T2_STATE_DONE:
        BspMotor_StopAll();
        break;

    default:
        break;
    }
}

void Task2_OnExit(void)
{
    /* K4 退出时立即急停并失能，避免圆弧测试继续运动。 */
    BspMotor_EmergencyStop(BSP_MOTOR_1);
    BspMotor_EmergencyStop(BSP_MOTOR_2);
    BspMotor_EmergencyStop(BSP_MOTOR_3);
    BspMotor_EmergencyStop(BSP_MOTOR_4);
    BspMotor_DisableAll();
    s_state = T2_STATE_START;
}
