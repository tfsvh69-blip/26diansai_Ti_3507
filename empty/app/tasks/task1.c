#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_motor.h"
/* 需要用到时再解开：#include "bsp_servo.h" / "app_imu_uart_task.h" / "laser_ld14.h" / "ball_parser.h" */

/* ==================================================================
 * 第 1 题：四轮正方向与安装位置核对
 *
 * 进入后按 M1→M2→M3→M4 的顺序，每次只让一只轮子正向转两圈；
 * 每路完全停稳后才开始下一路，四路完成后保持停车。用于确认：
 *   M1=左前，M2=左后，M3=右前，M4=右后。
 *
 * 当前“正方向”由 BSP 全局标定：M1/M2 已取反、M3/M4 保持原方向，
 * 因此正 steps 对四路都表示小车前进方向；不在本题临时交换电机编号。
 *
 * 调用节拍：OnLoop 每 30ms 被 UIMENU 调一次；机动级决策足够。
 * ================================================================== */

/* 单轮测试参数：低速定距转两整圈，便于观察方向且避免车辆明显窜动。 */
#define T1_DIR_TEST_REVS    (2U)
#define T1_DIR_TEST_RPM     (60U)

/* 本题状态机：一次只允许一台电机运动。 */
typedef enum {
    T1_STATE_START = 0,   /* 对当前电机下发正向定距指令 */
    T1_STATE_WAIT,        /* 等待当前电机完全停稳 */
    T1_STATE_DONE         /* 四路均已测试，保持停车 */
} Task1State_t;

static Task1State_t  s_state;
static BspMotorId_t s_testMotor;

void Task1_OnEnter(void)
{
    /* 进入测试前先确保四路停止，再使能驱动并从 M1 开始。 */
    BspMotor_StopAll();
    BspMotor_EnableAll();
    s_testMotor = BSP_MOTOR_1;
    s_state     = T1_STATE_START;
}

void Task1_OnLoop(void)
{
    switch (s_state) {
    case T1_STATE_START: {
        int32_t steps = (int32_t)((uint32_t)T1_DIR_TEST_REVS * BspMotor_StepsPerRev());

        /* 正 steps 代表待标定的正方向；其余三路始终保持停止。 */
        BspMotor_MoveSteps(s_testMotor, steps, T1_DIR_TEST_RPM);
        s_state = T1_STATE_WAIT;
        break;
    }

    case T1_STATE_WAIT:
        /* 当前轮停稳后，才切换到下一编号，保证观察时不会混淆。 */
        if (BspMotor_IsStopped(s_testMotor)) {
            if (s_testMotor < BSP_MOTOR_4) {
                s_testMotor = (BspMotorId_t)((uint32_t)s_testMotor + 1U);
                s_state = T1_STATE_START;
            } else {
                s_state = T1_STATE_DONE;
            }
        }
        break;

    case T1_STATE_DONE:
        /* 完成后不重复测试，等待用户观察并通过 K4 退出。 */
        BspMotor_StopAll();
        break;

    default:
        break;
    }
}

void Task1_OnExit(void)
{
    /* K4 退出时立即急停并失能，防止测试被中断后仍有轮子转动。 */
    BspMotor_EmergencyStop(BSP_MOTOR_1);
    BspMotor_EmergencyStop(BSP_MOTOR_2);
    BspMotor_EmergencyStop(BSP_MOTOR_3);
    BspMotor_EmergencyStop(BSP_MOTOR_4);
    BspMotor_DisableAll();
    s_testMotor = BSP_MOTOR_1;
    s_state     = T1_STATE_START;
}
