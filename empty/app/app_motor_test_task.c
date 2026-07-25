#include "app_motor_test_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_motor_status.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

/* ==================================================================
 * 电机测试任务（默认禁用，按键已让给 UIMENU；仅用于单独验证四路电机）。
 *
 * 演示 v1.9 四路独立驱动接口：
 *   KEY1(PA28)  四电机各自【正转】TEST_REVS 圈（分别调 BspMotor_MoveSteps）
 *   KEY2(PA31)  四电机各自【反转】TEST_REVS 圈
 *
 * 每次按下 → 四路各自定距移动、梯形加减速、走完自动停；移动中忽略按键。
 * 因四路完全独立，这里只是"同时下发相同命令"，改成不同圈数/转速即可看出彼此独立。
 * 运行状态发布到 g_motorDiag（取电机1为代表），供 PERIPH 任务在 OLED 只读显示。
 *
 * 文件结构：1.常量 → 2.诊断发布 → 3.串口日志 → 4.任务入口。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 1. 常量
 * ------------------------------------------------------------------ */

/* 每次按键让电机转的圈数（短转即可看出是否都在动）。 */
#define TEST_REVS            (2U)

/* 测试巡航转速（RPM）。1/32 细分下安全范围约 5~300 RPM。 */
#define TEST_RPM             (120U)

/* 每秒诊断打印节拍：20ms × 50 = 1s。 */
#define DIAG_PRINT_TICKS     (50U)

static TaskHandle_t s_motorTestTaskHandle = NULL;

/* ------------------------------------------------------------------
 * 2. 诊断发布：写 g_motorDiag 全局快照，供 PERIPH 任务在 OLED 只读显示。
 * ------------------------------------------------------------------ */

/* 全局诊断单例（定义在此，声明在 app_motor_status.h）。 */
AppMotorDiag_t g_motorDiag = { false, true, 0U };

static void MotorDiag_Publish(bool running, bool forward, uint8_t param)
{
    /* 三字段一起更新，临界区保证 PERIPH 读到的是一致快照（非撕裂帧）。 */
    taskENTER_CRITICAL();
    g_motorDiag.running    = running;
    g_motorDiag.dirForward = forward;
    g_motorDiag.param      = param;
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------------
 * 3. 串口日志（UART0 递归锁保证整行原子）。
 * ------------------------------------------------------------------ */

static void MotorLog_Event(const char *evt, uint8_t revs, bool forward, bool running)
{
    BspUart0_Lock();
    BspUart0_SendString("MOTORx4 ");
    BspUart0_SendString(evt);
    BspUart0_SendString(running ? " -> RUN " : " -> STOP ");
    BspUart0_SendString(forward ? "FWD " : "REV ");
    BspUart0_SendUint((uint32_t)revs);
    BspUart0_SendString(" rev\r\n");
    BspUart0_Unlock();
}

/* 周期诊断（每秒）：以电机1为代表报运行状态与剩余步数。 */
static void MotorLog_Periodic(void)
{
    BspUart0_Lock();
    BspUart0_SendString("MOTORx4 DIAG m1_run=");
    BspUart0_SendByte(BspMotor_IsStopped(BSP_MOTOR_1) ? (uint8_t)'0' : (uint8_t)'1');
    BspUart0_SendString(" m1_left=");
    BspUart0_SendUint(BspMotor_GetRemainingSteps(BSP_MOTOR_1));
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/* ------------------------------------------------------------------
 * 4. 任务入口：轮询 KEY1/KEY2，触发四电机各自定圈旋转。
 * ------------------------------------------------------------------ */

/* 启动一次四电机独立定距移动：使能→四路各发 MoveSteps（非阻塞，ISR 后台跑）。 */
static void MotorTest_Start(bool forward)
{
    int32_t steps = (int32_t)((uint32_t)TEST_REVS * BspMotor_StepsPerRev());
    if (!forward) {
        steps = -steps;   /* 负号 = 反转 */
    }

    BspMotor_EnableAll();
    BspMotor_MoveSteps4(steps, steps, steps, steps, TEST_RPM);
    MotorDiag_Publish(true, forward, (uint8_t)TEST_REVS);
    MotorLog_Event("key start", (uint8_t)TEST_REVS, forward, true);
}

static void AppMotorTestTask_Entry(void *argument)
{
    bool key1Prev, key2Prev;
    bool key1Now, key2Now;
    uint32_t diagTick = 0U;
    TickType_t lastWakeTime;

    (void)argument;

    /* 细分四路共用，整机只需设一次；上电停在待机，等按键触发。 */
    BspMotor_SetMicrostep(TMC_MICROSTEP_32);
    MotorDiag_Publish(false, true, 0U);

    BspUart0_Lock();
    BspUart0_SendString(
        "MOTOR test: K1=4 motors FWD 2rev, K2=4 motors REV 2rev (1/32, 120rpm)\r\n");
    BspUart0_Unlock();

    /* 用真实电平初始化按键基线，避免上电误判按下沿。 */
    key1Prev = BspKey_IsPressed(BSP_KEY_1);
    key2Prev = BspKey_IsPressed(BSP_KEY_2);

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        key1Now = BspKey_IsPressed(BSP_KEY_1);
        key2Now = BspKey_IsPressed(BSP_KEY_2);

        /* 只有电机1停稳时才接受按键（四路同时下发，用电机1代表判断）。 */
        if (BspMotor_IsStopped(BSP_MOTOR_1)) {
            if (key1Now && !key1Prev) {
                MotorTest_Start(true);
            } else if (key2Now && !key2Prev) {
                MotorTest_Start(false);
            }
        }

        key1Prev = key1Now;
        key2Prev = key2Now;

        /* 移动完成后发布停止快照并打印一次。 */
        if (BspMotor_IsStopped(BSP_MOTOR_1) && g_motorDiag.running) {
            MotorDiag_Publish(false, true, 0U);
            MotorLog_Event("done", 0U, true, false);
        }

        /* 每秒诊断打印。 */
        if (++diagTick >= DIAG_PRINT_TICKS) {
            diagTick = 0U;
            MotorLog_Periodic();
        }

        vTaskDelayUntil(&lastWakeTime, APP_MOTOR_KEY_POLL_TICKS);
    }
}

void AppMotorTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppMotorTestTask_Entry,
                      "MOTORTEST",
                      APP_MOTOR_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_MOTOR_TEST_TASK_PRIORITY,
                      &s_motorTestTaskHandle);
    configASSERT(ret == pdPASS);
}
