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
 * 电机测试任务：两个按键让「4 个电机一起转」，验证四路电机是否都正常。
 *
 * 按键（另两个 KEY3/KEY4 归舵机测试任务）：
 *   KEY1(PA28)  4 电机一起【正转】TEST_REVS 圈
 *   KEY2(PA31)  4 电机一起【反转】TEST_REVS 圈
 *
 * 行为：每次按下 → 四电机同频同向梯形加减速转完指定圈数后自动一起停。
 * 移动进行中忽略按键，停稳后才接受下一次；细分四路共用 1/32。
 * 底层由 bsp_motor 的 BspMotorAll_MoveSteps 实现（电机1 主控斜坡，2/3/4 镜像跟随）。
 *
 * 文件结构：1.常量 → 2.诊断发布(OLED读) → 3.串口日志 → 4.任务入口。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 1. 常量
 * ------------------------------------------------------------------ */

/* 1/32 细分每圈脉冲数：1 圈 = 200 × 32 = 6400 脉冲。 */
#define STEPS_PER_REV        (6400U)

/* 巡航周期（越小越快），period=500 → 8kHz 步频 ≈ 1.25 圈/秒。 */
#define MOVE_CRUISE_PERIOD   (500U)

/* 每次按键让四电机一起转的圈数（测试用，短转即可看出是否都在动）。 */
#define TEST_REVS            (2U)

/* 每秒诊断打印节拍：20ms × 50 = 1s。 */
#define DIAG_PRINT_TICKS     (50U)

static TaskHandle_t s_motorTestTaskHandle = NULL;

/* ------------------------------------------------------------------
 * 2. 诊断发布：写 g_motorDiag 全局快照，供 PERIPH 任务在 OLED 只读显示。
 * ------------------------------------------------------------------ */

/* 全局诊断单例（定义在此，声明在 app_motor_status.h）。 */
AppMotorDiag_t g_motorDiag = { false, true, 0U };

static void MotorDiag_Publish(bool running, BspMotorDir_t dir, uint8_t param)
{
    /* 三字段一起更新，临界区保证 PERIPH 读到的是一致快照（非撕裂帧）。 */
    taskENTER_CRITICAL();
    g_motorDiag.running    = running;
    g_motorDiag.dirForward = (dir == MOTOR_DIR_FORWARD);
    g_motorDiag.param      = param;
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------------
 * 3. 串口日志（UART0 递归锁保证整行原子）。
 * ------------------------------------------------------------------ */

/* 事件日志：格式 "MOTORx4 <事件> -> RUN/STOP FWD/REV <圈数> rev"。 */
static void MotorLog_Event(const char *evt, uint8_t revs, BspMotorDir_t dir,
                           bool running)
{
    BspUart0_Lock();
    BspUart0_SendString("MOTORx4 ");
    BspUart0_SendString(evt);
    BspUart0_SendString(running ? " -> RUN " : " -> STOP ");
    BspUart0_SendString((dir == MOTOR_DIR_FORWARD) ? "FWD " : "REV ");
    BspUart0_SendUint((uint32_t)revs);
    BspUart0_SendString(" rev\r\n");
    BspUart0_Unlock();
}

/* 周期诊断（每秒）：运行状态、剩余步数、当前周期（读电机1主控，四路同步）。 */
static void MotorLog_Periodic(void)
{
    BspUart0_Lock();
    BspUart0_SendString("MOTORx4 DIAG run=");
    BspUart0_SendByte(BspMotor1_IsStopped() ? (uint8_t)'0' : (uint8_t)'1');
    BspUart0_SendString(" left=");
    BspUart0_SendUint(BspMotor1_GetRemainingSteps());
    BspUart0_SendString(" per=");
    BspUart0_SendUint(BspMotor1_GetCurPeriod());
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/* ------------------------------------------------------------------
 * 4. 任务入口：轮询 KEY1/KEY2，触发四电机一起定圈旋转。
 * ------------------------------------------------------------------ */

/* 启动一次四电机一起转：设向→使能驱动→发 MoveSteps（非阻塞，ISR 后台跑）。 */
static void MotorTest_Start(BspMotorDir_t dir)
{
    BspTmc_EnableAll();
    BspMotorAll_MoveSteps((uint32_t)TEST_REVS * STEPS_PER_REV, MOVE_CRUISE_PERIOD, dir);
    MotorDiag_Publish(true, dir, (uint8_t)TEST_REVS);
    MotorLog_Event("key start", (uint8_t)TEST_REVS, dir, true);
}

static void AppMotorTestTask_Entry(void *argument)
{
    bool key1Prev, key2Prev;
    bool key1Now, key2Now;
    uint32_t diagTick = 0U;
    TickType_t lastWakeTime;

    (void)argument;

    /* 细分四路共用，整机只需设一次；上电停在待机，等按键触发。 */
    BspTmc_SetMicrostep(TMC_MICROSTEP_32);
    MotorDiag_Publish(false, MOTOR_DIR_FORWARD, 0U);

    BspUart0_Lock();
    BspUart0_SendString(
        "MOTOR test: K1=4 motors FWD 2rev, K2=4 motors REV 2rev (1/32, 6400p/rev)\r\n");
    BspUart0_Unlock();

    /* 用真实电平初始化按键基线，避免上电误判按下沿。 */
    key1Prev = BspKey_IsPressed(BSP_KEY_1);
    key2Prev = BspKey_IsPressed(BSP_KEY_2);

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        key1Now = BspKey_IsPressed(BSP_KEY_1);
        key2Now = BspKey_IsPressed(BSP_KEY_2);

        /* 只有停稳时才接受按键按下沿，避免打断进行中的定圈移动。 */
        if (BspMotor1_IsStopped()) {
            if (key1Now && !key1Prev) {
                MotorTest_Start(MOTOR_DIR_FORWARD);
            } else if (key2Now && !key2Prev) {
                MotorTest_Start(MOTOR_DIR_REVERSE);
            }
        }

        key1Prev = key1Now;
        key2Prev = key2Now;

        /* 移动完成后发布停止快照并打印一次。 */
        if (BspMotor1_IsStopped() && g_motorDiag.running) {
            MotorDiag_Publish(false, MOTOR_DIR_FORWARD, 0U);
            MotorLog_Event("done", 0U, MOTOR_DIR_FORWARD, false);
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
